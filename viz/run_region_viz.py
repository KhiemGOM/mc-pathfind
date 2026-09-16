"""Build a real-terrain-only viewer with strict route validation.

The default invocation targets ``regions/k1`` and writes both ``rviz_k1.html``
and ``rviz_k1_report.json``.  It deliberately does not publish a partial
viewer: if any selected route fails, the JSON report records its exact region,
start/goal, validation stage, and failure details before the process exits
non-zero.
"""

import argparse
import json
import os
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from build_visualization import (
    _find_region_route, _region_key, adaptive_chunk_range, build_html, crop_and_export,
)
from mca_convert import convert_region
from pathfind import get_neighbors, weighted_astar
from reachability import walkable_flood_fill

# This script's whole point is STRICT, self-consistent validation (see the
# module docstring): solve, then independently re-derive and re-check every
# edge against the same rules. That only means something if both halves use
# the same model -- so unlike build_visualization.py (which solves through
# the faster Java port for demo generation), this one deliberately stays on
# pathfind.py's own weighted_astar for both halves. Java has since diverged
# from this Python model in real ways (BRIDGE_UP has no Python equivalent;
# the MINE cost formula was refined in Java only -- see java/README.md and
# root README.md's pathfind.py note) -- validating a Java-solved path
# against Python's get_neighbors would spuriously fail on a correct route,
# not catch a real bug. Real-terrain search difficulty varies wildly enough
# that a single fixed epsilon is either too tight (blows the budget on hard
# regions) or needlessly loose on easy ones, hence the small-to-large
# epsilon/budget ladder below, tried in order until one finds a path.
REAL_TERRAIN_EPSILON_LADDER = (2.0, 3.5, 5.0, 8.0)
REAL_TERRAIN_EXPANSION_LADDER = (40_000, 100_000, 200_000, 300_000)


def solve_with_epsilon_ladder(world, start, goal, blocks_available=32,
                               epsilon_ladder=REAL_TERRAIN_EPSILON_LADDER,
                               max_expansions=REAL_TERRAIN_EXPANSION_LADDER):
    """Try weighted_astar at each epsilon in epsilon_ladder (ascending),
    stopping at the first one that finds a path. Returns (path, cost,
    actions, expansions, epsilon_used, attempts); if every rung fails, path
    and epsilon_used are None."""
    attempts = []
    path = cost = actions = expansions = epsilon_used = None
    for epsilon, budget in zip(epsilon_ladder, max_expansions):
        path, cost, actions, expansions = weighted_astar(
            world, start, goal, blocks_available=blocks_available,
            epsilon=epsilon, max_expansions=budget,
        )
        attempts.append({
            "epsilon": epsilon, "max_expansions": budget,
            "expansions": expansions, "found": path is not None,
        })
        if path is not None:
            epsilon_used = epsilon
            break
    return path, cost, actions, expansions, epsilon_used, attempts


Y_RANGE = (0, 128)
BLOCKS_AVAILABLE = 32
MAX_EXPANSIONS = REAL_TERRAIN_EXPANSION_LADDER[-1]
EPSILON_LADDER = REAL_TERRAIN_EPSILON_LADDER
EXPANSION_LADDER = REAL_TERRAIN_EXPANSION_LADDER


class RouteValidationError(RuntimeError):
    """A route failed a check that must be surfaced to the user."""


def validate_route(world, path, actions, cost, start, goal):
    """Validate solver output independently, including every returned edge."""
    if path is None:
        raise RouteValidationError("A* returned no path")
    if path[0][:3] != start:
        raise RouteValidationError(f"path starts at {path[0][:3]}, expected {start}")
    if path[-1][:3] != goal:
        raise RouteValidationError(f"path ends at {path[-1][:3]}, expected {goal}")
    if len(actions) != len(path) - 1:
        raise RouteValidationError(
            f"path has {len(path)} states but {len(actions)} actions"
        )

    recomputed_cost = 0.0
    for index, (state, next_state, action) in enumerate(zip(path, path[1:], actions)):
        matching_edge = None
        for candidate, edge_cost, edge_action in get_neighbors(world, state):
            if candidate == next_state and edge_action == action:
                matching_edge = edge_cost
                break
        if matching_edge is None:
            raise RouteValidationError(
                f"invalid edge at step {index}: {state[:3]} --{action}--> {next_state[:3]}"
            )
        recomputed_cost += matching_edge

    if abs(recomputed_cost - cost) > 1e-6:
        raise RouteValidationError(
            f"reported cost {cost:.9f} differs from edge total {recomputed_cost:.9f}"
        )


def run_region(mca_file):
    """Convert, select, solve, and validate one region; return scenario/report."""
    key = _region_key(mca_file)
    chunk_x_range, chunk_z_range = adaptive_chunk_range(mca_file)
    # anvil-parser expects a string path (it calls .read on its argument).
    world, origin = convert_region(
        str(mca_file), chunk_x_range=chunk_x_range, chunk_z_range=chunk_z_range,
        y_range=Y_RANGE,
    )
    start, goal = _find_region_route(world)
    report = {
        "region": str(mca_file),
        "scenario": key,
        "origin": origin,
        "chunk_range": [chunk_x_range, chunk_z_range],
        "y_range": Y_RANGE,
        "start": start,
        "goal": goal,
    }
    if start is None or goal is None:
        raise RouteValidationError("no supported-air start/goal could be selected")

    # The selector is walk-only; verify its premise before running costed A*.
    walkable = walkable_flood_fill(world, start, max_states=500_000)
    report["walkable_states_checked"] = len(walkable)
    report["goal_walk_reachable"] = goal in walkable
    if goal not in walkable:
        raise RouteValidationError(
            f"selected goal {goal} is not walk-reachable from start {start} "
            f"after checking {len(walkable)} states"
        )

    path, cost, actions, expansions, epsilon_used, attempts = solve_with_epsilon_ladder(
        world, start, goal, blocks_available=BLOCKS_AVAILABLE,
        epsilon_ladder=EPSILON_LADDER, max_expansions=EXPANSION_LADDER,
    )
    report["epsilon_attempts"] = attempts
    report["expansions"] = expansions
    if path is None:
        raise RouteValidationError(
            f"A* found no path for {start} -> {goal} even at the loosest "
            f"epsilon tried ({EPSILON_LADDER[-1]}), {MAX_EXPANSIONS}-expansion "
            f"cap each (ladder: {list(EPSILON_LADDER)})"
        )

    validate_route(world, path, actions, cost, start, goal)
    report.update({
        "status": "passed",
        "epsilon_used": epsilon_used,
        "cost": cost,
        "path_nodes": len(path),
        "action_counts": dict(sorted(Counter(actions).items())),
    })
    scenario = crop_and_export(world, path, actions, start, goal, cost, expansions)
    return key, scenario, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mca-dir", default=str(_REPO_ROOT / "regions" / "k1"))
    parser.add_argument(
        "--output", default=None,
        help="defaults to <this script's dir>/rviz_<mca-dir folder name>.html",
    )
    parser.add_argument(
        "--report", default=None,
        help="defaults to <this script's dir>/rviz_<mca-dir folder name>_report.json",
    )
    parser.add_argument(
        "--workers", type=int, default=None,
        help="parallel worker processes (default: min(cpu_count, region count)); "
             "regions are fully independent so this scales close to linearly",
    )
    args = parser.parse_args()

    region_dir = Path(args.mca_dir)
    here = Path(__file__).resolve().parent
    # Default output names track whatever folder is passed in, so pointing
    # this at a newly-dropped region set (e.g. --mca-dir regions/k2) produces
    # its own rviz_k2.html rather than requiring the k1-specific defaults to
    # be hand-edited every time a new batch of regions shows up. Anchored to
    # this script's own directory (not CWD) so the output lands next to
    # pathfind_viz_multi.html regardless of where this is invoked from.
    output = args.output or str(here / f"rviz_{region_dir.name}.html")
    report_path = args.report or str(here / f"rviz_{region_dir.name}_report.json")

    mca_files = sorted(region_dir.glob("*.mca")) if region_dir.is_dir() else []
    if not mca_files:
        raise SystemExit(f"no .mca files found in {region_dir}")

    workers = args.workers or min(len(mca_files), os.cpu_count() or 1)

    scenarios = {}
    reports = []
    failures = []
    # Each region file is converted, routed, and solved completely
    # independently -- no shared state -- so this is embarrassingly
    # parallel across processes (threads wouldn't help: the NBT decode and
    # A* search are both CPU-bound pure Python, held under the GIL).
    with ProcessPoolExecutor(max_workers=workers) as pool:
        future_to_file = {pool.submit(run_region, mca_file): mca_file for mca_file in mca_files}
        for future in as_completed(future_to_file):
            mca_file = future_to_file[future]
            try:
                key, scenario, report = future.result()
                scenarios[key] = scenario
                reports.append(report)
                print(
                    f"PASS {mca_file.name}: {report['start']} -> {report['goal']}; "
                    f"{report['path_nodes']} nodes, {report['cost']:.2f}s, "
                    f"{report['expansions']} expansions, eps={report['epsilon_used']}"
                )
            except Exception as exc:
                failure = {"region": str(mca_file), "status": "failed", "error": str(exc)}
                failures.append(failure)
                print(f"FAIL {mca_file.name}: {failure['error']}")

    report_data = {"passed": reports, "failed": failures}
    Path(report_path).write_text(json.dumps(report_data, indent=2), encoding="utf-8")

    # Publish whatever passed regardless of failures elsewhere in the batch:
    # a "failure" here is often just a region with no real generated terrain
    # (see adaptive_chunk_range/_generated_chunk_bbox) -- that's expected
    # input data, not a solver bug, and it shouldn't withhold the viewer for
    # every region that DID work. The report file and non-zero exit still
    # surface failures for anyone scripting around this.
    if scenarios:
        build_html(scenarios, output_path=output)
        print(f"{output} written with {len(scenarios)} region(s)")
    else:
        print(f"no regions passed -- {output} not written")

    print(f"validation report written to {report_path}")
    if failures:
        raise SystemExit(
            f"{len(failures)} of {len(mca_files)} region(s) failed; "
            f"details written to {report_path}"
        )


if __name__ == "__main__":
    main()
