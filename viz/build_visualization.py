"""
Regenerates all demo scenarios and rebuilds pathfind_viz_multi.html from scratch.

Run this after any change to world.py (world generation) or the Java solver
in java/core/ (java/README.md has the build command if java/out/ needs a
rebuild first) to keep the visualizer in sync -- forgetting this step was
the single most common source of "stale" bugs during development (editing
the solver but looking at an old cached scenario in the HTML). Solving
itself always goes through the Java port (see solve_via_java below), not
pathfind.py's weighted_astar -- this script only owns world generation/
conversion and rendering the result.

Usage (from anywhere -- paths below are relative to the repo root either way):
    python3 viz/build_visualization.py [--mca-dir regions] [--mca-file PATH]

Every ``.mca`` file in --mca-dir becomes its own real-terrain scenario.  Extra
--mca-file arguments are also accepted for one-off regions.  Each region gets
an automatically selected walkable start/goal pair, so the visualizer is not
tied to coordinates that only happen to work in one saved world.
"""

import argparse
import json
import os
import struct
import subprocess
import sys
import tempfile
from pathlib import Path
import numpy as np

# This module lives in viz/, one level under the repo root where world.py,
# pathfind.py, mca_convert.py, and reachability.py live -- add that root to
# sys.path so those imports resolve regardless of which directory this is
# invoked from (repo root, viz/, or anywhere else).
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from world import (
    generate_world, generate_wall_world, generate_bridge_world,
    generate_cave_shortcut_world, standing_y,
    AIR, DIRT, STONE, OBSIDIAN, BEDROCK,
)

# Every scenario is solved by the Java port (dev.mcpathfind.core), not
# pathfind.py's weighted_astar -- the Java solver is roughly two orders of
# magnitude faster per expansion (dense state ids + a decrease-key heap vs.
# Python dicts/heapq), which is the entire reason real-terrain scenarios
# used to need an epsilon-retry ladder here (see the module's old docstring
# note) and made every visualizer rebuild slow even for a template-only
# change. See java/README.md for what the port does and doesn't cover
# (search only -- .mca parsing/replanning/viz all stay Python-side, so this
# script still owns world generation/conversion and hands the finished
# voxel array to Java purely to search it).
JAVA_CLASSES_DIR = _REPO_ROOT / "java" / "out"
JAVA_FASTUTIL_JAR = _REPO_ROOT / "java" / "fastutil.jar"
WBIN_MAGIC = b"MCPW"
WBIN_VERSION = 1


def _pack_wbin(world, start, goal, blocks_available, origin=(0, 0, 0)):
    """Same on-disk format as export_world_bin.py's export_world_bin (kept
    in sync manually -- duplicated rather than imported to avoid a circular
    import, since export_world_bin.py itself imports from this module)."""
    size_x, size_y, size_z = world.shape
    header = struct.pack(
        "<4s14i", WBIN_MAGIC, WBIN_VERSION,
        size_x, size_y, size_z,
        origin[0], origin[1], origin[2],
        start[0], start[1], start[2],
        goal[0], goal[1], goal[2],
        blocks_available,
    )
    return header + world.tobytes()


def solve_via_java(world, start, goal, blocks_available=32, epsilon=1.5, max_expansions=8_000_000):
    """
    Solve via the Java port's BidirectionalWeightedAStar (experiments/DumpPathJson.java)
    instead of pathfind.py's weighted_astar. Returns (path, cost, actions, expansions)
    in the same shape weighted_astar did, so call sites are a drop-in swap:
    path is a list of (x,y,z,blocks) tuples, actions a list of action-name strings,
    or (None, None, None, expansions) if no path was found within max_expansions.

    Requires java/out/ (compiled classes) and java/fastutil.jar to already
    exist -- from java/, once: `curl -sL -o fastutil.jar <maven-central-url>`
    then `javac -cp fastutil.jar -d out $(find core/src/main/java -name '*.java') ../experiments/DumpPathJson.java`
    (see java/README.md). Not auto-built here so a stale/missing build fails
    loudly instead of silently recompiling mid-run.
    """
    if not JAVA_CLASSES_DIR.exists() or not JAVA_FASTUTIL_JAR.exists():
        raise RuntimeError(
            f"Java solver not built -- expected {JAVA_CLASSES_DIR} and {JAVA_FASTUTIL_JAR}. "
            "From java/: javac -cp fastutil.jar -d out $(find core/src/main/java -name '*.java') "
            "../experiments/DumpPathJson.java (fetch fastutil.jar from Maven Central first if missing; see java/README.md)."
        )

    fd, wbin_path = tempfile.mkstemp(suffix=".wbin")
    os.close(fd)
    try:
        Path(wbin_path).write_bytes(_pack_wbin(world, start, goal, blocks_available))
        classpath = os.pathsep.join([str(JAVA_CLASSES_DIR), str(JAVA_FASTUTIL_JAR)])
        result = subprocess.run(
            ["java", "-cp", classpath, "DumpPathJson", wbin_path,
             "--epsilon", str(epsilon), "--max-expansions", str(max_expansions)],
            capture_output=True, text=True, timeout=120,
        )
        if result.returncode != 0:
            raise RuntimeError(f"DumpPathJson failed (exit {result.returncode}): {result.stderr}")
        data = json.loads(result.stdout)
    finally:
        os.unlink(wbin_path)

    if not data.get("found"):
        return None, None, None, data.get("expansions", 0)
    path = [tuple(p) for p in data["path"]]
    return path, data["cost"], data["actions"], data["expansions"]


# Real-terrain search difficulty varies wildly (a few dozen blocks of
# connected cave vs. a 500+-block open cavern), which used to need an
# epsilon-retry ladder here to keep a too-tight epsilon from blowing an
# enormous expansion budget on pathfind.py's Python solver (measured
# directly on this project's worst-case region: eps=2.0 at a flat
# 300k-expansion cap ran the full budget and still failed, costing ~130s).
# solve_via_java doesn't need that workaround -- the Java port is fast
# enough that a single eps=1.5 attempt with a generous budget handles every
# region this project has thrown at it in seconds, so real terrain now uses
# the exact same one-shot call as every synthetic scenario.
REAL_TERRAIN_EPSILON = 1.5
REAL_TERRAIN_MAX_EXPANSIONS = 8_000_000


def export_scenario(world, path_or_executed, actions, start, goal, cost, expansions, replans=None):
    sx, sy, sz = world.shape
    surface_blocks = []
    for x in range(sx):
        for z in range(sz):
            col = world[x, :, z]
            solid_ys = np.nonzero(col != AIR)[0]
            for y in solid_ys:
                exposed = False
                for dx, dy, dz in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]:
                    nx, ny, nz = x + dx, y + dy, z + dz
                    if not (0 <= nx < sx and 0 <= ny < sy and 0 <= nz < sz) or world[nx, ny, nz] == AIR:
                        exposed = True
                        break
                if exposed:
                    surface_blocks.append([int(x), int(y), int(z), int(world[x, y, z])])
    return {
        "dims": [sx, sy, sz], "blocks": surface_blocks,
        "path": [[int(p[0]), int(p[1]), int(p[2]), int(p[3])] for p in path_or_executed],
        "actions": list(actions), "start": list(start), "goal": list(goal),
        "cost": cost, "expansions": expansions, "replans": replans or [],
    }


def crop_and_export(world, path, actions, start, goal, cost, expansions, pad=4):
    """Like export_scenario, but crops the world to a padded bounding box
    around the path first -- needed for large real-terrain worlds so the
    exported JSON doesn't include the entire (possibly huge) map."""
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    zs = [p[2] for p in path]
    x0, x1 = max(0, min(xs) - pad), min(world.shape[0], max(xs) + pad + 1)
    y0, y1 = max(0, min(ys) - pad), min(world.shape[1], max(ys) + pad + 1)
    z0, z1 = max(0, min(zs) - pad), min(world.shape[2], max(zs) + pad + 1)
    cropped = world[x0:x1, y0:y1, z0:z1]

    path_local = [[p[0] - x0, p[1] - y0, p[2] - z0, p[3]] for p in path]
    start_local = [start[0] - x0, start[1] - y0, start[2] - z0]
    goal_local = [goal[0] - x0, goal[1] - y0, goal[2] - z0]
    return export_scenario(cropped, path_local, actions, start_local, goal_local, cost, expansions)


def _find_region_route(world, seed_stride=8):
    """Return a walkable start/goal pair that spans the largest connected
    walkable area in this region, as far apart as a walk-only route allows.

    Region files do not share a guaranteed player location, and a route
    picked from just the first candidate spot (or "far from an arbitrary
    start") can land in a small disconnected pocket -- e.g. a 3-node, 5-block
    scenario that says nothing about how the solver behaves across real
    terrain. Instead:

      1. Gather standing-spot candidates at the dominant supported elevation.
         (Restricting to the dominant y-level is intentional: the first
         supported-air voxel found unrestricted tends to be a cave floor,
         which produces a high-branching 3-D search that can exhaust the A*
         cap even when an open surface route is available nearby. The most
         common supported y-level is this region's walkable surface in these
         Nether captures, and gives a representative, tractable route.)
      2. Flood-fill from a spread of those candidates (deduped by a global
         seen-set, so cost scales with total cells visited once, not with
         the number of candidates) and keep the largest connected component
         -- the area the route should actually span.
      3. Within that component, run a double-sweep BFS (diameter_endpoints)
         to find two points that are maximally far apart by actual walked
         hop count, not Manhattan distance, so the route is a genuine
         cross-terrain traversal rather than a short local hop.
    """
    from reachability import largest_walkable_component, diameter_endpoints

    supported_air = ((world[:, 1:, :] == AIR) &
                     np.isin(world[:, :-1, :], (DIRT, STONE, OBSIDIAN, BEDROCK)))
    candidates = np.argwhere(supported_air)
    if len(candidates) == 0:
        return None, None

    levels = candidates[:, 1]
    surface_y_minus_one = int(np.bincount(levels).argmax())
    surface_candidates = candidates[levels == surface_y_minus_one]
    seeds = [(int(x), int(y) + 1, int(z)) for x, y, z in surface_candidates[::seed_stride]]
    if not seeds:
        seeds = [(int(x), int(y) + 1, int(z)) for x, y, z in surface_candidates]

    best_seed, best_component = largest_walkable_component(world, seeds)
    if best_seed is None:
        return None, None

    start, goal, _ = diameter_endpoints(world, best_seed, max_states=len(best_component) + 1)
    return start, goal


def _generated_chunk_bbox(mca_path, region_chunks=32):
    """
    Scan an Anvil file for which chunks are actually fully generated, and
    return the tight (chunk_x_range, chunk_z_range) bounding box around
    them, or None if nothing is fully generated at all.

    A chunk having a region-file entry (Region.chunk_location() sectors !=
    0) is NOT the same as having real terrain: Minecraft's generation
    pipeline writes a stub entry -- Status "structure_starts" or similar,
    empty Sections, no blocks -- for chunks merely touched by a neighbor's
    structure generation reaching across the chunk border, without ever
    promoting them to "full". Observed directly on this project's own test
    data: several region files had 100% stub chunks (an apparent "highway
    tunnel" shape that was actually just the stub border around a generated
    area elsewhere), and even a mostly-real region had ~40% stub chunks
    interspersed with the real ones, corrupting a naive existence-only bbox
    with dead air. Checking chunk.data["Status"] == "full" (still no block
    section decode, just the chunk's NBT header) costs well under a second
    for a full 1024-chunk file and gives the tight, correct bounding box.
    """
    import anvil

    region = anvil.Region.from_file(str(mca_path))
    xs, zs = [], []
    for cz in range(region_chunks):
        for cx in range(region_chunks):
            _, sectors = region.chunk_location(cx, cz)
            if sectors == 0:
                continue
            try:
                chunk = anvil.Chunk.from_region(region, cx, cz)
            except Exception:
                continue
            if str(chunk.data.get("Status", "")) != "full":
                continue
            xs.append(cx)
            zs.append(cz)
    if not xs:
        return None
    return (min(xs), max(xs) + 1), (min(zs), max(zs) + 1)


def adaptive_chunk_range(mca_path, span=16, region_chunks=32):
    """
    Return the (chunk_x_range, chunk_z_range) window to convert for this
    region file: the exact generated-chunk bounding box (see
    _generated_chunk_bbox) when detectable, falling back to a `span`-wide
    window biased toward whichever edge is closest to the origin region
    (r.0.0) if the header scan finds nothing (e.g. a non-Anvil-shaped input).
    Real single/multiplayer worlds generate outward from spawn, so a region
    file at (rx, rz) != (0, 0) is typically only populated near the edge
    adjacent to the origin region.
    """
    bbox = _generated_chunk_bbox(mca_path, region_chunks=region_chunks)
    if bbox is not None:
        return bbox

    parts = Path(mca_path).stem.split(".")  # "r.-1.0" -> ["r", "-1", "0"]
    rx, rz = int(parts[1]), int(parts[2])

    def axis_start(coord):
        if coord < 0:
            return region_chunks - span   # near the high-index edge (closest to origin)
        if coord > 0:
            return 0                       # near the low-index edge (closest to origin)
        return (region_chunks - span) // 2  # origin region: centered is fine, fully generated

    x0 = axis_start(rx)
    z0 = axis_start(rz)
    return (x0, x0 + span), (z0, z0 + span)


def _region_key(mca_path):
    """Stable scenario key from an Anvil filename such as r.-1.0.mca.

    Includes an identifying ancestor directory name (e.g. "k1", "long1", or
    a save/match id) since multiple saved worlds reuse the same r.X.Z.mca
    region filenames -- keying off the stem alone would collide and
    silently drop one world's scenario. _identifying_ancestor walks up past
    generic path segments ("region", "DIM-1") that carry no identifying
    information on their own (real save layouts route every dimension's
    regions through a folder literally named "region", so using the
    immediate parent alone collides across every save).
    """
    folder = _identifying_ancestor(mca_path)
    stem = Path(mca_path).stem.replace(".", "_").replace("-", "neg")
    return f"real_{folder}_{stem}" if folder else f"real_{stem}"


_GENERIC_PATH_SEGMENTS = {"region", "dim-1", "dim1", "dim_1"}


def _identifying_ancestor(mca_path):
    """First ancestor directory name that isn't a generic dimension/region
    folder -- e.g. for ".../saves/mcsrranked #BX7TX6pyc/DIM-1/region/r.0.0.mca"
    this is "mcsrranked #BX7TX6pyc" (trimmed to just "BX7TX6pyc" if it has
    the "mcsrranked #<id>" shape), not "region" (every save has one of
    those, so it identifies nothing)."""
    for parent in Path(mca_path).parents:
        name = parent.name
        if not name or name.lower() in _GENERIC_PATH_SEGMENTS:
            continue
        if name.lower().startswith("mcsrranked #"):
            return name.split("#", 1)[1]
        return name
    return ""


def add_real_terrain_scenarios(scenarios, mca_files):
    """Convert every requested region and add a route (or inspection) tab.

    A single problematic region (no walkable spot in the sampled chunks, or
    a route the bounded search can't close within its expansion cap -- both
    real possibilities on real terrain, e.g. a chunk range that's mostly a
    lava ocean) is skipped with a warning rather than aborting the whole
    build, so one bad region doesn't take down every other scenario.
    """
    from mca_convert import convert_region

    for mca_file in mca_files:
        key = _region_key(mca_file)
        print(f"Converting real terrain from {mca_file} ...")
        chunk_x_range, chunk_z_range = adaptive_chunk_range(mca_file)
        world_real, origin = convert_region(
            mca_file, chunk_x_range=chunk_x_range, chunk_z_range=chunk_z_range,
            y_range=(0, 128),
        )
        start_r, goal_r = _find_region_route(world_real)
        if start_r is None:
            print(f"  {key}: skipped -- no supported air cell in selected chunks")
            continue

        path_r, cost_r, actions_r, exp_r = solve_via_java(
            world_real, start_r, goal_r, blocks_available=32,
            epsilon=REAL_TERRAIN_EPSILON, max_expansions=REAL_TERRAIN_MAX_EXPANSIONS,
        )
        if path_r is None:
            print(f"  {key}: skipped -- A* found no path within "
                  f"{REAL_TERRAIN_MAX_EXPANSIONS} expansions at eps={REAL_TERRAIN_EPSILON} "
                  f"from {start_r} to {goal_r}")
            continue

        scenarios[key] = crop_and_export(
            world_real, path_r, actions_r, start_r, goal_r, cost_r, exp_r,
        )
        print(f"  {key}: cost={cost_r:.2f} nodes={len(path_r)} origin={origin}")


def build_all_scenarios(mca_files=None):
    scenarios = {}

    # 1. Mixed terrain: sprint/mine/bridge/climb/fall all in play
    w1, h1 = generate_world(size_x=40, size_y=20, size_z=40, seed=0)
    s1 = (2, standing_y(h1, 2, 5), 5)
    g1 = (35, standing_y(h1, 35, 5), 5)
    p1, c1, a1, e1 = solve_via_java(w1, s1, g1, blocks_available=64, epsilon=1.5)
    scenarios["mixed_terrain"] = export_scenario(w1, p1, a1, s1, g1, c1, e1)
    print(f"mixed_terrain: cost={c1:.2f} nodes={len(p1)}")

    # 2. Giant unbreakable wall with a single gap -- "find the open terrain"
    w2, h2 = generate_wall_world(size_x=60, size_y=20, size_z=60, wall_thickness=5, gap_x=(27, 33), seed=1)
    s2 = (2, standing_y(h2, 2, 5), 5)
    g2 = (55, standing_y(h2, 55, 5), 5)
    p2, c2, a2, e2 = solve_via_java(w2, s2, g2, blocks_available=10, epsilon=1.5)
    scenarios["giant_wall"] = export_scenario(w2, p2, a2, s2, g2, c2, e2)
    print(f"giant_wall: cost={c2:.2f} nodes={len(p2)}")

    # 5. Cave shortcut where mining a short plug beats a long detour
    w4, h4 = generate_cave_shortcut_world(
        size_x=60, size_y=20, size_z=80, wall_thickness=6,
        detour_gap_z=(2, 8), cave_z=(38, 42), cave_mine_depth=3, seed=3,
    )
    s4 = (2, standing_y(h4, 2, 40), 40)
    g4 = (57, standing_y(h4, 57, 40), 40)
    p4, c4, a4, e4 = solve_via_java(w4, s4, g4, blocks_available=20, epsilon=1.3)
    scenarios["cave_shortcut_wins"] = export_scenario(w4, p4, a4, s4, g4, c4, e4)
    print(f"cave_shortcut_wins: cost={c4:.2f} nodes={len(p4)}")

    # 6. Same setup but the plug is thick enough that detouring wins instead
    w5, h5 = generate_cave_shortcut_world(
        size_x=60, size_y=20, size_z=80, wall_thickness=14,
        detour_gap_z=(35, 37), cave_z=(60, 64), cave_mine_depth=12, seed=3,
    )
    s5 = (2, standing_y(h5, 2, 61), 61)
    g5 = (57, standing_y(h5, 57, 61), 61)
    p5, c5, a5, e5 = solve_via_java(w5, s5, g5, blocks_available=20, epsilon=1.3)
    scenarios["detour_wins"] = export_scenario(w5, p5, a5, s5, g5, c5, e5)
    print(f"detour_wins: cost={c5:.2f} nodes={len(p5)}")

    # 7. Tunnel vs bridge: a below-surface route competing with a surface bridge
    w6, h6 = generate_bridge_world(size_x=50, size_y=20, size_z=20, seed=2)
    chasm_x0, chasm_x1 = 21, 29
    tunnel_floor_y = 2
    for x in range(chasm_x0, chasm_x1):
        w6[x, tunnel_floor_y, 10] = DIRT
        w6[x, tunnel_floor_y + 1, 10] = AIR
        w6[x, tunnel_floor_y + 2, 10] = AIR
        w6[x, tunnel_floor_y + 3, 10] = AIR
    mid = (chasm_x0 + chasm_x1) // 2
    for x in [mid, mid + 1]:
        w6[x, tunnel_floor_y + 1, 10] = STONE
    s6 = (2, standing_y(h6, 2, 10), 10)
    g6 = (47, standing_y(h6, 47, 10), 10)
    p6, c6, a6, e6 = solve_via_java(w6, s6, g6, blocks_available=20, epsilon=1.0)
    scenarios["tunnel_vs_bridge"] = export_scenario(w6, p6, a6, s6, g6, c6, e6)
    print(f"tunnel_vs_bridge: cost={c6:.2f} nodes={len(p6)}")

    # 8. Bridge gauntlet: a flat chasm crossing (sideways BRIDGE) followed
    # by a sheer bedrock ledge too tall for CLIMB (forces BRIDGE_UP --
    # pillaring straight up by placing a block underfoot each step). Only
    # solvable at all via the Java port: pathfind.py's own BRIDGE branch
    # only ever gains height over LAVA (see generate_ledge_world's
    # docstring, lava=False is "EXPECTED to be genuinely unsolvable" there)
    # -- EdgeRules.bridgeUpEdge is a separate, more general mechanic with no
    # Python equivalent. Start/goal are offset in z as well as x so a
    # diagonal bridge step across the chasm corner is at least available to
    # the search if it's ever actually cheaper than going straight -- not
    # forced, since whether that's optimal here isn't obvious either way.
    ground_h = 5
    ledge_x = 55
    ledge_height = 6
    high_h = ground_h + ledge_height
    w7, _ = generate_bridge_world(size_x=70, size_y=20, size_z=20, seed=2)
    w7[ledge_x, ground_h:high_h, :] = BEDROCK
    w7[ledge_x + 1:, 0:high_h, :] = DIRT
    s7 = (2, ground_h, 6)
    g7 = (65, high_h, 14)
    p7, c7, a7, e7 = solve_via_java(w7, s7, g7, blocks_available=20, epsilon=1.0)
    scenarios["bridge_gauntlet"] = export_scenario(w7, p7, a7, s7, g7, c7, e7)
    print(f"bridge_gauntlet: cost={c7:.2f} nodes={len(p7)} actions={sorted(set(a7))}")

    if mca_files:
        add_real_terrain_scenarios(scenarios, mca_files)

    return scenarios


def build_html(scenarios, output_path="pathfind_viz_multi.html"):
    viz_data = json.dumps(scenarios)

    scenario_labels = {
        "mixed_terrain": "Mixed Terrain",
        "giant_wall": "Giant Wall",
        "cave_shortcut_wins": "Mining Wins",
        "detour_wins": "Detour Wins",
        "tunnel_vs_bridge": "Tunnel vs Bridge",
        "bridge_gauntlet": "Bridge Gauntlet",
    }
    scenario_groups = {k: "synthetic" for k in scenario_labels}
    # Real-terrain scenario keys are derived per-region at discovery time
    # (see _region_key: "real_<folder>_<r.X.Z>"), so they can't be listed
    # statically above -- auto-label anything not already named, as a short
    # "Bastion: <folder>" form (folder alone is enough to disambiguate since
    # each folder contributes at most one curated region file here).
    for key in sorted(scenarios):
        if key not in scenario_labels:
            label = key
            if key.startswith("real_"):
                rest = key[len("real_"):]
                folder = rest.split("_r_", 1)[0] if "_r_" in rest else rest
                label = f"Bastion: {folder}"
            scenario_labels[key] = label
            scenario_groups[key] = "real"
    scenario_order = [k for k in scenario_labels if k in scenarios]

    template_head = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Bastion Pathfinder Demo</title>
<style>
  html, body { margin:0; padding:0; background:#0a0a0a; overflow:hidden; font-family: -apple-system, "Segoe UI", sans-serif; }
  #canvas-wrap { position: fixed; inset: 0; }
  #tabs {
    position: fixed; top: 14px; left: 50%; transform: translateX(-50%); z-index: 11;
    /* Wrap rather than scroll: in a narrow embed a scrollable bar just looks
       like a clipped one (a tab sliced at the edge, no affordance). Wrapping
       shows every scenario, and layoutTopOverlays() drops the HUD/hint below
       whatever height the bar ends up being. */
    display: flex; flex-wrap: wrap; align-items: center; justify-content: center;
    gap: 4px 10px; background: rgba(13,15,18,0.92);
    border: 1px solid rgba(0,229,255,0.22); box-shadow: 0 6px 24px rgba(0,0,0,0.4);
    border-radius: 10px; padding: 6px; backdrop-filter: blur(6px);
    max-width: calc(100% - 24px); max-height: 42vh; overflow-y: auto;
  }
  /* A group must be allowed to wrap internally, not just the bar between
     groups: otherwise a single group (the 6 synthetic labs) is an unbreakable
     nowrap flex item wider than a narrow embed and overflows the bar. */
  #tabs .group { display: flex; flex-wrap: wrap; justify-content: center; align-items: center; gap: 3px; }
  #tabs .group-label {
    font-size: 9px; letter-spacing: 0.08em; color: #55606e; text-transform: uppercase;
    padding: 0 8px 0 6px; font-weight: 600; white-space: nowrap;
  }
  #tabs .divider { width: 1px; align-self: stretch; background: rgba(255,255,255,0.1); margin: 4px 2px; }
  #tabs button {
    background: transparent; border: 1px solid transparent; color: #8fa0b3; font-size: 12px;
    padding: 7px 12px; border-radius: 7px; cursor: pointer; white-space: nowrap;
    transition: background 0.15s ease, color 0.15s ease, border-color 0.15s ease;
  }
  #tabs button.active {
    background: rgba(0,229,255,0.14); color: #00e5ff; border-color: rgba(0,229,255,0.35);
    box-shadow: inset 0 0 0 1px rgba(0,229,255,0.08);
  }
  #tabs button:hover:not(.active) { color: #e4edf5; background: rgba(255,255,255,0.05); }
  #hud {
    position: fixed; top: 60px; left: 12px; z-index: 10;
    background: rgba(15,15,15,0.85); border: 1px solid rgba(0,229,255,0.35);
    border-radius: 8px; padding: 12px 16px; color: #dfefff; font-size: 13px;
    max-width: min(300px, calc(100% - 24px));
    max-height: calc(100% - 132px); overflow-y: auto;
    backdrop-filter: blur(4px);
  }
  #hud h1 { font-size: 13px; margin: 0 0 8px; color: #00e5ff; font-weight: 600; letter-spacing: 0.02em; }
  #hud .row { display:flex; justify-content:space-between; gap:12px; margin: 2px 0; }
  #hud .label { color: #8fa; opacity:0.6; }
  #legend { margin-top: 10px; border-top: 1px solid rgba(255,255,255,0.1); padding-top: 8px; }
  #legend .item { display:flex; align-items:center; gap:6px; margin: 3px 0; font-size:12px; }
  #legend .sw { width:10px; height:10px; border-radius:2px; flex-shrink:0; }
  #replan-log { margin-top: 10px; border-top: 1px solid rgba(255,255,255,0.1); padding-top: 8px; max-height: 140px; overflow-y: auto; }
  #replan-log .entry { font-size: 11px; color: #ffd166; margin: 2px 0; }
  #controls {
    position: fixed; bottom: 12px; left: 12px; z-index: 10;
    background: rgba(15,15,15,0.85); border: 1px solid rgba(0,229,255,0.35);
    border-radius: 8px; padding: 10px 14px; color: #dfefff; font-size: 12px;
    display:flex; align-items:center; gap:10px;
  }
  #controls button {
    background: rgba(0,229,255,0.12); border: 1px solid rgba(0,229,255,0.4); color:#00e5ff;
    border-radius:6px; padding:5px 10px; cursor:pointer; font-size:12px;
  }
  #controls button:hover { background: rgba(0,229,255,0.25); }
  #controls input[type=range] { width: 160px; }
  #step-label { min-width: 130px; }
  #play-btn.running {
    background: rgba(0,229,255,0.35); color: #eafcff;
    animation: play-pulse 1.1s ease-in-out infinite;
  }
  @keyframes play-pulse {
    0%, 100% { box-shadow: 0 0 0 0 rgba(0,229,255,0.55); }
    50% { box-shadow: 0 0 0 7px rgba(0,229,255,0); }
  }
  #mode-switch {
    position: fixed; bottom: 12px; right: 12px; z-index: 10;
    background: rgba(15,15,15,0.85); border: 1px solid rgba(0,229,255,0.35);
    border-radius: 8px; padding: 4px; display:flex; gap:3px;
  }
  #mode-switch button {
    background: transparent; border: 1px solid transparent; color:#8fa0b3; font-size:11px;
    border-radius:6px; padding:6px 10px; cursor:pointer;
  }
  #mode-switch button.active { background: rgba(0,229,255,0.16); color:#00e5ff; border-color: rgba(0,229,255,0.35); }
  #mode-switch button:hover:not(.active) { color:#e4edf5; background: rgba(255,255,255,0.05); }
  #crosshair {
    position: fixed; top: 50%; left: 50%; width: 6px; height: 6px; z-index: 9;
    transform: translate(-50%,-50%); border-radius: 50%; background: rgba(0,229,255,0.85);
    box-shadow: 0 0 4px rgba(0,229,255,0.8); display: none; pointer-events: none;
  }
  #pointer-lock-hint {
    position: fixed; top: 50%; left: 50%; transform: translate(-50%,-50%); z-index: 12;
    background: rgba(10,12,15,0.92); border: 1px solid rgba(0,229,255,0.4); border-radius: 10px;
    padding: 14px 20px; color: #dfefff; font-size: 13px; text-align: center; display: none;
    cursor: pointer;
  }
  /* Control tutorial. Never hidden: it sits clear below the (now scrollable)
     tab bar and gets its own backing so it stays readable over any terrain. */
  #hint {
    position: fixed; top: 64px; right: 12px; z-index: 10;
    max-width: min(280px, calc(100% - 24px));
    background: rgba(13,15,18,0.72); border: 1px solid rgba(255,255,255,0.08);
    border-radius: 8px; padding: 5px 9px; color: #9fb0c0;
    font-size: 11px; line-height: 1.4; text-align: right; pointer-events: none;
  }
  @media (max-width: 640px) {
    /* Keep the bottom-left controls and bottom-right mode switch from
       colliding in a narrow embed. (The HUD/hint stacking is handled in
       layoutTopOverlays(), which can measure the wrapped tab bar.) */
    #controls { gap: 7px; padding: 8px 10px; }
    #controls input[type=range] { width: 90px; }
    #step-label { min-width: 0; }
    #mode-switch button { padding: 6px 8px; }
  }
  @media (max-width: 460px) {
    /* Even the slimmed pair is too wide side by side: lift the mode switch
       above the controls instead. */
    #mode-switch { bottom: 60px; }
  }
</style>
</head>
<body>
<div id="canvas-wrap"></div>
<div id="tabs"></div>
<div id="hud">
  <h1 id="scenario-title">SCENARIO</h1>
  <div class="row"><span class="label">algorithm</span><span>weighted A* (eps=1.5)</span></div>
  <div class="row"><span class="label">path cost</span><span id="cost-val">-</span></div>
  <div class="row"><span class="label">nodes expanded</span><span id="exp-val">-</span></div>
  <div class="row"><span class="label">path length</span><span id="len-val">-</span></div>
  <div id="legend">
    <div class="item"><div class="sw" style="background:#7ee787"></div>SPRINT</div>
    <div class="item"><div class="sw" style="background:#ff6b6b"></div>MINE</div>
    <div class="item"><div class="sw" style="background:#00e5ff"></div>BRIDGE</div>
    <div class="item"><div class="sw" style="background:#ff9f1a"></div>BRIDGE_UP</div>
    <div class="item"><div class="sw" style="background:#ffd166"></div>CLIMB / FALL</div>
    <div class="item"><div class="sw" style="background:#c678dd"></div>BOAT_CRAWL</div>
    <div class="item"><div class="sw" style="background:#ff8fd6"></div>PARKOUR</div>
  </div>
  <div id="replan-log" style="display:none"></div>
</div>
<div id="hint">drag to rotate &middot; scroll to zoom &middot; right-drag to pan &middot; enter to play/pause</div>
<div id="crosshair"></div>
<div id="pointer-lock-hint">click to look around<br><span id="pointer-lock-subtext" style="opacity:0.6;font-size:11px">WASD move &middot; space/shift up-down &middot; esc to release</span></div>
<div id="controls">
  <button id="play-btn">play</button>
  <input type="range" id="step-slider" min="0" max="1" value="0" step="1">
  <span id="step-label">step 0 / 0</span>
</div>
<div id="mode-switch">
  <button id="mode-orbit" class="active" data-mode="orbit">Orbit</button>
  <button id="mode-freefly" data-mode="freefly">Free-Fly</button>
  <button id="mode-pov" data-mode="pov">POV</button>
</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>
const SCENARIOS = """

    template_tail = """;

const SCENARIO_LABELS = """ + json.dumps(scenario_labels) + """;
const SCENARIO_ORDER = """ + json.dumps(scenario_order) + """;
const SCENARIO_GROUPS = """ + json.dumps(scenario_groups) + """;
const GROUP_LABELS = { synthetic: "Synthetic", real: "Real Bastion Terrain" };

const BLOCK_COLORS = { 1: 0x6b4a2f, 2: 0x8a8a8a, 3: 0x1a0a2e, 4: 0x2b2b2b, 5: 0xff4500 };
const ACTION_COLORS = {
  SPRINT: 0x7ee787, MINE: 0xff6b6b, MINE_DOWN: 0xff6b6b,
  BRIDGE: 0x00e5ff, BRIDGE_UP: 0xff9f1a, CLIMB: 0xffd166, FALL: 0xffd166, START: 0xffffff,
  BOAT_CRAWL: 0xc678dd, PARKOUR: 0xff8fd6
};
// Blocks the bot places (BRIDGE / BRIDGE_UP) get their own color and a
// cyan emissive flash when they pop in, so a placed scaffold block never
// reads as pre-existing terrain -- the whole point of the animation.
const PLACED_BLOCK_COLOR = 0x9ad0ff;
const PLACED_BLOCK_EMISSIVE = 0x00e5ff;
// How fast a block's scale/opacity eases toward its current target, per
// second (1 - exp(-rate*dt) per frame). High enough that a break reads as
// a snap rather than a slow dissolve, low enough to be visible at 180ms
// playback steps.
const BLOCK_ANIM_RATE = 16;
const blockKey = (x, y, z) => x + ',' + y + ',' + z;

let scene, camera, renderer, spherical, target;
let currentWorldGroup = null;
let pathGroup = null;
let segMeshes = [], lineSegs = [];
let currentScenarioKey = SCENARIO_ORDER[0];
let currentStep = 0;
let playing = false, playTimer = null;
let markerMeshes = [];
let replanMarkers = [];
// Mined/placed blocks that animate as the route is scrubbed. Separate from
// the instanced terrain shell because each one needs its own scale/opacity
// tween; there are few enough per route for individual meshes to be cheap.
let blockMeshes = [];

// --- camera modes: 'orbit' (default, mouse drag/wheel/pan) | 'freefly'
// (Minecraft-creative-style WASD + pointer-lock mouselook) | 'pov' (rides
// the solved path, interpolated between nodes rather than snapping) ---
let cameraMode = 'orbit';
let clock = new THREE.Clock();
const HINT_TEXT = {
  orbit: 'drag to rotate &middot; scroll to zoom &middot; right-drag to pan &middot; enter to play/pause',
  freefly: 'click to capture mouse &middot; WASD move &middot; space/shift up-down &middot; esc to release &middot; enter to play/pause',
  pov: 'riding the solved path &middot; enter to play/pause &middot; click to look around &middot; drag the slider to scrub',
};
const FREEFLY_SPEED = 22; // blocks/sec
// Not-yet-reached path steps render at this opacity instead of vanishing --
// dimmed but still their real action color, so "upcoming" reads distinctly
// from "doesn't exist" without losing the color-coded action legend.
const FUTURE_OPACITY = 0.22;
const POV_EYE_HEIGHT = 1.6;
const POV_STEP_MS = 180; // must match the playTimer interval below

let freeflyYaw = 0, freeflyPitch = 0;
let freeflyKeys = Object.create(null);
let pointerLocked = false;
let lastStepAt = performance.now();

function initThree() {
  scene = new THREE.Scene();

  camera = new THREE.PerspectiveCamera(55, window.innerWidth/window.innerHeight, 0.1, 600);

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setClearColor(0x0a0a0a);
  document.getElementById('canvas-wrap').appendChild(renderer.domElement);

  scene.add(new THREE.AmbientLight(0x8899aa, 0.7));
  const dl = new THREE.DirectionalLight(0xffffff, 0.6);
  dl.position.set(30, 50, 20);
  scene.add(dl);
  const dl2 = new THREE.DirectionalLight(0x3399ff, 0.25);
  dl2.position.set(-20, 10, -30);
  scene.add(dl2);

  const dom = renderer.domElement;
  let isDragging = false, isPanning = false, lastX = 0, lastY = 0;
  dom.addEventListener('mousedown', e => {
    if (cameraMode !== 'orbit') return;
    if (e.button === 2) { isPanning = true; } else { isDragging = true; }
    lastX = e.clientX; lastY = e.clientY;
  });
  window.addEventListener('mouseup', () => { isDragging = false; isPanning = false; });
  window.addEventListener('mousemove', e => {
    if (cameraMode !== 'orbit') return;
    const dx = e.clientX - lastX, dy = e.clientY - lastY;
    if (isDragging) {
      spherical.theta -= dx * 0.006;
      spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi - dy * 0.006));
      updateCamera();
    } else if (isPanning) {
      const panSpeed = 0.05;
      const offset = new THREE.Vector3().setFromSpherical(spherical);
      const forward = offset.clone().normalize();
      const right = new THREE.Vector3().crossVectors(forward, camera.up).normalize();
      const up = new THREE.Vector3().crossVectors(right, forward).normalize();
      target.addScaledVector(right, -dx * panSpeed);
      target.addScaledVector(up, dy * panSpeed);
      updateCamera();
    }
    lastX = e.clientX; lastY = e.clientY;
  });
  dom.addEventListener('contextmenu', e => e.preventDefault());
  dom.addEventListener('wheel', e => {
    if (cameraMode !== 'orbit') return;
    e.preventDefault();
    spherical.radius = Math.max(4, Math.min(260, spherical.radius + e.deltaY * 0.05));
    updateCamera();
  }, { passive: false });

  // --- free-fly: click to pointer-lock (Minecraft-style mouselook), WASD
  // to move relative to look direction, space/shift for up/down, no
  // gravity or collision -- creative-flight, not survival movement.
  // POV shares this same pointer-lock mouselook (yaw/pitch only -- WASD
  // does nothing there) so you can look around freely while the camera
  // rides the solved path, rather than being locked facing forward. ---
  const usesMouselook = () => cameraMode === 'freefly' || cameraMode === 'pov';
  dom.addEventListener('click', () => {
    if (usesMouselook() && !pointerLocked) dom.requestPointerLock();
  });
  document.addEventListener('pointerlockchange', () => {
    pointerLocked = document.pointerLockElement === dom;
    document.getElementById('pointer-lock-hint').style.display =
      (usesMouselook() && !pointerLocked) ? 'block' : 'none';
    document.getElementById('pointer-lock-subtext').textContent = cameraMode === 'pov'
      ? 'press play to ride the route · esc to release'
      : 'WASD move · space/shift up-down · esc to release';
  });
  window.addEventListener('mousemove', e => {
    if (!usesMouselook() || !pointerLocked) return;
    freeflyYaw -= e.movementX * 0.0022;
    freeflyPitch = Math.max(-1.5, Math.min(1.5, freeflyPitch - e.movementY * 0.0022));
  });
  const FLY_KEYS = new Set(['KeyW','KeyA','KeyS','KeyD','Space','ShiftLeft','ShiftRight']);
  window.addEventListener('keydown', e => {
    freeflyKeys[e.code] = true;
    if (cameraMode === 'freefly' && FLY_KEYS.has(e.code)) e.preventDefault(); // stop space/arrows scrolling the page
  });
  window.addEventListener('keyup', e => { freeflyKeys[e.code] = false; });

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
    layoutTopOverlays();
  });
}

function updateCamera() {
  const pos = new THREE.Vector3().setFromSpherical(spherical).add(target);
  camera.position.copy(pos);
  camera.lookAt(target);
}

function updateFreefly(dt) {
  const dir = new THREE.Vector3(
    Math.cos(freeflyPitch) * Math.sin(freeflyYaw),
    Math.sin(freeflyPitch),
    Math.cos(freeflyPitch) * Math.cos(freeflyYaw),
  );
  camera.lookAt(camera.position.clone().add(dir));

  const forward = dir.clone(); forward.y = 0; forward.normalize();
  const right = new THREE.Vector3().crossVectors(forward, camera.up).normalize();
  const speed = FREEFLY_SPEED * dt;
  const move = new THREE.Vector3();
  if (freeflyKeys['KeyW']) move.add(forward);
  if (freeflyKeys['KeyS']) move.sub(forward);
  if (freeflyKeys['KeyD']) move.add(right);
  if (freeflyKeys['KeyA']) move.sub(right);
  if (move.lengthSq() > 0) {
    move.normalize().multiplyScalar(speed);
    camera.position.add(move);
  }
  const vertSpeed = FREEFLY_SPEED * dt * 0.7;
  if (freeflyKeys['Space']) camera.position.y += vertSpeed;
  if (freeflyKeys['ShiftLeft'] || freeflyKeys['ShiftRight']) camera.position.y -= vertSpeed;
}

function updatePov() {
  const path = SCENARIOS[currentScenarioKey].path;
  if (path.length < 2) return;
  const i = Math.min(currentStep, path.length - 2);
  // Interpolate smoothly toward the next node only while actively
  // playing -- while paused/scrubbing the slider, sit exactly at the
  // current node instead of drifting, so manual scrubbing feels precise.
  const frac = playing ? Math.min(1, (performance.now() - lastStepAt) / POV_STEP_MS) : 0;
  const a = path[i], b = path[Math.min(i + 1, path.length - 1)];
  const x = a[0] + (b[0] - a[0]) * frac;
  const y = a[1] + (b[1] - a[1]) * frac;
  const z = a[2] + (b[2] - a[2]) * frac;
  camera.position.set(x, y + POV_EYE_HEIGHT, z);
  // Position rides the path; orientation is free-look (mouse-driven
  // freeflyYaw/freeflyPitch, same as free-fly) rather than locked facing
  // the direction of travel -- you can look around while riding the rails.
  const dir = new THREE.Vector3(
    Math.cos(freeflyPitch) * Math.sin(freeflyYaw),
    Math.sin(freeflyPitch),
    Math.cos(freeflyPitch) * Math.cos(freeflyYaw),
  );
  camera.lookAt(camera.position.clone().add(dir));
}

/**
 * Derive which blocks each step removes or places from the path + action
 * list alone -- the exported scenario stores only the ORIGINAL terrain, so
 * the world changes the bot causes have to be reconstructed here. Mirrors
 * EdgeRules.java's semantics exactly:
 *   MINE / BOAT_CRAWL -> the destination column's feet (and head, if the
 *     2-tall dig needed it) blocks are broken and become air.
 *   MINE_DOWN        -> the block straight below is broken.
 *   BRIDGE           -> a support block is placed under the destination
 *     (flat gap crossing, or rising above lava).
 *   BRIDGE_UP        -> a block is placed underfoot at the source and the
 *     bot pillars up into the cell above.
 * Values are the step count (n) at which the change has happened, i.e.
 * action index + 1, so showStep(n) can test n < removedAt / n >= placedAt.
 */
function computeBlockEvents(path, actions) {
  const removals = new Map();
  const placements = new Map();
  for (let i = 0; i < actions.length; i++) {
    const action = actions[i];
    const f = path[i], t = path[i + 1];
    const step = i + 1;
    if (action === 'MINE' || action === 'BOAT_CRAWL') {
      removals.set(blockKey(t[0], t[1], t[2]), step);
      removals.set(blockKey(t[0], t[1] + 1, t[2]), step);
    } else if (action === 'MINE_DOWN') {
      removals.set(blockKey(t[0], t[1], t[2]), step);
    } else if (action === 'BRIDGE') {
      placements.set(blockKey(t[0], t[1] - 1, t[2]), step);
    } else if (action === 'BRIDGE_UP') {
      placements.set(blockKey(f[0], f[1], f[2]), step);
    }
  }
  return { removals, placements };
}

function clearScene() {
  if (currentWorldGroup) { scene.remove(currentWorldGroup); currentWorldGroup = null; }
  if (pathGroup) { scene.remove(pathGroup); pathGroup = null; }
  markerMeshes.forEach(m => scene.remove(m));
  markerMeshes = [];
  replanMarkers.forEach(m => scene.remove(m));
  replanMarkers = [];
  blockMeshes = [];
  segMeshes = []; lineSegs = [];
}

function loadScenario(key) {
  clearScene();
  currentScenarioKey = key;
  const DATA = SCENARIOS[key];

  const DX = DATA.dims[0], DY = DATA.dims[1], DZ = DATA.dims[2];
  const cx = DX/2, cy = DY/2, cz = DZ/2;

  target = new THREE.Vector3(cx, cy, cz);
  // Frame on the bounding SPHERE (diagonal), not just the widest horizontal
  // axis -- a wide-but-short region (a shallow cave corridor spanning a
  // huge flat footprint, e.g. real terrain with DY << DX,DZ) previously
  // used the same oversized max(DX,DZ) radius as a tall, compact one,
  // leaving its path too small/thin to read at the default zoom without
  // the viewer already knowing to scroll in.
  const diag = Math.sqrt(DX*DX + DY*DY + DZ*DZ);
  spherical = new THREE.Spherical(Math.max(diag * 0.62, Math.max(DX, DZ) * 0.55), 1.0, 0.7);
  updateCamera();

  const path = DATA.path;
  const actions = DATA.actions;
  const { removals, placements } = computeBlockEvents(path, actions);
  const typeByKey = new Map();
  for (const b of DATA.blocks) typeByKey.set(blockKey(b[0], b[1], b[2]), b[3]);

  currentWorldGroup = new THREE.Group();
  const boxGeo = new THREE.BoxGeometry(0.98, 0.98, 0.98);
  const byType = {};
  for (const b of DATA.blocks) {
    const x=b[0], y=b[1], z=b[2], t=b[3];
    // Mined blocks are excluded from the static shell and get their own
    // animated mesh below, so they can crumble at the right step instead of
    // just vanishing (or never disappearing) with the rest of the terrain.
    if (removals.has(blockKey(x, y, z))) continue;
    (byType[t] = byType[t] || []).push([x,y,z]);
  }
  for (const t in byType) {
    const list = byType[t];
    const mat = new THREE.MeshLambertMaterial({ color: BLOCK_COLORS[t] || 0x888888 });
    const mesh = new THREE.InstancedMesh(boxGeo, mat, list.length);
    const dummy = new THREE.Object3D();
    list.forEach((p, i) => {
      dummy.position.set(p[0], p[1], p[2]);
      dummy.updateMatrix();
      mesh.setMatrixAt(i, dummy.matrix);
    });
    mesh.instanceMatrix.needsUpdate = true;
    currentWorldGroup.add(mesh);
  }
  scene.add(currentWorldGroup);

  // Dynamic blocks: mined ones start visible and shrink/spin/fade away at
  // their step; placed ones start collapsed and grow in with an emissive
  // flash. updateBlockAnimations eases each mesh toward userData.visible,
  // so scrubbing backward runs the same tween in reverse and restores the
  // terrain, as the write-up promises.
  for (const [key, removedAt] of removals) {
    const t = typeByKey.get(key);
    if (t === undefined) continue; // interior block, never part of the shell
    const [x, y, z] = key.split(',').map(Number);
    const mat = new THREE.MeshLambertMaterial({ color: BLOCK_COLORS[t] || 0x888888, transparent: true, opacity: 1 });
    const mesh = new THREE.Mesh(boxGeo, mat);
    mesh.position.set(x, y, z);
    mesh.userData = { kind: 'remove', threshold: removedAt, visible: true };
    currentWorldGroup.add(mesh);
    blockMeshes.push(mesh);
  }
  for (const [key, placedAt] of placements) {
    if (removals.has(key)) continue; // placed then mined -- removal wins
    const [x, y, z] = key.split(',').map(Number);
    const mat = new THREE.MeshLambertMaterial({ color: PLACED_BLOCK_COLOR, emissive: PLACED_BLOCK_EMISSIVE, transparent: true, opacity: 1 });
    const mesh = new THREE.Mesh(boxGeo, mat);
    mesh.position.set(x, y, z);
    mesh.scale.setScalar(0.001);
    mesh.visible = false;
    mesh.userData = { kind: 'place', threshold: placedAt, visible: false };
    currentWorldGroup.add(mesh);
    blockMeshes.push(mesh);
  }

  pathGroup = new THREE.Group();
  scene.add(pathGroup);

  const sphereGeo = new THREE.SphereGeometry(0.18, 12, 12);
  for (let i = 0; i < path.length; i++) {
    const x=path[i][0], y=path[i][1], z=path[i][2];
    const actionColor = i === 0 ? ACTION_COLORS.START : (ACTION_COLORS[actions[i-1]] || 0xffffff);
    // transparent:true from creation so showStep can just tune .opacity
    // per-frame afterward (cheap) instead of toggling .visible, which used
    // to make not-yet-reached path steps disappear outright -- faded but
    // still colored reads as "upcoming", not "doesn't exist yet".
    const mat = new THREE.MeshBasicMaterial({ color: actionColor, transparent: true, opacity: FUTURE_OPACITY });
    const sphere = new THREE.Mesh(sphereGeo, mat);
    sphere.position.set(x, y + 0.55, z);
    pathGroup.add(sphere);
    segMeshes.push(sphere);
  }

  for (let i = 0; i < path.length - 1; i++) {
    const x0=path[i][0], y0=path[i][1], z0=path[i][2];
    const x1=path[i+1][0], y1=path[i+1][1], z1=path[i+1][2];
    const color = ACTION_COLORS[actions[i]] || 0xffffff;
    const from = new THREE.Vector3(x0, y0+0.55, z0);
    const to = new THREE.Vector3(x1, y1+0.55, z1);
    const delta = new THREE.Vector3().subVectors(to, from);
    const length = delta.length() || 0.0001;
    // ArrowHelper instead of a plain line so a long, near-vertical FALL
    // (a single edge can drop many blocks in this model, see README) reads
    // unambiguously as "moving toward the arrowhead", not as an undirected
    // segment that's easy to misread as going the other way. Cap the
    // head size instead of using ArrowHelper's default proportional
    // (length * 0.2) sizing, so a 9-block fall doesn't grow a
    // disproportionately huge cone compared to a 1-block sprint step.
    const headLength = Math.min(0.35, length * 0.5);
    const headWidth = Math.min(0.2, length * 0.3);
    const arrow = new THREE.ArrowHelper(delta.clone().normalize(), from, length, color, headLength, headWidth);
    arrow.line.material.linewidth = 2;
    arrow.line.material.transparent = true;
    arrow.line.material.opacity = FUTURE_OPACITY;
    arrow.cone.material.transparent = true;
    arrow.cone.material.opacity = FUTURE_OPACITY;
    pathGroup.add(arrow);
    lineSegs.push(arrow);
  }

  function makeMarker(pos, color) {
    const geo = new THREE.OctahedronGeometry(0.45);
    const mat = new THREE.MeshBasicMaterial({ color });
    const m = new THREE.Mesh(geo, mat);
    m.position.set(pos[0], pos[1] + 0.6, pos[2]);
    scene.add(m);
    markerMeshes.push(m);
    return m;
  }
  makeMarker(DATA.start, 0xffffff);
  makeMarker(DATA.goal, 0xff33aa);

  const replanLog = document.getElementById('replan-log');
  if (DATA.replans && DATA.replans.length > 0) {
    replanLog.style.display = 'block';
    replanLog.innerHTML = '<div style="font-size:11px;color:#8fa;opacity:0.6;margin-bottom:4px">REPLAN EVENTS (' + DATA.replans.length + ')</div>';
    DATA.replans.forEach((r, idx) => {
      const div = document.createElement('div');
      div.className = 'entry';
      div.textContent = (idx+1) + '. ' + r.reason + ' @ (' + r.at.join(',') + ')';
      replanLog.appendChild(div);
      const geo = new THREE.RingGeometry(0.5, 0.65, 16);
      const mat = new THREE.MeshBasicMaterial({ color: 0xffd166, side: THREE.DoubleSide, transparent: true, opacity: 0.7 });
      const ring = new THREE.Mesh(geo, mat);
      ring.position.set(r.at[0], r.at[1] + 0.5, r.at[2]);
      ring.rotation.x = Math.PI / 2;
      scene.add(ring);
      replanMarkers.push(ring);
    });
  } else {
    replanLog.style.display = 'none';
    replanLog.innerHTML = '';
  }

  document.getElementById('scenario-title').textContent = SCENARIO_LABELS[key].toUpperCase();
  document.getElementById('cost-val').textContent = (typeof DATA.cost === 'number' ? DATA.cost.toFixed(2) + 's (sim)' : DATA.cost);
  document.getElementById('exp-val').textContent = DATA.expansions.toString();
  document.getElementById('len-val').textContent = path.length + ' nodes';

  const slider = document.getElementById('step-slider');
  slider.max = path.length - 1;
  showStep(path.length - 1, true); // snap: the first frame shows the finished route, not a reveal animation

  document.querySelectorAll('#tabs button').forEach(b => {
    b.classList.toggle('active', b.dataset.key === key);
  });
}

function showStep(n, snap) {
  currentStep = n;
  lastStepAt = performance.now();
  for (let i = 0; i < segMeshes.length; i++) segMeshes[i].material.opacity = i <= n ? 1 : FUTURE_OPACITY;
  for (let i = 0; i < lineSegs.length; i++) {
    const opacity = i < n ? 1 : FUTURE_OPACITY;
    lineSegs[i].line.material.opacity = opacity;
    lineSegs[i].cone.material.opacity = opacity;
  }
  for (const mesh of blockMeshes) {
    const ud = mesh.userData;
    ud.visible = ud.kind === 'remove' ? n < ud.threshold : n >= ud.threshold;
    if (snap) {
      mesh.scale.setScalar(ud.visible ? 1 : 0.001);
      mesh.rotation.set(0, 0, 0);
      mesh.material.opacity = 1;
      if (ud.kind === 'place') {
        mesh.material.emissive.setHex(PLACED_BLOCK_EMISSIVE);
        mesh.material.emissiveIntensity = ud.visible ? 0 : 0.9;
      }
      mesh.visible = ud.visible;
    }
  }
  const path = SCENARIOS[currentScenarioKey].path;
  document.getElementById('step-label').textContent = "step " + n + " / " + (path.length - 1);
  document.getElementById('step-slider').value = n;
}

function updateBlockAnimations(dt) {
  if (blockMeshes.length === 0) return;
  const k = 1 - Math.exp(-BLOCK_ANIM_RATE * dt);
  for (const mesh of blockMeshes) {
    const ud = mesh.userData;
    const target = ud.visible ? 1 : 0.001;
    let next = mesh.scale.x + (target - mesh.scale.x) * k;
    if (Math.abs(next - target) < 0.002) next = target;
    mesh.scale.setScalar(next);
    if (ud.kind === 'remove') {
      // Crumble: spin and fade out together with the shrink, so a broken
      // block reads as "destroyed", not just "scaled down".
      mesh.material.opacity = Math.max(0, Math.min(1, next));
      if (ud.visible) mesh.rotation.set(0, 0, 0);
      else { mesh.rotation.y += dt * 15; mesh.rotation.x += dt * 8; }
      mesh.visible = next > 0.02;
    } else {
      // Placed: grow in, with the emissive flash fading as it settles.
      mesh.material.emissive.setHex(PLACED_BLOCK_EMISSIVE);
      mesh.material.emissiveIntensity = Math.max(0, 1 - next) * 0.9;
      mesh.visible = next > 0.02;
    }
  }
}

function setCameraMode(mode) {
  cameraMode = mode;
  document.querySelectorAll('#mode-switch button').forEach(b => {
    b.classList.toggle('active', b.dataset.mode === mode);
  });
  document.getElementById('hint').innerHTML = HINT_TEXT[mode];
  document.getElementById('crosshair').style.display = (mode === 'freefly' || mode === 'pov') ? 'block' : 'none';

  if (mode !== 'freefly' && mode !== 'pov' && pointerLocked) document.exitPointerLock();
  document.getElementById('pointer-lock-hint').style.display = 'none';

  if (mode === 'freefly') {
    // Seed yaw/pitch from wherever the camera currently is (e.g. the
    // orbit view just left off) so switching modes doesn't snap the view.
    const dir = new THREE.Vector3();
    camera.getWorldDirection(dir);
    freeflyYaw = Math.atan2(dir.x, dir.z);
    freeflyPitch = Math.asin(Math.max(-1, Math.min(1, dir.y)));
  } else if (mode === 'pov') {
    // Seed the free-look direction toward where the path is actually
    // heading (current node -> next node), not wherever orbit/free-fly
    // happened to be facing -- riding backward by default would be a bad
    // first impression of a mode whose whole point is watching the route.
    const path = SCENARIOS[currentScenarioKey].path;
    const i = Math.min(currentStep, path.length - 2);
    const a = path[i], b = path[Math.min(i + 1, path.length - 1)];
    const dir = new THREE.Vector3(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    if (dir.lengthSq() > 1e-6) {
      dir.normalize();
      freeflyYaw = Math.atan2(dir.x, dir.z);
      freeflyPitch = Math.asin(Math.max(-1, Math.min(1, dir.y)));
    }
  } else if (mode === 'orbit') {
    // Rebuild the orbit spherical from the camera's current position so
    // returning from free-fly/POV doesn't jump back to the old orbit spot.
    const offset = camera.position.clone().sub(target);
    spherical.setFromVector3(offset);
    updateCamera();
  }
}

/**
 * The tab bar wraps, so its height depends on the embed width. Rather than
 * guessing an offset, measure the bar and drop the HUD and control guide
 * below it -- and below each other when the width is too small for them to
 * sit side by side. Called on load, on resize, and once fonts settle (tab
 * text width can change after the first paint).
 */
function layoutTopOverlays() {
  const tabs = document.getElementById('tabs');
  const hint = document.getElementById('hint');
  const hud = document.getElementById('hud');
  if (!tabs || !hint || !hud) return;
  const top = Math.round(tabs.getBoundingClientRect().bottom + 10);
  const narrow = window.innerWidth < 640;

  hint.style.top = top + 'px';
  hint.style.left = '';
  hint.style.right = '12px';

  hud.style.top = narrow
    ? Math.round(hint.getBoundingClientRect().bottom + 8) + 'px'
    : top + 'px';
  const hudTop = parseInt(hud.style.top, 10) || top;
  hud.style.maxHeight = Math.max(120, window.innerHeight - hudTop - 72) + 'px';
}

function setupUI() {
  const tabs = document.getElementById('tabs');
  let lastGroup = null;
  let groupEl = null;
  SCENARIO_ORDER.forEach(key => {
    const groupName = SCENARIO_GROUPS[key];
    if (groupName !== lastGroup) {
      if (lastGroup !== null) {
        const divider = document.createElement('div');
        divider.className = 'divider';
        tabs.appendChild(divider);
      }
      groupEl = document.createElement('div');
      groupEl.className = 'group';
      const label = document.createElement('span');
      label.className = 'group-label';
      label.textContent = GROUP_LABELS[groupName] || groupName;
      groupEl.appendChild(label);
      tabs.appendChild(groupEl);
      lastGroup = groupName;
    }
    const btn = document.createElement('button');
    btn.textContent = SCENARIO_LABELS[key];
    btn.dataset.key = key;
    btn.onclick = () => { stopPlayback(); loadScenario(key); };
    groupEl.appendChild(btn);
  });

  const slider = document.getElementById('step-slider');
  slider.addEventListener('input', () => showStep(parseInt(slider.value)));

  const playBtn = document.getElementById('play-btn');
  playBtn.title = 'Enter to play/pause';
  playBtn.addEventListener('click', togglePlayback);

  document.querySelectorAll('#mode-switch button').forEach(b => {
    b.addEventListener('click', () => setCameraMode(b.dataset.mode));
  });

  window.addEventListener('keydown', e => {
    if (e.code !== 'Enter') return;
    e.preventDefault(); // also stops a focused button/slider from double-firing its own click/change
    togglePlayback();
  });

  layoutTopOverlays();
}

function togglePlayback() {
  if (playing) { stopPlayback(); return; }
  playing = true;
  const playBtn = document.getElementById('play-btn');
  playBtn.textContent = 'pause';
  playBtn.classList.add('running');
  const path = SCENARIOS[currentScenarioKey].path;
  if (currentStep >= path.length - 1) showStep(0);
  playTimer = setInterval(() => {
    const path = SCENARIOS[currentScenarioKey].path;
    if (currentStep >= path.length - 1) { stopPlayback(); return; }
    showStep(currentStep + 1);
  }, 180);
}

function stopPlayback() {
  clearInterval(playTimer);
  playing = false;
  const playBtn = document.getElementById('play-btn');
  playBtn.textContent = 'play';
  playBtn.classList.remove('running');
}

function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(0.1, clock.getDelta()); // clamp: a tab-switch stall shouldn't teleport the camera
  if (cameraMode === 'freefly') updateFreefly(dt);
  else if (cameraMode === 'pov') updatePov();
  updateBlockAnimations(dt);
  renderer.render(scene, camera);
}

initThree();
setupUI();
loadScenario(SCENARIO_ORDER[0]);
animate();
// Tab widths can change once webfonts/metrics settle, and again on full load.
if (document.fonts && document.fonts.ready) document.fonts.ready.then(layoutTopOverlays);
window.addEventListener('load', layoutTopOverlays);
</script>
</body>
</html>"""

    html = template_head + viz_data + template_tail
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"\nwritten {output_path}, size KB: {len(html) / 1024:.1f}")


def load_scenarios_from_html(html_path):
    """Extract the already-solved SCENARIOS data back out of a previously
    built pathfind_viz_multi.html, so a template/JS/CSS-only edit can
    rebuild the page instantly with --from-cache instead of re-solving
    every scenario (world generation + Java search) from scratch just to
    change a CSS rule. Raises if the file doesn't look like one this
    script wrote (no point silently producing a broken page)."""
    text = Path(html_path).read_text(encoding="utf-8")
    marker = "const SCENARIOS = "
    start = text.find(marker)
    if start == -1:
        raise RuntimeError(f"{html_path} has no 'const SCENARIOS = ' -- not a file this script wrote, can't use --from-cache")
    start += len(marker)
    end = text.find("\n\nconst SCENARIO_LABELS", start)
    if end == -1:
        raise RuntimeError(f"{html_path}: couldn't find the end of the SCENARIOS blob -- template format may have changed")
    return json.loads(text[start:end].rstrip().rstrip(";"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mca-dir", default=str(_REPO_ROOT / "regions"),
        help="directory whose .mca region files become real-terrain scenarios (default: <repo root>/regions)",
    )
    parser.add_argument(
        "--mca-file", action="append", default=[],
        help="additional .mca region file; may be repeated",
    )
    parser.add_argument("--output", default=str(Path(__file__).resolve().parent / "pathfind_viz_multi.html"))
    parser.add_argument(
        "--from-cache", action="store_true",
        help="skip world generation/solving entirely and reuse --output's own "
             "already-solved SCENARIOS data -- for template/JS/CSS-only changes "
             "(instant, no Java/mca conversion). --mca-dir/--mca-file are ignored.",
    )
    args = parser.parse_args()

    if args.from_cache:
        scenarios = load_scenarios_from_html(args.output)
        print(f"loaded {len(scenarios)} cached scenarios from {args.output}, skipping solve")
    else:
        mca_files = []
        region_dir = Path(args.mca_dir)
        if region_dir.is_dir():
            mca_files.extend(sorted(region_dir.glob("**/*.mca")))
        elif args.mca_dir:
            print(f"warning: region directory not found, skipping: {region_dir}")
        mca_files.extend(Path(path) for path in args.mca_file)
        # A file passed explicitly may also be in --mca-dir; convert it once.
        mca_files = list(dict.fromkeys(str(path.resolve()) for path in mca_files))

        scenarios = build_all_scenarios(mca_files=mca_files)

    build_html(scenarios, output_path=args.output)
