"""
Regenerates all demo scenarios and rebuilds pathfind_viz_multi.html from scratch.

Run this after any change to pathfind.py/world.py to keep the visualizer in
sync with the current solver -- forgetting this step was the single most
common source of "stale" bugs during development (editing the solver but
looking at an old cached scenario in the HTML).

Usage:
    python3 build_visualization.py [--mca-dir regions] [--mca-file PATH]

Every ``.mca`` file in --mca-dir becomes its own real-terrain scenario.  Extra
--mca-file arguments are also accepted for one-off regions.  Each region gets
an automatically selected walkable start/goal pair, so the visualizer is not
tied to coordinates that only happen to work in one saved world.
"""

import argparse
import json
from pathlib import Path
import numpy as np

from world import (
    generate_world, generate_wall_world, generate_bridge_world,
    generate_cave_shortcut_world, standing_y,
    AIR, DIRT, STONE, OBSIDIAN, BEDROCK,
)
from pathfind import weighted_astar


# Search difficulty varies wildly across real regions (a few dozen blocks of
# connected cave vs. a 500+-block open cavern spanning a fully-generated
# region), so one fixed epsilon is either too tight (blows the expansion
# budget on hard regions) or needlessly loose on easy ones. Try increasingly
# loose bounds and stop at the first one that finds anything -- for real-
# terrain testing we care whether the solver reaches the goal at all, not
# path optimality.
#
# Each rung gets its OWN (small-to-large) expansion budget rather than one
# fixed cap for every attempt: an early, too-tight epsilon on a hard region
# doesn't just fail, it fails only after burning its *entire* budget first
# (weighted A* has no way to know a rung is hopeless before exhausting it).
# Measured directly on this project's own worst-case region: eps=2.0 at a
# flat 300k-expansion cap ran the full budget and still failed, costing
# ~130s, before eps=3.5 converged in ~17k expansions / 5s. Giving early rungs
# a small budget makes their failures cheap, and only the last (loosest,
# fastest-converging) rung gets the generous ceiling.
REAL_TERRAIN_EPSILON_LADDER = (2.0, 3.5, 5.0, 8.0)
REAL_TERRAIN_EXPANSION_LADDER = (40_000, 100_000, 200_000, 300_000)
REAL_TERRAIN_MAX_EXPANSIONS = REAL_TERRAIN_EXPANSION_LADDER[-1]


def solve_with_epsilon_ladder(world, start, goal, blocks_available=32,
                               epsilon_ladder=REAL_TERRAIN_EPSILON_LADDER,
                               max_expansions=REAL_TERRAIN_EXPANSION_LADDER):
    """
    Try weighted_astar at each epsilon in epsilon_ladder (ascending), stopping
    at the first one that finds a path. `max_expansions` may be a single int
    (same budget for every rung) or a sequence matching epsilon_ladder
    (per-rung budget -- see module comment for why this matters: it keeps a
    hopeless low-epsilon attempt cheap instead of burning a large budget
    before ever trying the epsilon that actually works).

    Returns (path, cost, actions, expansions, epsilon_used, attempts) where
    attempts is a list of {"epsilon", "max_expansions", "expansions", "found"}
    dicts for every rung tried. If every rung fails, path is None and
    epsilon_used is None; expansions is from the last attempt.
    """
    if isinstance(max_expansions, (int, float)):
        budgets = [int(max_expansions)] * len(epsilon_ladder)
    else:
        budgets = list(max_expansions)
        if len(budgets) != len(epsilon_ladder):
            raise ValueError("max_expansions sequence must match epsilon_ladder length")

    attempts = []
    path = cost = actions = expansions = epsilon_used = None
    for epsilon, budget in zip(epsilon_ladder, budgets):
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

    Includes the parent directory name (e.g. "k1", "long1") since multiple
    saved worlds reuse the same r.X.Z.mca region filenames -- keying off the
    stem alone would collide and silently drop one world's scenario.
    """
    p = Path(mca_path)
    folder = p.parent.name
    stem = p.stem.replace(".", "_").replace("-", "neg")
    return f"real_{folder}_{stem}" if folder else f"real_{stem}"


def _region_label(mca_path):
    p = Path(mca_path)
    folder = p.parent.name
    return f"Real Nether: {folder}/{p.stem}" if folder else f"Real Nether: {p.stem}"


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

        path_r, cost_r, actions_r, exp_r, eps_used, _ = solve_with_epsilon_ladder(
            world_real, start_r, goal_r, blocks_available=32,
        )
        if path_r is None:
            print(f"  {key}: skipped -- A* found no path even at the loosest "
                  f"epsilon tried ({REAL_TERRAIN_EPSILON_LADDER[-1]}) from "
                  f"{start_r} to {goal_r}")
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
    p1, c1, a1, e1 = weighted_astar(w1, s1, g1, blocks_available=64, epsilon=1.5)
    scenarios["mixed_terrain"] = export_scenario(w1, p1, a1, s1, g1, c1, e1)
    print(f"mixed_terrain: cost={c1:.2f} nodes={len(p1)}")

    # 2. Giant unbreakable wall with a single gap -- "find the open terrain"
    w2, h2 = generate_wall_world(size_x=60, size_y=20, size_z=60, wall_thickness=5, gap_x=(27, 33), seed=1)
    s2 = (2, standing_y(h2, 2, 5), 5)
    g2 = (55, standing_y(h2, 55, 5), 5)
    p2, c2, a2, e2 = weighted_astar(w2, s2, g2, blocks_available=10, epsilon=1.5)
    scenarios["giant_wall"] = export_scenario(w2, p2, a2, s2, g2, c2, e2)
    print(f"giant_wall: cost={c2:.2f} nodes={len(p2)}")

    # 5. Cave shortcut where mining a short plug beats a long detour
    w4, h4 = generate_cave_shortcut_world(
        size_x=60, size_y=20, size_z=80, wall_thickness=6,
        detour_gap_z=(2, 8), cave_z=(38, 42), cave_mine_depth=3, seed=3,
    )
    s4 = (2, standing_y(h4, 2, 40), 40)
    g4 = (57, standing_y(h4, 57, 40), 40)
    p4, c4, a4, e4 = weighted_astar(w4, s4, g4, blocks_available=20, epsilon=1.3)
    scenarios["cave_shortcut_wins"] = export_scenario(w4, p4, a4, s4, g4, c4, e4)
    print(f"cave_shortcut_wins: cost={c4:.2f} nodes={len(p4)}")

    # 6. Same setup but the plug is thick enough that detouring wins instead
    w5, h5 = generate_cave_shortcut_world(
        size_x=60, size_y=20, size_z=80, wall_thickness=14,
        detour_gap_z=(35, 37), cave_z=(60, 64), cave_mine_depth=12, seed=3,
    )
    s5 = (2, standing_y(h5, 2, 61), 61)
    g5 = (57, standing_y(h5, 57, 61), 61)
    p5, c5, a5, e5 = weighted_astar(w5, s5, g5, blocks_available=20, epsilon=1.3)
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
    p6, c6, a6, e6 = weighted_astar(w6, s6, g6, blocks_available=20, epsilon=1.0)
    scenarios["tunnel_vs_bridge"] = export_scenario(w6, p6, a6, s6, g6, c6, e6)
    print(f"tunnel_vs_bridge: cost={c6:.2f} nodes={len(p6)}")

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
    display: flex; align-items: center; gap: 10px; background: rgba(13,15,18,0.92);
    border: 1px solid rgba(0,229,255,0.22); box-shadow: 0 6px 24px rgba(0,0,0,0.4);
    border-radius: 10px; padding: 6px; max-width: 92vw; backdrop-filter: blur(6px);
  }
  #tabs .group { display: flex; align-items: center; gap: 3px; }
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
    max-width: 300px; backdrop-filter: blur(4px);
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
  #hint { position: fixed; top: 12px; right: 12px; z-index:10; color:#556; font-size:11px; text-align:right; }
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
<div id="hint">drag to rotate &middot; scroll to zoom &middot; right-drag to pan</div>
<div id="controls">
  <button id="play-btn">play</button>
  <input type="range" id="step-slider" min="0" max="1" value="0" step="1">
  <span id="step-label">step 0 / 0</span>
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

let scene, camera, renderer, spherical, target;
let currentWorldGroup = null;
let pathGroup = null;
let segMeshes = [], lineSegs = [];
let currentScenarioKey = SCENARIO_ORDER[0];
let currentStep = 0;
let playing = false, playTimer = null;
let markerMeshes = [];
let replanMarkers = [];

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
    if (e.button === 2) { isPanning = true; } else { isDragging = true; }
    lastX = e.clientX; lastY = e.clientY;
  });
  window.addEventListener('mouseup', () => { isDragging = false; isPanning = false; });
  window.addEventListener('mousemove', e => {
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
    e.preventDefault();
    spherical.radius = Math.max(4, Math.min(260, spherical.radius + e.deltaY * 0.05));
    updateCamera();
  }, { passive: false });

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });
}

function updateCamera() {
  const pos = new THREE.Vector3().setFromSpherical(spherical).add(target);
  camera.position.copy(pos);
  camera.lookAt(target);
}

function clearScene() {
  if (currentWorldGroup) { scene.remove(currentWorldGroup); currentWorldGroup = null; }
  if (pathGroup) { scene.remove(pathGroup); pathGroup = null; }
  markerMeshes.forEach(m => scene.remove(m));
  markerMeshes = [];
  replanMarkers.forEach(m => scene.remove(m));
  replanMarkers = [];
  segMeshes = []; lineSegs = [];
}

function loadScenario(key) {
  clearScene();
  currentScenarioKey = key;
  const DATA = SCENARIOS[key];

  const DX = DATA.dims[0], DY = DATA.dims[1], DZ = DATA.dims[2];
  const cx = DX/2, cy = DY/2, cz = DZ/2;

  target = new THREE.Vector3(cx, cy, cz);
  spherical = new THREE.Spherical(Math.max(DX,DZ) * 1.15, 1.0, 0.7);
  updateCamera();

  currentWorldGroup = new THREE.Group();
  const boxGeo = new THREE.BoxGeometry(0.98, 0.98, 0.98);
  const byType = {};
  for (const b of DATA.blocks) {
    const x=b[0], y=b[1], z=b[2], t=b[3];
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

  const path = DATA.path;
  const actions = DATA.actions;

  pathGroup = new THREE.Group();
  scene.add(pathGroup);

  const sphereGeo = new THREE.SphereGeometry(0.18, 12, 12);
  for (let i = 0; i < path.length; i++) {
    const x=path[i][0], y=path[i][1], z=path[i][2];
    const actionColor = i === 0 ? ACTION_COLORS.START : (ACTION_COLORS[actions[i-1]] || 0xffffff);
    const mat = new THREE.MeshBasicMaterial({ color: actionColor });
    const sphere = new THREE.Mesh(sphereGeo, mat);
    sphere.position.set(x, y + 0.55, z);
    sphere.visible = false;
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
    arrow.visible = false;
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
  showStep(path.length - 1);

  document.querySelectorAll('#tabs button').forEach(b => {
    b.classList.toggle('active', b.dataset.key === key);
  });
}

function showStep(n) {
  currentStep = n;
  for (let i = 0; i < segMeshes.length; i++) segMeshes[i].visible = i <= n;
  for (let i = 0; i < lineSegs.length; i++) lineSegs[i].visible = i < n;
  const path = SCENARIOS[currentScenarioKey].path;
  document.getElementById('step-label').textContent = "step " + n + " / " + (path.length - 1);
  document.getElementById('step-slider').value = n;
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
  playBtn.addEventListener('click', () => {
    if (playing) { stopPlayback(); return; }
    playing = true; playBtn.textContent = 'pause';
    const path = SCENARIOS[currentScenarioKey].path;
    if (currentStep >= path.length - 1) showStep(0);
    playTimer = setInterval(() => {
      const path = SCENARIOS[currentScenarioKey].path;
      if (currentStep >= path.length - 1) { stopPlayback(); return; }
      showStep(currentStep + 1);
    }, 180);
  });
}

function stopPlayback() {
  clearInterval(playTimer);
  playing = false;
  document.getElementById('play-btn').textContent = 'play';
}

function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}

initThree();
setupUI();
loadScenario(SCENARIO_ORDER[0]);
animate();
</script>
</body>
</html>"""

    html = template_head + viz_data + template_tail
    with open(output_path, "w") as f:
        f.write(html)
    print(f"\nwritten {output_path}, size KB: {len(html) / 1024:.1f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mca-dir", default="regions",
        help="directory whose .mca region files become real-terrain scenarios (default: regions)",
    )
    parser.add_argument(
        "--mca-file", action="append", default=[],
        help="additional .mca region file; may be repeated",
    )
    parser.add_argument("--output", default="pathfind_viz_multi.html")
    args = parser.parse_args()

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
