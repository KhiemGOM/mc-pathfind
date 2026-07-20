# Minecraft speedrun pathfinder

Weighted A* over a full 3D Minecraft voxel world, modeling the real actions a
speedrunner can take (sprint, mine, bridge, boat-crawl, parkour/boat-jump,
fall/clutch) with realistic per-action time costs, then visualized in an
interactive 3D viewer. Runs against both synthetic test worlds and real
terrain decoded from Minecraft's own region files.

A Java port of just the search core (see [`java/`](java/)) exists for
raw search-speed comparison against the Python baseline, including
forward, bidirectional, and parallel-bidirectional A* variants.

## Files

- **`world.py`** - voxel world representation (AIR/DIRT/STONE/OBSIDIAN/
  BEDROCK/LAVA), synthetic test-world generators (mixed terrain, a giant
  unbreakable wall with a single gap, a pure bridging chasm, a cave-shortcut
  vs. detour tradeoff world).

- **`pathfind.py`** - the core solver. `weighted_astar(world, start, goal,
  blocks_available, epsilon)` runs bounded-suboptimal A* with these actions:
  - `SPRINT` - walking on solid ground
  - `MINE` / `MINE_DOWN` - breaking through solid blocks (Steve is 2 blocks
    tall, so normal mining always clears both foot and head level)
  - `BOAT_CRAWL` - the real boat-crawl technique: ducks through a 1-tall gap
    at reduced speed, with a one-time startup tax the first time you enter a
    crawl sequence (not re-charged on consecutive crawl moves)
  - `BRIDGE` - placing a block to cross a gap or lava; costs real time plus a
    risk penalty plus a scarcity-scaled tax on your remaining block supply
  - `PARKOUR` - long jump / boat-jump technique, up to 7 blocks, gated behind
    a cheap ledge-detection check and validated for a genuinely clear flight
    path (can't jump through walls) and a genuine gap being crossed (can't
    be used to fast-travel over normal ground)
  - `CLIMB` / `FALL` - vertical movement; falls of any height are survivable
    via clutch (water bucket) at a small time cost, except landing in or
    passing through lava, which is the one truly forbidden outcome
  - Lava is modeled as impassable but not solid: you can never stand in it
    (foot or head level), but you can bridge/climb over its surface.

- **`replan.py`** - simulates limited visibility (a reveal radius around the
  agent, like render distance) and replans when newly-discovered terrain
  invalidates the current route, rather than assuming the whole map is known
  upfront.

- **`reachability.py`** - a cheap walk-only BFS utility for sanity-checking
  whether a start/goal pair is reachable at all before running the full
  costed search (useful on real terrain, which can have genuinely
  disconnected pockets).

- **`mca_convert.py`** - reads a real Minecraft Anvil region file (`.mca`)
  and converts it into the solver's voxel array, with a block-type mapping
  table (netherrack/soul sand/etc -> DIRT, blackstone/ores -> STONE,
  obsidian/ancient debris -> OBSIDIAN, bedrock and lava handled specially).
  Uses `Chunk.stream_chunk()` for bulk decoding rather than per-voxel
  lookups, ~5x faster.

- **`build_visualization.py`** - regenerates all demo scenarios and rebuilds
  `pathfind_viz_multi.html` from scratch. Run this after any change to
  `pathfind.py`/`world.py` to keep the visualizer in sync with the solver.

- **`pathfind_viz_multi.html`** - the interactive 3D viewer (Three.js).
  Drag to rotate, scroll to zoom, right-drag to pan. Tabs switch between 12
  scenarios demonstrating each action and tradeoff; a step slider/play button
  replays the path move by move, color-coded by action type.

## Setup

```
pip install -r requirements.txt
```

Python 3.9+. The only non-stdlib dependencies are `numpy` and
`anvil-parser2` (for reading real `.mca` region files -- not needed if you
only care about the synthetic worlds in `world.py`).

## Usage

Regenerate everything (synthetic scenarios only):
```
python3 build_visualization.py
```

Regenerate including every real-terrain region in the bundled `regions/`
folder (the default), or point to another directory:
```
python3 build_visualization.py --mca-dir /path/to/regions
```

You can also add individual regions with `--mca-file /path/to/r.0.0.mca`.
The builder selects a walkable start/goal route independently for each region
and fails the rebuild if the full pathfinder cannot solve one within its search
limit, rather than silently publishing a route-less scenario.

For a real-terrain-only viewer with strict validation and a machine-readable
report, use the dedicated runner (defaults to `regions/k1`):
```
python3 run_region_viz.py
```
It writes `rviz_k1.html` and `rviz_k1_report.json`. The HTML is only written
when every selected route passes walk reachability, A*, and edge-by-edge route
validation; otherwise the report records the exact failing region and reason.

Use the solver directly:
```python
from world import generate_world, standing_y
from pathfind import weighted_astar

world, height = generate_world(size_x=40, size_y=20, size_z=40, seed=0)
start = (2, standing_y(height, 2, 5), 5)
goal = (35, standing_y(height, 35, 5), 5)

path, cost, actions, expansions = weighted_astar(
    world, start, goal, blocks_available=64, epsilon=1.5
)
```

`epsilon` controls the optimality/speed tradeoff (weighted A*): `1.0` is
true-optimal but slower; `1.5-2.0` is a good default for large worlds.

## Java port

[`java/`](java/) has a from-scratch Java port of just the search core (not
the `.mca` parsing, replanning, or visualization -- those stay Python-side),
for raw search-speed comparison against the Python baseline. It includes
forward, bidirectional, and parallel-bidirectional weighted A* variants, a
benchmark CLI that loads the same real-terrain regions used here, and an
experimental fork (`pearl`) exploring further solver optimizations. See
[`java/README.md`](java/README.md) for build/run instructions, and
[`experiments/SESSION_LOG.md`](experiments/SESSION_LOG.md) for the
optimization work behind it (dense state IDs, a decrease-key heap, and an
"air potential" heuristic that prunes low-value mining candidates).

## Project layout

```
world.py, pathfind.py, replan.py, reachability.py   Python solver
mca_convert.py                                      real Minecraft region -> voxel array
build_visualization.py, run_region_viz.py           demo/viewer generation
export_world_bin.py, export_all_regions.py          Python -> Java world-data bridge
*.html                                               pre-built interactive 3D demos
regions/                                             real Minecraft .mca region files (demo/benchmark input)
java/                                                Java port of the search core (see java/README.md)
experiments/                                         optimization dev log + one-off debug/sweep scripts
```

## Known simplifications

- Movement is 8-directional (4 cardinal + 4 diagonal) plus vertical, not
  full sub-block positioning.
- Cost constants (sprint speed, mining times, jump/bridge/parkour risk
  premiums) are hand-tuned approximations, not measured from real gameplay.
- `replan.py`'s replanning is a pragmatic re-run-A*-on-invalidation approach,
  not a full incremental D* Lite.

## License

[MIT](LICENSE)
