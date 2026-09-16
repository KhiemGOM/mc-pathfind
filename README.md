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

- **`pathfind.py`** - the original solver and canonical reference for the
  action model below (most heavily commented; the Java port in `java/core/`
  is a faithful port of this same model, restructured for speed -- see
  `java/README.md`). `viz/build_visualization.py` solves through Java now
  (not this module), but `pathfind.py` itself is still live and directly
  required: `replan.py` (limited-visibility replanning has no Java port),
  `viz/run_region_viz.py` (solves AND independently re-validates through
  this same module on purpose -- see its own docstring), and as a
  standalone library -- see Usage below. `weighted_astar(world, start, goal,
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

- **`viz/`** - everything that generates or serves the interactive demos;
  see [`viz/build_visualization.py`](viz/build_visualization.py) and
  [`viz/run_region_viz.py`](viz/run_region_viz.py) for what each script
  does. `build_visualization.py` regenerates all demo scenarios and
  rebuilds `viz/pathfind_viz_multi.html` from scratch, solving every
  scenario through the **Java port** (`solve_via_java`, see
  [`java/README.md`](java/README.md) to build it first) rather than
  `pathfind.py` -- this script only owns world generation/conversion and
  rendering the result. Run it again after any change to `world.py` (world
  generation) or the Java solver; pass `--from-cache` to just re-render the
  template/JS/CSS against the already-solved data instead of resolving
  everything (seconds instead of minutes).

- **`viz/pathfind_viz_multi.html`** - the interactive 3D viewer (Three.js).
  Orbit (drag to rotate, scroll to zoom, right-drag to pan), Free-Fly
  (Minecraft-style WASD + mouse-look), and POV (rides the solved route,
  free look layered on top) camera modes; Enter toggles play/pause. Tabs
  switch between synthetic lab scenarios (each isolating one action or
  tradeoff) and real-terrain regions.

## Setup

```
pip install -r requirements.txt
```

Python 3.9+. The only non-stdlib dependencies are `numpy` and
`anvil-parser2` (for reading real `.mca` region files -- not needed if you
only care about the synthetic worlds in `world.py`).

## Usage

Regenerate everything (synthetic scenarios only), from the repo root:
```
python3 viz/build_visualization.py
```

Regenerate including every real-terrain region in the bundled `regions/`
folder (the default), or point to another directory:
```
python3 viz/build_visualization.py --mca-dir /path/to/regions
```

You can also add individual regions with `--mca-file /path/to/r.0.0.mca`.
The builder selects a walkable start/goal route independently for each region
and fails the rebuild if the full pathfinder cannot solve one within its search
limit, rather than silently publishing a route-less scenario. Already solved
everything once and just changing the viewer's template/JS/CSS? Pass
`--from-cache` to skip straight to re-rendering (seconds, not minutes).

For a real-terrain-only viewer with strict, self-consistent validation
(solved AND independently re-checked through `pathfind.py`'s own model, not
the faster Java port -- see [`viz/run_region_viz.py`](viz/run_region_viz.py)'s
module docstring for why that matters) and a machine-readable report, use the
dedicated runner (defaults to `regions/k1`):
```
python3 viz/run_region_viz.py
```
It writes `viz/rviz_k1.html` and `viz/rviz_k1_report.json`. The HTML is only
written when every selected route passes walk reachability, A*, and
edge-by-edge route validation; otherwise the report records the exact
failing region and reason.

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
"air potential" heuristic that prunes low-value mining candidates). `viz/`
(the demo generator) and [`mod/`](mod/) (see below) both solve through this
port now, not `pathfind.py`.

## Playable mod

[`mod/`](mod/) is Nether Pathfinder, a Fabric client mod that wraps this same
engine into an actual in-game tool -- `/pathfind goto <x> <y> <z>` computes
and draws a route for you to follow (command-driven, Baritone-style; nothing
runs automatically). See [`mod/README.md`](mod/README.md).

## Project layout

```
world.py, pathfind.py, replan.py, reachability.py   Python solver (pathfind.py is now a reference/library module -- see above)
mca_convert.py                                      real Minecraft region -> voxel array
viz/                                                 demo/viewer generation + pre-built interactive 3D demos (all solve via Java)
regions/                                             real Minecraft .mca region files (demo/benchmark input)
java/                                                Java port of the search core (see java/README.md)
mod/                                                 playable Fabric mod wrapping the Java port (see mod/README.md)
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
