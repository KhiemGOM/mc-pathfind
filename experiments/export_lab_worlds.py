"""
One-off dev tool: exports the hand-built SYNTHETIC test worlds from world.py
(generate_wall_world, generate_bridge_world, generate_cave_shortcut_world,
generate_world) to .wbin, so the Java solvers -- specifically the new
AirPotential MINE pruning -- can be checked against known, hand-designed
scenarios instead of only real .mca terrain. These are the "artificial lab
room" tests: each one isolates a specific behavior (find-the-gap, bridge-only,
mine-vs-detour cost tradeoff) with a geometry simple enough that the
"correct" answer is obvious by inspection, unlike real terrain where you
can't eyeball whether a path is actually optimal.

generate_cave_shortcut_world is the one that matters most for THIS check:
it's built specifically to force a mine_time-vs-detour_distance tradeoff,
which is exactly what MINE_PRUNE_THRESHOLD / OLD_GROUND_CUTOFF touch. If the
prune wrongly skips the shortcut, the solver will take the (much more
expensive) detour instead -- easy to see by comparing path cost against a
hand-computed expectation, unlike on real terrain.

Usage: python3 experiments/export_lab_worlds.py [output_dir]
"""
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from world import (
    generate_world, generate_wall_world, generate_bridge_world,
    generate_cave_shortcut_world, generate_ledge_world, generate_floating_islands_world,
    generate_mine_vs_crawl_world,
)

MAGIC = b"MCPW"
VERSION = 1


def write_wbin(world, start, goal, blocks_available, output_path):
    size_x, size_y, size_z = world.shape
    header = struct.pack(
        "<4s14i",
        MAGIC, VERSION,
        size_x, size_y, size_z,
        0, 0, 0,  # origin -- these are standalone arrays, not sliced from a larger region
        start[0], start[1], start[2],
        goal[0], goal[1], goal[2],
        blocks_available,
    )
    payload = world.tobytes()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(header + payload)
    print(f"wrote {output_path}: shape={world.shape} start={start} goal={goal}")


def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("java/benchmark/data/lab")

    # wall_world: gap at z=27..33 (size_z=60), wall at x=30..35. Start/goal
    # both at z=10 (OUTSIDE the gap) so the solver must detour in z to find
    # the only passable opening -- tests "find the open terrain", not mining
    # (wall is BEDROCK, unmineable).
    world, _ = generate_wall_world()
    write_wbin(world, (10, 5, 10), (50, 5, 10), 32, out_dir / "wall_world.wbin")

    # bridge_world: chasm at x=21..29 (size_x=50), full z-width void. Only
    # BRIDGE can cross it (no walkable floor, no wall to route around).
    world, _ = generate_bridge_world()
    write_wbin(world, (10, 5, 10), (40, 5, 10), 32, out_dir / "bridge_world.wbin")

    # cave_shortcut_world: THE mine-vs-detour test. detour_gap_z=(2,8) near
    # one edge (size_z=80), cave_z=(38,42) near the middle with a 3-block
    # STONE plug (cave_mine_depth=3) otherwise-tunneled through a BEDROCK
    # wall at x=30..36. Start/goal both at z=40 (dead center of the cave
    # shortcut, size_z=80/2=40) -- straight through means MINE 3 stone
    # blocks (~3.6s mine time); going around via the z=2..8 gap costs an
    # extra ~2*(40-5)=70 blocks of walking (~12.5s at SPRINT_SPEED=5.6) each
    # way. Mining is unambiguously cheaper -- any correct/reasonably-tuned
    # solver MUST take the shortcut, so this is a hard pass/fail check, not
    # just "did cost change a little".
    world, _ = generate_cave_shortcut_world()
    write_wbin(world, (10, 5, 40), (50, 5, 40), 32, out_dir / "cave_shortcut_world.wbin")

    # mine_vs_crawl_{short,long}_world: the MINE-vs-BOAT_CRAWL crossover
    # test -- see generate_mine_vs_crawl_world's docstring for the full cost
    # derivation (breakeven around plug_depth~4.55). Unlike
    # cave_shortcut_world, there's no detour-around option here at all (the
    # wall spans the full z-width), so the solver is always forced through
    # the plug -- the ONLY thing left to decide is which technique. depth=3
    # is on the MINE side of the breakeven, depth=8 is well on the
    # BOAT_CRAWL side; both use the same wall_x=size_x//2=30, so the same
    # start/goal pair works for both.
    world, _ = generate_mine_vs_crawl_world(plug_depth=3)
    write_wbin(world, (10, 5, 10), (50, 5, 10), 32, out_dir / "mine_vs_crawl_short_world.wbin")

    world, _ = generate_mine_vs_crawl_world(plug_depth=8)
    write_wbin(world, (10, 5, 10), (50, 5, 10), 32, out_dir / "mine_vs_crawl_long_world.wbin")

    # generate_world: the general-purpose scenario (ravine forcing BRIDGE,
    # stone obstruction forcing MINE, floating overhang forcing full-3D
    # reasoning). No single hand-computed expected cost -- included as a
    # broad "did anything obviously break" regression check, not a
    # pass/fail tradeoff test like cave_shortcut_world.
    world, height = generate_world()
    sx, sy, sz = world.shape
    start = (2, int(height[2, 2]), 2)
    goal = (sx - 3, int(height[sx - 3, sz - 3]), sz - 3)
    write_wbin(world, start, goal, 32, out_dir / "rolling_terrain_world.wbin")

    # ledge_world: straight sheer cliff, low ground -> high ground, no
    # stairs, no route around. bedrock face has NO way up at all UNLESS
    # BRIDGE_UP exists (no plain pillaring mechanic before that). lava
    # face routes through the pre-existing "rise above lava" BRIDGE branch
    # regardless. Goal placed just past the ledge (not deep across the
    # map): BRIDGE_UP's goal-relative prune only fires when the goal's y
    # delta exceeds its x/z delta -- a goal far across the map would keep
    # horizontal distance dominant right up to the ledge, never tripping
    # the prune's "close to the thing" condition, so this placement is
    # what actually exercises it instead of silently always skipping it.
    ledge_x, ledge_height = 15, 4
    close_goal_x = ledge_x + 2
    world, _ = generate_ledge_world(ledge_x=ledge_x, ledge_height=ledge_height, lava=False)
    sx, sy, sz = world.shape
    write_wbin(world, (5, 5, 5), (close_goal_x, 5 + ledge_height, 5), 32, out_dir / "ledge_bedrock_world.wbin")

    world, _ = generate_ledge_world(ledge_x=ledge_x, ledge_height=ledge_height, lava=True)
    write_wbin(world, (5, 5, 5), (close_goal_x, 5 + ledge_height, 5), 32, out_dir / "ledge_lava_world.wbin")

    # floating_islands_world: a staircase of disconnected floating slabs,
    # each gap wider (in x) than the height step is tall (in y) -- neither
    # flat BRIDGE alone nor BRIDGE_UP alone can cross a gap, only a
    # horizontal-then-vertical-then-horizontal combo. Chains this twice
    # (3 islands) to check the combo composes correctly across repeated
    # crossings, not just once.
    world, islands = generate_floating_islands_world()
    start_x0, start_x1, start_y = islands[0]
    goal_x0, goal_x1, goal_y = islands[-1]
    mid_z = world.shape[2] // 2
    start = ((start_x0 + start_x1) // 2, start_y, mid_z)
    goal = ((goal_x0 + goal_x1) // 2, goal_y, mid_z)
    write_wbin(world, start, goal, 32, out_dir / "floating_islands_world.wbin")


if __name__ == "__main__":
    main()
