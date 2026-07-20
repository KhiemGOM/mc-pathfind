"""
Synthetic 3D voxel world generator for Minecraft speedrun pathfinding prototype.

Grid convention: voxel[x][y][z] = block type at that coordinate.
y is vertical (up = +y). x, z are horizontal.

Block types (int codes):
  0 = AIR
  1 = DIRT       (fast to mine)
  2 = STONE      (slow to mine, needs pickaxe)
  3 = OBSIDIAN   (very slow, needs diamond pickaxe)
  -1 = VOID      (out of bounds / no block, used for ravines/world edge)
"""

import numpy as np

AIR = 0
DIRT = 1
STONE = 2
OBSIDIAN = 3
BEDROCK = 4  # unbreakable, e.g. world border / bedrock layer / barrier blocks
LAVA = 5     # impassable but NOT solid: can't stand in it or walk through it,
             # can't be mined (nothing to mine, it's a fluid), but you CAN
             # bridge over it (place a block on top to stand on) same as any
             # void gap. Falling into it is lethal, unlike a normal void fall.

# hardness -> base seconds to mine with a "reasonable" tool (hand-tuned, MC-ish)
MINE_TIME = {
    DIRT: 0.3,
    STONE: 1.2,
    OBSIDIAN: 9.0,
    # BEDROCK, LAVA intentionally absent: neither can be mined
}

UNBREAKABLE = {BEDROCK}


def generate_world(size_x=40, size_y=20, size_z=40, seed=0):
    """
    Generates a 3D world:
    - rolling terrain height (perlin-ish via smoothed random noise)
    - a ravine (void gap) cutting across it, to force bridging
    - a stone hill obstruction, to force mining
    - a floating overhang, to test full-3D reasoning (2.5D can't represent this)
    """
    rng = np.random.default_rng(seed)
    world = np.full((size_x, size_y, size_z), AIR, dtype=np.int8)

    # --- base rolling terrain heightmap ---
    coarse = rng.uniform(4, 10, size=(size_x // 5 + 2, size_z // 5 + 2))
    xs = np.linspace(0, coarse.shape[0] - 1, size_x)
    zs = np.linspace(0, coarse.shape[1] - 1, size_z)
    height = np.zeros((size_x, size_z), dtype=int)
    for xi in range(size_x):
        for zi in range(size_z):
            x0, z0 = int(xs[xi]), int(zs[zi])
            x1, z1 = min(x0 + 1, coarse.shape[0] - 1), min(z0 + 1, coarse.shape[1] - 1)
            fx, fz = xs[xi] - x0, zs[zi] - z0
            h = (coarse[x0, z0] * (1 - fx) * (1 - fz) +
                 coarse[x1, z0] * fx * (1 - fz) +
                 coarse[x0, z1] * (1 - fx) * fz +
                 coarse[x1, z1] * fx * fz)
            height[xi, zi] = int(round(h))

    for xi in range(size_x):
        for zi in range(size_z):
            h = height[xi, zi]
            world[xi, 0:h, zi] = DIRT

    # --- carve a ravine (void gap) across the map to force bridging ---
    # runs diagonally-ish through the middle
    ravine_center = lambda xi: int(size_z * 0.5 + 3 * np.sin(xi / 6.0))
    ravine_width = 4
    for xi in range(size_x):
        zc = ravine_center(xi)
        for zi in range(max(0, zc - ravine_width // 2), min(size_z, zc + ravine_width // 2)):
            world[xi, :, zi] = AIR  # carve all the way down -> void, forces bridge or long drop

    # --- add a stone obstruction wall the agent likely must cross or mine through ---
    wall_x = size_x // 2 + 10
    for zi in range(0, size_z // 2):
        for yi in range(0, 8):
            if wall_x < size_x:
                world[wall_x, yi + height[wall_x, zi], zi] = STONE if yi < 6 else AIR

    # --- floating overhang: solid block hanging in the air with air below it down to void ---
    # this is impossible to represent correctly in a 2.5D heightmap (single height per column)
    ox0, ox1 = 5, 12
    oz0, oz1 = 5, 12
    oy = 12
    world[ox0:ox1, oy, oz0:oz1] = STONE
    world[ox0:ox1, oy - 1, oz0:oz1] = AIR  # air gap under the overhang -> void below (over ravine-adjacent area)

    return world, height


def standing_y(height, x, z):
    """Feet-level y for standing on top of the terrain at column (x,z)."""
    return int(height[x, z])


def generate_cave_shortcut_world(size_x=60, size_y=20, size_z=80, wall_thickness=6,
                                  detour_gap_z=(2, 8), cave_z=(38, 42), cave_mine_depth=3,
                                  seed=3):
    """
    A giant wall spanning the full z-width, made mostly of BEDROCK (unmineable),
    except for two features that create a genuine cost tradeoff:

      1. A detour gap near one edge of the map (detour_gap_z): fully open,
         zero mining required, but far from a central start/goal -> long
         walking distance.
      2. A short STONE-plugged cave shortcut near the middle (cave_z): the
         wall here is regular STONE (mineable) instead of bedrock, and a
         tunnel is already carved most of the way through from both sides,
         leaving only `cave_mine_depth` blocks of stone to break through in
         the middle. Short distance, but costs real mining time.

    This forces the planner to weigh (mine_time + short_distance) against
    (zero_mining + long_detour_distance) -- the actual "cave shortcut vs.
    surface route" tradeoff speedrunners face, rather than a case where one
    option is simply impossible (bedrock-only gap-finding, tested separately).
    """
    world = np.full((size_x, size_y, size_z), AIR, dtype=np.int8)
    ground_h = 5
    world[:, 0:ground_h, :] = DIRT

    wall_x0 = size_x // 2
    wall_x1 = wall_x0 + wall_thickness
    wall_top = ground_h + 10

    dg0, dg1 = detour_gap_z
    cz0, cz1 = cave_z

    for zi in range(size_z):
        if dg0 <= zi < dg1:
            continue  # detour gap: fully open, no mining, but far from center
        if cz0 <= zi < cz1:
            continue  # cave shortcut zone: handled explicitly below (tunnel + plug)
        world[wall_x0:wall_x1, ground_h:wall_top, zi] = BEDROCK

    # now carve the cave shortcut tunnel explicitly: open at tunnel height
    # (ground_h .. ground_h+2, i.e. 3 blocks tall for headroom) through the
    # full wall thickness, except leave a STONE plug of cave_mine_depth in x
    tunnel_y0, tunnel_y1 = ground_h, ground_h + 3
    plug_x0 = wall_x0 + (wall_thickness - cave_mine_depth) // 2
    plug_x1 = plug_x0 + cave_mine_depth
    for zi in range(cz0, cz1):
        world[wall_x0:wall_x1, tunnel_y0:tunnel_y1, zi] = AIR  # carve tunnel air
        world[wall_x0:wall_x1, tunnel_y1:wall_top, zi] = BEDROCK  # ceiling stays solid
        world[plug_x0:plug_x1, tunnel_y0:tunnel_y1, zi] = STONE  # re-plug the middle with mineable stone

    return world, np.full((size_x, size_z), ground_h, dtype=int)


def generate_wall_world(size_x=60, size_y=20, size_z=60, wall_thickness=5, gap_x=None, seed=1):
    """
    Flat open terrain with one giant BEDROCK wall spanning the full z-width and
    most of the height, with a single deliberate gap. Forces the planner to
    either detour to the gap or discover mining is impossible (bedrock) and
    route around, never straight through. Tests "find the open terrain" behavior.
    """
    rng = np.random.default_rng(seed)
    world = np.full((size_x, size_y, size_z), AIR, dtype=np.int8)
    ground_h = 5
    world[:, 0:ground_h, :] = DIRT

    wall_x0 = size_x // 2
    wall_x1 = wall_x0 + wall_thickness
    wall_top = ground_h + 10  # tall enough that jumping over isn't an option

    if gap_x is None:
        gap_z0, gap_z1 = size_z // 2 - 3, size_z // 2 + 3
    else:
        gap_z0, gap_z1 = gap_x

    for zi in range(size_z):
        if gap_z0 <= zi < gap_z1:
            continue  # leave this section open -> the only passable route
        world[wall_x0:wall_x1, ground_h:wall_top, zi] = BEDROCK

    return world, np.full((size_x, size_z), ground_h, dtype=int)


def generate_bridge_world(size_x=50, size_y=20, size_z=20, seed=2):
    """
    Flat terrain split by one wide, deep void chasm (no far side reachable by
    falling+walking back up within a reasonable radius). Forces BRIDGE as the
    only viable action across, testing the bridging-specific behavior in isolation
    (no wall/mining confound).
    """
    world = np.full((size_x, size_y, size_z), AIR, dtype=np.int8)
    ground_h = 5
    world[:, 0:ground_h, :] = DIRT

    chasm_x0 = size_x // 2 - 4
    chasm_x1 = size_x // 2 + 4
    world[chasm_x0:chasm_x1, :, :] = AIR  # void all the way down, full z-width

    return world, np.full((size_x, size_z), ground_h, dtype=int)


def generate_ledge_world(size_x=30, size_y=20, size_z=10, ledge_x=15, ledge_height=4, lava=False):
    """
    A single straight, sheer ledge splitting the map into low ground
    (x < ledge_x) and high ground (x > ledge_x), ledge_height blocks taller
    -- too tall for CLIMB's 1-block step, no stairs, and no route around
    (the ledge face spans the full z-width, bounded by world edges on both
    sides -- and get_neighbors never mines vertically upward, so tunneling
    under it can't reach the goal's surface height either). Tests "the only
    way up is bridging" in isolation.

    lava=False (default): the ledge face is BEDROCK. There is NO mechanic
    in get_neighbors for pillaring straight up a plain solid wall -- only
    CLIMB's 1-block step, or the BRIDGE branch's rise-above-LAVA case,
    ever gain height. This variant is EXPECTED to be genuinely unsolvable:
    the point is to confirm the solver reports failure cleanly rather than
    silently doing something wrong, making that real gap visible instead
    of assuming a "bridge straight up" mechanic exists that doesn't.

    lava=True: the ledge face is a LAVA column instead of bedrock, routing
    the climb through BRIDGE's rise-above-lava branch -- the ONLY actual
    "gain height via bridging" mechanic that exists. ledge_height must be
    <= 6 (max_lava_climb in get_neighbors) for this to be solvable in a
    single application; a taller column would need a clear non-lava gap
    within every 6-block window, which this generator doesn't create.
    """
    world = np.full((size_x, size_y, size_z), AIR, dtype=np.int8)
    ground_h = 5
    high_h = ground_h + ledge_height

    world[:ledge_x, 0:ground_h, :] = DIRT               # low ground
    world[ledge_x, 0:ground_h, :] = DIRT                # base of the ledge face, level with low ground
    world[ledge_x, ground_h:high_h, :] = LAVA if lava else BEDROCK  # the ledge face itself
    world[ledge_x + 1:, 0:high_h, :] = DIRT             # high ground

    return world, None


def generate_mine_vs_crawl_world(size_x=60, size_y=20, size_z=20, plug_depth=3, seed=4):
    """
    A full BEDROCK wall spanning the WHOLE z-width and most of the height --
    no gap anywhere, no route around at all (same "no alternate route"
    design as generate_wall_world/generate_bridge_world) -- except the
    wall's bottom two layers (feet + head) are STONE for plug_depth blocks
    in x, instead of bedrock. That 2-tall mineable strip is the ONLY way
    across, forcing a genuine choice between MINE (clear both layers, walk
    through at WALK_SPEED) and BOAT_CRAWL (clear only the feet layer, crawl
    through at BOAT_CRAWL_SPEED with a one-time BOAT_CRAWL_TAX to start).

    Deliberately NOT built on generate_cave_shortcut_world's architecture:
    that world also offers a detour-around-the-wall route, so for a deep
    enough plug the detour can beat crossing the wall AT ALL regardless of
    technique, which would silently defeat a MINE-vs-BOAT_CRAWL test (no
    route through the wall ever gets taken to compare). This world has no
    detour, period -- the wall spans the full z-width -- so the solver is
    always forced to cross via the plug, isolating the MINE-vs-BOAT_CRAWL
    choice with nothing else to confound it.

    Per-block cost straight through STONE (mineTime=1.2, toolMultiplier=1.0,
    dist=1.0): MINE = 2*mineTime + dist/WALK_SPEED = 2.633/block;
    BOAT_CRAWL = mineTime + dist/BOAT_CRAWL_SPEED = 1.533/block, plus a
    one-time BOAT_CRAWL_TAX=5.0 on the first block only. Breakeven is around
    plug_depth~4.55: MINE wins for depth<=4, BOAT_CRAWL wins for depth>=5.
    """
    world = np.full((size_x, size_y, size_z), AIR, dtype=np.int8)
    ground_h = 5
    world[:, 0:ground_h, :] = DIRT

    wall_x0 = size_x // 2
    wall_x1 = wall_x0 + plug_depth
    wall_top = ground_h + 10  # tall enough that jumping/climbing over isn't an option

    world[wall_x0:wall_x1, ground_h:wall_top, :] = BEDROCK
    world[wall_x0:wall_x1, ground_h:ground_h + 2, :] = STONE  # the mineable/crawlable strip

    return world, np.full((size_x, size_z), ground_h, dtype=int)


def generate_floating_islands_world(size_x=30, size_y=25, size_z=7, num_islands=3,
                                     island_width=4, gap_width=8, height_step=5):
    """
    A staircase of small, fully-disconnected floating platforms (each one
    just a single 1-block-thick slab, void on every side and below it --
    nothing else in the world at all) rising in both x and y. Each gap is
    WIDER (in x) than the height step is TALL (in y), so neither a flat
    BRIDGE alone (never gains height) nor a single BRIDGE_UP-only pillar
    (never gains x) can cross it -- the solver has to combine horizontal
    bridging with vertical bridging: bridge across part of the gap at the
    current platform's height, pillar up once close enough (BRIDGE_UP's
    goal-relative prune only unlocks once horizontal distance drops below
    the remaining vertical need), then bridge the rest of the way across
    at the new height to land on the next island.

    With num_islands=3 this chains the combo twice in a row, so it isn't
    just "one lucky single-gap solution" -- built specifically to check
    EdgeRules.bridgeUpEdge's originY chain-tracking (added after the
    ledge_bedrock_world test) actually composes correctly with ordinary
    horizontal BRIDGE across multiple, repeated combined crossings.
    """
    world = np.full((size_x, size_y, size_z), AIR, dtype=np.int8)
    islands = []
    x0 = 1
    y = 8
    for i in range(num_islands):
        x1 = x0 + island_width
        world[x0:x1, y - 1, :] = DIRT  # 1-block-thick floating slab, nothing above/below/around it
        islands.append((x0, x1, y))
        x0 = x1 + gap_width
        y += height_step

    return world, islands


def voxel_at(world, x, y, z):
    sx, sy, sz = world.shape
    if 0 <= x < sx and 0 <= y < sy and 0 <= z < sz:
        return int(world[x, y, z])
    return -1  # void / out of bounds


def is_solid(block):
    return block in (DIRT, STONE, OBSIDIAN, BEDROCK)


def is_lava(block):
    return block == LAVA


def is_unbreakable(block):
    return block in UNBREAKABLE


def is_void(block):
    return block == -1
