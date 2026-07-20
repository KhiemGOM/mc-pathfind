"""
Weighted A* over a 3D voxel Minecraft world.

State = (x, y, z, blocks_remaining, crawling)
  crawling is 0/1: whether the agent is currently in an active boat-crawl
  sequence (see BOAT_CRAWL below). Tracked in state because the crawl
  technique has a one-time startup tax that should NOT be re-charged on
  every consecutive crawl move, only when starting a new crawl segment.

Actions considered per step (6-connectivity + diagonal horizontal):
  - SPRINT     : moving onto/through an air voxel with solid support below
  - MINE       : moving into a solid voxel; Steve is 2 blocks tall, so this
                 always clears BOTH foot and head level (mining cost is
                 charged for each level that is actually solid)
  - BOAT_CRAWL : a real MC speedrunning technique using a boat placed on
                 your head to compress your hitbox, letting you tunnel
                 through a 1-block-tall gap (foot level only, no head
                 clearance needed) instead of the normal 2-tall dig. Much
                 faster per block once started, but has a fixed BOAT_CRAWL_TAX
                 startup cost the first time you enter a crawl sequence
                 (getting the boat out, positioning, entering crawl stance).
                 Consecutive crawl moves after the first do not re-pay this
                 tax; exiting and later re-entering a crawl does.
  - BRIDGE     : moving into a void voxel with void below (place a block to
                 stand on); costs real time, a risk penalty, and a
                 scarcity-scaled block tax (see BLOCK_TAX_* below)
  - FALL       : moving down into air with no support (allowed up to a safe
                 fall height)
  - CLIMB      : moving up one level onto solid ground (jump)

Costs are in simulated seconds. Heuristic = octile horizontal distance / max
sprint speed, which is an admissible (never-overestimating) underestimate
since no action is ever faster than free sprinting.
"""

import heapq
import math
from dataclasses import dataclass, field
from world import voxel_at, is_solid, is_void, is_unbreakable, is_lava, MINE_TIME, AIR

UNMINEABLE_COST = float("inf")

BOAT_CRAWL_TAX = 5.0     # sec, one-time fixed cost to START a boat-crawl sequence
                         # (only charged on the transition into crawling, not per block)
BOAT_CRAWL_SPEED = 3.0   # blocks/sec while crawling (slower than normal walk/sprint,
                         # since you're crouched/compressed and moving carefully)

SPRINT_SPEED = 5.6      # blocks/sec, MC sprint speed
WALK_SPEED = 4.3        # blocks/sec, normal walk (used while mining through)
JUMP_PENALTY = 0.15     # sec, extra cost for a step-up / jump
PLACE_TIME = 0.20       # sec, time to place a bridge block
BRIDGE_RISK_PENALTY = 1.0  # sec, flat cost added per bridge placement: accounts for the
                           # real risk a sprint-bridge carries (misplaced block, fall into
                           # void/lava, clutch-timing failure) that a flat time cost doesn't
                           # capture. This is "insurance premium" time, not physical travel time.
BLOCK_TAX_BASE = 0.6       # sec, base scarcity cost charged per block spent bridging,
                           # on top of BRIDGE_RISK_PENALTY and travel time. Represents the
                           # opportunity cost of consuming a finite resource: a block spent
                           # now is a block you might need later for a real gap, so bridging
                           # should be discouraged in proportion to how much of your remaining
                           # supply it costs, not treated as free once you have any blocks.
BLOCK_TAX_SCARCITY_SCALE = 3.0  # multiplier applied to BLOCK_TAX_BASE as inventory gets low;
                                 # at full inventory the tax is BLOCK_TAX_BASE, and it scales
                                 # up smoothly toward BLOCK_TAX_BASE * (1+BLOCK_TAX_SCARCITY_SCALE)
                                 # as blocks_remaining approaches 0, so running low on blocks
                                 # makes every further bridge much more expensive, not just
                                 # the last one that would fail outright.
CLUTCH_THRESHOLD = 3      # blocks; drops beyond this require a "clutch" (water bucket /
                          # similar) to survive without fall damage -- but a clutch always
                          # works, it's a real, reliable speedrunning technique, not a risk.
CLUTCH_SETUP_TIME = 0.4   # sec, one-time cost for placing/using the clutch item on a drop
                          # that exceeds CLUTCH_THRESHOLD (below it, no fall damage occurs
                          # anyway in real MC, so no clutch or extra cost is needed)
LAVA_DEATH_PENALTY = 1e6  # falling INTO or THROUGH lava remains genuinely lethal -- no
                          # clutch technique saves you from lava. This is the only fall
                          # outcome that's ever treated as forbidden.

# 8 horizontal directions (cardinal + diagonal), MC allows diagonal strafing at full speed
DIRS_H = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]

# --- PARKOUR (includes boat-jump and equivalent long-jump techniques) ---
MIN_PARKOUR_DIST = 2.0   # blocks; short jumps ARE sometimes genuinely the cheapest
                         # crossing (parkour's flat per-jump risk vs. bridge's
                         # per-block risk means a narrow gap can favor a short
                         # jump over 1-2 bridge placements) so this floor only
                         # excludes trivially adjacent cells (dist<=2, already a
                         # normal SPRINT/CLIMB step), not the useful short-jump
                         # range. Setting this too high forces the search to only
                         # find much longer kernel-sampled jumps, which then
                         # overshoot narrow gaps and land far past where they
                         # needed to -- correct on cost grounds given the flat
                         # risk base, but wasteful and a worse fit for what
                         # PARKOUR should represent (clearing a specific gap).
MAX_PARKOUR_DIST = 7.0   # blocks; the practical limit of boat-jump-assisted long jumps
PARKOUR_SPEED = 6.5      # blocks/sec-equivalent for the leap itself (a bit faster than
                         # sprint since it's covering the gap in one committed motion)
PARKOUR_RISK_BASE = 0.8  # sec, flat risk premium for attempting a jump at all (setup,
                         # aim, commit) -- a missed landing is a real failure mode
PARKOUR_RISK_PER_BLOCK = 0.15  # sec, additional risk cost per block of jump distance;
                               # longer jumps are progressively less forgiving to land


PARKOUR_N_DIRECTIONS = 16   # compass directions sampled around the full circle
PARKOUR_N_DIST_STEPS = 8    # distinct distances sampled per direction, spread
                            # evenly between MIN_PARKOUR_DIST and MAX_PARKOUR_DIST.
                            # Higher than before since MIN_PARKOUR_DIST now starts
                            # low (short jumps can be genuinely optimal), so more
                            # steps are needed to actually sample that short end
                            # rather than jumping straight to the coarse long range.


def _build_parkour_kernel():
    """
    Precompute a SPARSE set of (dx, dy, dz, dist) offsets approximating valid
    PARKOUR jumps, rather than enumerating every integer lattice point in the
    outer shell (which is ~1000 cells at MAX_PARKOUR_DIST=11 and would make
    every single node expansion checked against ~1000 extra candidate edges
    -- intractable for a search that already runs over thousands of nodes).

    Instead we sample PARKOUR_N_DIRECTIONS compass directions around the full
    circle, each at PARKOUR_N_DIST_STEPS evenly-spaced distances between
    MIN_PARKOUR_DIST and MAX_PARKOUR_DIST, times 3 dy offsets. This gives a
    kernel of size (directions x distances x 3), typically a couple hundred
    entries -- roughly 5x smaller than the full lattice -- while still
    covering the full range of directions and distances a real jump could
    take. The tradeoff is angular coarseness: a specific exact-integer target
    a few degrees off from a sampled direction might be missed, but since
    this is a heuristic (weighted, bounded-suboptimal) search to begin with,
    and any real terrain target is "close enough" to one of 16 evenly-spaced
    directions to be useful, this is an acceptable and deliberate tradeoff
    for tractability. Building once at import time; get_neighbors just
    iterates this static list per call.

    Restricting to distance > MIN_PARKOUR_DIST is deliberate: short-range
    cells are already reachable cheaply by SPRINT/CLIMB/BRIDGE, so sampling
    them here would add branching factor for essentially no benefit -- a
    short "parkour jump" would never beat a plain sprint step once the risk
    premium is priced in.
    """
    kernel = []
    seen_offsets = set()
    for i in range(PARKOUR_N_DIRECTIONS):
        angle = 2 * math.pi * i / PARKOUR_N_DIRECTIONS
        for j in range(PARKOUR_N_DIST_STEPS):
            dist_target = MIN_PARKOUR_DIST + (MAX_PARKOUR_DIST - MIN_PARKOUR_DIST) * (j + 1) / PARKOUR_N_DIST_STEPS
            dx = round(dist_target * math.cos(angle))
            dz = round(dist_target * math.sin(angle))
            actual_dist = math.sqrt(dx * dx + dz * dz)
            if actual_dist <= MIN_PARKOUR_DIST or actual_dist > MAX_PARKOUR_DIST:
                continue
            for dy in (-1, 0, 1):
                key = (dx, dy, dz)
                if key in seen_offsets:
                    continue
                seen_offsets.add(key)
                kernel.append((dx, dy, dz, actual_dist))
    return kernel


PARKOUR_KERNEL = _build_parkour_kernel()


@dataclass(order=True)
class PQItem:
    priority: float
    counter: int
    state: tuple = field(compare=False)
    g: float = field(compare=False)
    action: str = field(compare=False)


def block_tax(blocks_remaining):
    """
    Scarcity cost (in simulated seconds) for spending one block right now,
    given how many blocks remain BEFORE this spend. Blends in smoothly: at
    generous inventory the tax sits near BLOCK_TAX_BASE; as blocks_remaining
    gets low the tax rises toward BLOCK_TAX_BASE * (1 + BLOCK_TAX_SCARCITY_SCALE),
    so the planner increasingly prefers mining/detouring over bridging once
    it's down to its last few blocks, rather than only refusing to bridge
    once it literally has zero left.
    """
    if blocks_remaining <= 0:
        return float("inf")
    scarcity = BLOCK_TAX_SCARCITY_SCALE / blocks_remaining
    return BLOCK_TAX_BASE * (1.0 + scarcity)


def octile(dx, dz):
    dx, dz = abs(dx), abs(dz)
    return (dx + dz) + (math.sqrt(2) - 2) * min(dx, dz)


def heuristic(pos, goal):
    x, y, z = pos
    gx, gy, gz = goal
    horiz = octile(gx - x, gz - z)
    vert = abs(gy - y)
    # divide by max speed; vertical movement never faster than sprint, so this stays admissible
    return (horiz + vert) / SPRINT_SPEED


def find_landing_y(world, x, y, z, max_drop=None):
    """
    Search downward from y for the first solid voxel; returns (landing_y, drop, passed_lava).

    passed_lava is True if EITHER the fall path crosses a lava cell while descending,
    OR the landing cell itself (the empty space directly above the solid ground found)
    is lava -- both are lethal in real MC. The latter catches the case where lava is
    sitting directly on top of solid ground: the solid-block scan would otherwise
    return that lava cell as a valid "landing spot" since it never descends past it.
    """
    sx, sy, sz = world.shape
    drop = 0
    yy = y
    passed_lava = False
    while yy >= 0:
        v = voxel_at(world, x, yy, z)
        if is_lava(v):
            passed_lava = True
        if is_solid(v):
            landing_y = yy + 1
            landing_cell = voxel_at(world, x, landing_y, z)      # foot level at landing
            landing_head = voxel_at(world, x, landing_y + 1, z)  # head level at landing
            if is_lava(landing_cell) or is_lava(landing_head):
                passed_lava = True
            return landing_y, drop, passed_lava
        yy -= 1
        drop += 1
        if max_drop is not None and drop > max_drop:
            return None, drop, passed_lava
    return None, drop, passed_lava  # bottomless void


def _diagonal_corner_blocked(world, x, y, z, dx, dz):
    """
    MC never lets you cut a diagonal corner when BOTH flanking columns are
    solid -- your 2-tall hitbox can't clip through two solid corners at once.
    If only one of the two is solid you can still squeeze past it, so this
    only blocks the case where neither side offers a gap.
    Cardinal moves (dx==0 or dz==0) have no corner to cut.
    """
    if dx == 0 or dz == 0:
        return False
    corner1_blocked = is_solid(voxel_at(world, x + dx, y, z)) or is_solid(voxel_at(world, x + dx, y + 1, z))
    corner2_blocked = is_solid(voxel_at(world, x, y, z + dz)) or is_solid(voxel_at(world, x, y + 1, z + dz))
    return corner1_blocked and corner2_blocked


def get_neighbors(world, state, tool_multiplier=1.0):
    """
    Yields (new_state, cost, action_label) tuples from the given state.
    state = (x, y, z, blocks, crawling)
    """
    x, y, z, blocks, crawling = state
    sx, sy, sz = world.shape

    # --- horizontal moves (sprint / mine / boat-crawl / bridge / step-up) ---
    for dx, dz in DIRS_H:
        nx, nz = x + dx, z + dz
        if not (0 <= nx < sx and 0 <= nz < sz):
            continue
        if _diagonal_corner_blocked(world, x, y, z, dx, dz):
            continue
        dist = math.sqrt(dx * dx + dz * dz)

        below_target = voxel_at(world, nx, y - 1, nz)  # placement/floor cell
        target = voxel_at(world, nx, y, nz)             # foot level at target
        head = voxel_at(world, nx, y + 1, nz)            # head level at target

        if is_solid(target):
            # Steve is 2 blocks tall: normal mining always needs foot AND
            # head level cleared, regardless of what's actually there.
            # Lava at head level blocks this just like unbreakable: you can't
            # mine a fluid out of the way, and standing there means your head
            # is literally in lava.
            if is_unbreakable(target) or is_unbreakable(head) or is_lava(head):
                pass  # normal 2-tall mining blocked; boat-crawl may still work below
            else:
                mine_cost = MINE_TIME.get(target, 2.0) * tool_multiplier
                if is_solid(head):
                    mine_cost += MINE_TIME.get(head, 2.0) * tool_multiplier
                cost = mine_cost + dist / WALK_SPEED
                yield (nx, y, nz, blocks, 0), cost, "MINE"  # exits any crawl (crawling=0)

            # Boat-crawl alternative: only requires FOOT level cleared, head
            # can stay solid (compressed hitbox ducks under it). Lava at head
            # level does NOT enable crawling either: the boat-crawl compresses
            # your hitbox to fit a 1-tall solid gap, it doesn't make you
            # immune to touching lava if your head-space is a fluid.
            if is_solid(head) and not is_unbreakable(target):
                crawl_mine_cost = MINE_TIME.get(target, 2.0) * tool_multiplier
                startup_tax = 0.0 if crawling else BOAT_CRAWL_TAX
                cost = crawl_mine_cost + startup_tax + dist / BOAT_CRAWL_SPEED
                yield (nx, y, nz, blocks, 1), cost, "BOAT_CRAWL"
            continue

        # --- Flat bridge/sprint/fall at the CURRENT y (no elevation change) ---
        # This requires checking all three relevant cells together:
        #   below_target (y-1) = what you'd stand ON (or place a block into)
        #   target       (y)   = where your feet go
        #   head         (y+1) = where your head goes
        # A flat crossing at this y is only valid if target and head are both
        # clear of lava (you can never stand with any part of your body in
        # lava, regardless of what gets placed at foot level).
        target_or_head_is_lava = is_lava(target) or is_lava(head)
        # Also blocks a flat crossing if head is solid -- a low overhang
        # (target/foot level open, but a solid block directly above) means
        # your 2-tall body can't walk into or through that column at all,
        # same physical reason CLIMB checks head_clear. This was previously
        # missed here (only lava was checked), letting SPRINT/BRIDGE/FALL
        # walk the agent's head straight through a solid ceiling -- caught
        # by the Java port's independent PathValidator re-checking every
        # returned edge against the world rules from scratch.
        target_or_head_blocked = target_or_head_is_lava or is_solid(head)

        if is_solid(below_target) and not target_or_head_blocked:
            # solid floor already exists, feet/head both clear -> normal sprint
            cost = dist / SPRINT_SPEED
            yield (nx, y, nz, blocks, 0), cost, "SPRINT"
            continue

        if not is_solid(below_target) and not target_or_head_blocked:
            # below_target is void/air/lava (no solid floor) but feet/head
            # space itself is clear -> can bridge flat here: place a block at
            # y-1 (into the void or onto/into the lava below), stand at y.
            if blocks > 0:
                tax = block_tax(blocks)
                cost = PLACE_TIME + BRIDGE_RISK_PENALTY + tax + dist / SPRINT_SPEED
                yield (nx, y, nz, blocks - 1, 0), cost, "BRIDGE"
            # also offer: fall to whatever landing exists below. Any height is
            # survivable via clutch (water bucket / similar) -- speedrunners
            # do this routinely and reliably, so a big drop costs a bit of
            # setup time, not a death chance. The only thing that's genuinely
            # forbidden is passing through or landing in lava, which no
            # clutch technique saves you from.
            landing_y, drop, passed_lava = find_landing_y(world, nx, y - 1, nz, max_drop=None)
            if landing_y is not None:
                fall_cost = dist / SPRINT_SPEED + drop * 0.05
                if drop > CLUTCH_THRESHOLD:
                    fall_cost += CLUTCH_SETUP_TIME
                if passed_lava:
                    fall_cost += LAVA_DEATH_PENALTY  # the one truly forbidden outcome
                yield (nx, landing_y, nz, blocks, 0), fall_cost, "FALL"
            continue

        if target_or_head_is_lava:
            # Feet and/or head would be IN lava at this y -- flat crossing is
            # never valid here no matter what's below, since placing a block
            # at y-1 doesn't remove lava occupying y or y+1. The only way
            # across is to rise above the lava's height entirely. We don't
            # know how deep/tall the lava column is from here, so we search
            # upward for the first y where target and head both clear lava,
            # and offer a bridge that both raises up AND places the
            # supporting block, capped to a reasonable climb height so this
            # doesn't degenerate into an unbounded search.
            max_lava_climb = 6
            for climb in range(1, max_lava_climb + 1):
                ty = y + climb
                clear_target = voxel_at(world, nx, ty, nz)
                clear_head = voxel_at(world, nx, ty + 1, nz)
                support = voxel_at(world, nx, ty - 1, nz)
                if is_lava(clear_target) or is_lava(clear_head):
                    continue  # still inside the lava column at this height, keep climbing
                if is_solid(clear_target) or is_solid(clear_head):
                    break  # ran into solid ground/ceiling above the lava; no clean bridge line up
                # found a height where both feet and head clear the lava --
                # bridge here (placing support at ty-1, which may itself be
                # capping the lava's surface if support is lava or void)
                if blocks > 0:
                    tax = block_tax(blocks)
                    climb_cost = climb * JUMP_PENALTY  # rough cost for the extra height gained
                    cost = PLACE_TIME + BRIDGE_RISK_PENALTY + tax + dist / SPRINT_SPEED + climb_cost
                    yield (nx, ty, nz, blocks - 1, 0), cost, "BRIDGE"
                break
            continue

    # --- step-up (jump) explicitly: target at y+1 has solid floor ---
    for dx, dz in DIRS_H:
        nx, nz = x + dx, z + dz
        if not (0 <= nx < sx and 0 <= nz < sz):
            continue
        if _diagonal_corner_blocked(world, x, y, z, dx, dz):
            continue
        dist = math.sqrt(dx * dx + dz * dz)
        target_up = voxel_at(world, nx, y + 1, nz)
        below_up = voxel_at(world, nx, y, nz)
        head_clear = voxel_at(world, nx, y + 2, nz)
        if (not is_solid(target_up) and not is_lava(target_up)
                and is_solid(below_up)
                and not is_solid(head_clear) and not is_lava(head_clear)):
            cost = dist / SPRINT_SPEED + JUMP_PENALTY
            yield (nx, y + 1, nz, blocks, 0), cost, "CLIMB"

    # --- PARKOUR (long jump, includes boat-jump and equivalent techniques) ---
    # Gate this behind a cheap "is this even a ledge?" check first: scanning
    # the full ~180-entry kernel from EVERY node (including ones deep in
    # open flat terrain where a long jump is never useful) would multiply
    # the cost of every single expansion by ~180x for no benefit. We only
    # bother if at least one immediately-adjacent cell lacks solid footing
    # (void, lava, or a drop), i.e. the agent is actually standing at an edge.
    at_a_ledge = False
    for dx, dz in DIRS_H:
        nx, nz = x + dx, z + dz
        if not (0 <= nx < sx and 0 <= nz < sz):
            continue
        adj_below = voxel_at(world, nx, y - 1, nz)
        if not is_solid(adj_below):
            at_a_ledge = True
            break

    if at_a_ledge:
        for kdx, kdy, kdz, kdist in PARKOUR_KERNEL:
            nx, ny, nz = x + kdx, y + kdy, z + kdz
            if not (0 <= nx < sx and 0 <= ny < sy and 0 <= nz < sz):
                continue

            # Sample at integer-block resolution along the straight-line
            # trajectory (excluding the launch and landing cells themselves).
            # Collision detection needs every obstructing column caught, so
            # unlike a "find at least one gap" check, we can't use a sparse
            # fixed sample count here -- a small number of samples can easily
            # skip over the one solid column blocking the path (rounding a
            # handful of t-fractions can jump straight over a single-block
            # wall). One sample per block of distance is worth the extra
            # cost given the ledge-gate already limits how often this runs.
            n_samples = max(2, int(round(kdist)))
            crosses_gap = False
            trajectory_blocked = False
            for s in range(1, n_samples):
                t = s / n_samples
                sx_ = round(x + kdx * t)
                sz_ = round(z + kdz * t)
                sy_ = round(y + kdy * t)
                sample_foot = voxel_at(world, sx_, sy_, sz_)
                sample_head = voxel_at(world, sx_, sy_ + 1, sz_)
                if is_solid(sample_foot) or is_lava(sample_foot) or is_solid(sample_head) or is_lava(sample_head):
                    trajectory_blocked = True
                    break
                sample_below = voxel_at(world, sx_, sy_ - 1, sz_)
                if not is_solid(sample_below):
                    crosses_gap = True
            if trajectory_blocked:
                continue  # something solid or lava is directly in the flight path -- can't jump through it
            if not crosses_gap:
                continue  # this "jump" would just be sailing over normal solid ground

            land_target = voxel_at(world, nx, ny, nz)
            land_head = voxel_at(world, nx, ny + 1, nz)
            land_support = voxel_at(world, nx, ny - 1, nz)
            # must land on solid ground, with feet/head both clear and lava-free
            if not is_solid(land_support):
                continue
            if is_solid(land_target) or is_lava(land_target):
                continue
            if is_solid(land_head) or is_lava(land_head):
                continue
            risk = PARKOUR_RISK_BASE + PARKOUR_RISK_PER_BLOCK * kdist
            cost = risk + kdist / PARKOUR_SPEED
            yield (nx, ny, nz, blocks, 0), cost, "PARKOUR"

    # --- vertical mining (straight down / up through solid, rare but keeps it general) ---
    below_here = voxel_at(world, x, y - 1, z)
    if is_solid(below_here) and not is_unbreakable(below_here):
        cost = MINE_TIME.get(below_here, 2.0) * tool_multiplier + 0.2
        yield (x, y - 1, z, blocks, 0), cost, "MINE_DOWN"


def weighted_astar(world, start, goal, blocks_available=32, epsilon=1.5, tool_multiplier=1.0, max_expansions=200000):
    """
    start, goal = (x, y, z) tuples.
    epsilon > 1 gives bounded-suboptimal but much faster search (good for huge MC worlds).
    Returns (path, total_cost, actions) or (None, None, None) if unreachable.
    """
    start_state = (start[0], start[1], start[2], blocks_available, 0)
    goal_xyz = goal

    counter = 0
    open_heap = []
    g_score = {start_state: 0.0}
    came_from = {}
    action_used = {}

    h0 = heuristic(start, goal_xyz)
    heapq.heappush(open_heap, PQItem(epsilon * h0, counter, start_state, 0.0, "START"))
    closed = set()
    expansions = 0

    while open_heap:
        item = heapq.heappop(open_heap)
        state = item.state
        if state in closed:
            continue
        closed.add(state)
        expansions += 1
        if expansions > max_expansions:
            return None, None, None, expansions

        x, y, z, blocks, crawling = state
        if (x, y, z) == goal_xyz:
            # reconstruct
            path = [state]
            actions = []
            s = state
            while s in came_from:
                actions.append(action_used[s])
                s = came_from[s]
                path.append(s)
            path.reverse()
            actions.reverse()
            return path, g_score[state], actions, expansions

        for new_state, cost, action in get_neighbors(world, state, tool_multiplier):
            if new_state in closed:
                continue
            tentative_g = g_score[state] + cost
            if tentative_g < g_score.get(new_state, float("inf")):
                g_score[new_state] = tentative_g
                came_from[new_state] = state
                action_used[new_state] = action
                nx, ny, nz, _, _ = new_state
                h = heuristic((nx, ny, nz), goal_xyz)
                counter += 1
                heapq.heappush(open_heap, PQItem(tentative_g + epsilon * h, counter, new_state, tentative_g, action))

    return None, None, None, expansions
