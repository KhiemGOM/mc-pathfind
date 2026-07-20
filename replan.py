"""
Replanning layer for the MC pathfinder.

Real speedrunning doesn't have the whole world loaded upfront: terrain is
revealed as you move (render distance / explored chunks). This module
simulates that:

  - `known_world` starts as a copy of the true world but masked to "unknown"
    outside a reveal radius of the start.
  - As the agent advances along its planned path, cells within reveal_radius
    of its current position are revealed (copied from the true world into
    known_world).
  - Unknown cells are treated as AIR (optimistic assumption: "probably
    passable") so the initial plan isn't overly conservative.
  - If a reveal changes a cell that lies on (or near) the remaining planned
    path in a way that invalidates it (e.g. an assumed-open cell turns out
    solid, or vice versa in a way that changes the best route), replan from
    the agent's current position using the goal and updated known_world.

This is a pragmatic stand-in for full D* Lite: instead of incremental graph
repair, we just detect invalidation and re-run weighted A* from the current
position. For MC-scale worlds where each replan is local (small remaining
distance) this is fast enough in practice, and much simpler to reason about
and extend than implementing full incremental search.
"""

import numpy as np
from world import AIR, voxel_at
from pathfind import weighted_astar


UNKNOWN = -2  # sentinel distinct from AIR(0) and VOID(-1)


def make_known_world(true_world, start, reveal_radius):
    """Initialize known_world as all-UNKNOWN except a sphere around start."""
    known = np.full_like(true_world, UNKNOWN)
    reveal(known, true_world, start, reveal_radius)
    return known


def reveal(known_world, true_world, center, radius):
    """Copy true_world into known_world within `radius` (Chebyshev) of center. Returns True if anything changed."""
    x0, y0, z0 = center
    sx, sy, sz = true_world.shape
    xr = range(max(0, x0 - radius), min(sx, x0 + radius + 1))
    yr = range(max(0, y0 - radius), min(sy, y0 + radius + 1))
    zr = range(max(0, z0 - radius), min(sz, z0 + radius + 1))
    changed = False
    for x in xr:
        for y in yr:
            for z in zr:
                if known_world[x, y, z] == UNKNOWN:
                    known_world[x, y, z] = true_world[x, y, z]
                    changed = True
    return changed


def known_voxel_at(known_world, x, y, z, ground_y=None, optimistic=True):
    """
    Like world.voxel_at, but UNKNOWN cells resolve to an assumed value.

    ground_y: the y-level the agent is currently treating as "standing
    height" for this query's column context (passed through from the search's
    reference point). Cells at y < ground_y that are UNKNOWN resolve to DIRT
    (assume normal solid ground continues, avoiding a bridge-storm from
    assuming bottomless void everywhere unseen). Cells at y >= ground_y that
    are UNKNOWN resolve to AIR (assume walkable headroom, avoiding a
    mine-storm from assuming solid obstruction everywhere unseen).

    If ground_y is None (caller doesn't have column context), fall back to a
    single global assumption: AIR if optimistic else STONE.
    """
    sx, sy, sz = known_world.shape
    if not (0 <= x < sx and 0 <= y < sy and 0 <= z < sz):
        return -1
    v = int(known_world[x, y, z])
    if v != UNKNOWN:
        return v
    if not optimistic:
        return 2  # STONE: pessimistic prior, assume obstruction everywhere
    if ground_y is not None:
        return 1 if y < ground_y else AIR  # DIRT below assumed ground, AIR above
    return AIR


class KnownWorldView:
    """
    Thin wrapper so pathfind.get_neighbors (which calls world.voxel_at) can
    operate over a known_world array with UNKNOWN resolved optimistically,
    without changing pathfind.py's world-shape assumptions. Duck-types the
    parts of a numpy array that voxel_at/is_solid/is_void need.
    """
    def __init__(self, known_world, optimistic=True):
        self.known_world = known_world
        self.optimistic = optimistic
        self.shape = known_world.shape

    def __getitem__(self, idx):
        # only used via voxel_at's bounds-checked access pattern in pathfind;
        # we bypass that by monkeypatching voxel_at usage below instead.
        raise NotImplementedError("use known_voxel_at directly")


def replanning_run(true_world, start, goal, blocks_available=32, reveal_radius=8,
                    epsilon=1.5, optimistic=True, max_replans=50, verbose=True):
    """
    Simulates step-by-step traversal with limited visibility, replanning
    whenever newly revealed terrain invalidates the current plan.

    Returns a log dict with the final executed path/actions and a list of
    replan events (position + reason) for visualization.
    """
    import pathfind as pf

    known = make_known_world(true_world, start, reveal_radius)

    # monkeypatch pathfind's voxel_at to read through the known-world lens
    # for the duration of this run, then restore it. This keeps pathfind.py's
    # core logic completely unaware of the known/unknown distinction.
    real_voxel_at = pf.voxel_at
    real_get_neighbors = pf.get_neighbors

    def patched_get_neighbors(world_arr, state, tool_multiplier=1.0):
        # ground_y = this state's own y -> UNKNOWN cells at/above it assume AIR
        # (walkable headroom), UNKNOWN cells below it assume DIRT (solid ground).
        state_y = state[1]

        def scoped_voxel_at(w, x, y, z):
            return known_voxel_at(w, x, y, z, ground_y=state_y, optimistic=optimistic)

        pf.voxel_at = scoped_voxel_at
        try:
            yield from real_get_neighbors(world_arr, state, tool_multiplier)
        finally:
            pf.voxel_at = real_voxel_at

    pf.get_neighbors = patched_get_neighbors

    replans = []
    executed_path = [start + (blocks_available, 0)]
    executed_actions = []

    try:
        current = start
        blocks = blocks_available
        planned_path, cost, actions, expansions = weighted_astar(
            known, current, goal, blocks_available=blocks, epsilon=epsilon
        )
        replans.append({"at": current, "reason": "initial plan", "expansions": expansions})

        replan_count = 0
        step_i = 0
        while planned_path is not None and step_i < len(planned_path) - 1:
            next_state = planned_path[step_i + 1]
            nx, ny, nz, nblocks, ncrawling = next_state
            action = actions[step_i]

            # reveal terrain around the new position (simulating render distance)
            changed = reveal(known, true_world, (nx, ny, nz), reveal_radius)

            executed_path.append(next_state)
            executed_actions.append(action)
            current = (nx, ny, nz)
            blocks = nblocks
            step_i += 1

            if changed:
                # check whether the *remaining* planned path is still valid
                # under newly revealed info by re-deriving neighbor costs along it
                remaining = planned_path[step_i:]
                remaining_actions = actions[step_i:]
                invalid = False
                for k in range(len(remaining) - 1):
                    s0 = remaining[k]
                    s1 = remaining[k + 1]
                    valid_edges = {ns: (c, a) for ns, c, a in pf.get_neighbors(known, s0)}
                    if s1 not in valid_edges or valid_edges[s1][0] == float("inf"):
                        invalid = True
                        break

                if invalid and replan_count < max_replans:
                    replan_count += 1
                    new_path, new_cost, new_actions, expansions = weighted_astar(
                        known, current, goal, blocks_available=blocks, epsilon=epsilon
                    )
                    replans.append({
                        "at": current, "reason": "revealed terrain invalidated path",
                        "expansions": expansions
                    })
                    if new_path is None:
                        break
                    planned_path, actions = new_path, new_actions
                    step_i = 0

        return {
            "executed_path": executed_path,
            "executed_actions": executed_actions,
            "replans": replans,
            "reached_goal": current[:3] if 'current' in dir() else None,
            "final_pos": current,
            "goal": goal,
        }
    finally:
        pf.voxel_at = real_voxel_at
        pf.get_neighbors = real_get_neighbors
