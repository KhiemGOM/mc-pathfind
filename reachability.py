"""
Quick reachability utilities for real converted MC terrain.

Real terrain (unlike hand-built synthetic worlds) can have start/goal pairs
that are separated by huge mining/bridging requirements, or are in genuinely
disconnected pockets. Before running the expensive weighted A* search, it's
useful to sanity-check with a cheap unweighted BFS over "walk only" moves
(no mining, no bridging) to understand how large the easily-reachable region
is, and to pick sensible test goals.
"""

from collections import deque
from world import is_solid


def walkable_flood_fill(world, start, max_states=500_000):
    """
    BFS over states reachable by walking/stepping only (no mine, no bridge,
    no risky falls beyond 1 block). Returns the set of visited (x,y,z) states.
    Cheap sanity check, not a substitute for the real cost-aware search.

    Must also check head clearance (y+1 not solid): pathfind.py's SPRINT/
    CLIMB/FALL preconditions all require this (a low overhang blocks the
    2-tall body even when the foot cell itself is open air), and this BFS
    is used to hand-pick start/goal pairs for the real solver -- if it
    claims a pair is walk-reachable via a looser rule than the solver
    actually enforces, route selection can pick a goal that isn't reachable
    at all, and the costed search burns its whole budget never finding it.
    """
    sx, sy, sz = world.shape
    visited = set()
    q = deque([start])
    visited.add(start)
    while q and len(visited) < max_states:
        x, y, z = q.popleft()
        for dx, dz in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            for dy in (0, 1, -1):
                nx, ny, nz = x + dx, y + dy, z + dz
                if not (0 <= nx < sx and 0 <= ny < sy and 0 <= nz < sz):
                    continue
                if (nx, ny, nz) in visited:
                    continue
                if world[nx, ny, nz] != 0:  # must be air to stand in
                    continue
                head = world[nx, ny + 1, nz] if ny + 1 < sy else 0
                if is_solid(head):  # overhang blocks the 2-tall body
                    continue
                below = world[nx, ny - 1, nz] if ny > 0 else 0
                if ny == 0 or is_solid(below):
                    visited.add((nx, ny, nz))
                    q.append((nx, ny, nz))
    return visited


def pick_far_goal(world, start, max_states=500_000):
    """Convenience helper: flood-fill from start, return the walk-reachable
    state with the largest Manhattan (x,z) distance from start, plus the
    full visited set for further inspection."""
    visited = walkable_flood_fill(world, start, max_states=max_states)
    if not visited:
        return None, visited
    far = max(visited, key=lambda p: abs(p[0] - start[0]) + abs(p[2] - start[2]))
    return far, visited


def _bfs_farthest_by_hops(world, start, max_states=500_000):
    """
    BFS layer-by-layer from start, returning (farthest_node, visited_set)
    where farthest_node is at the maximum BFS depth actually reached (true
    hop-count distance, not Manhattan distance -- a winding cave can put two
    cells far apart in hops while close in Manhattan distance, or vice versa
    across a chasm, so Manhattan distance is a poor proxy for "how long a
    walked route between these two points actually is").
    """
    sx, sy, sz = world.shape
    visited = {start}
    frontier = [start]
    farthest = start
    while frontier and len(visited) < max_states:
        next_frontier = []
        for (x, y, z) in frontier:
            for dx, dz in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                for dy in (0, 1, -1):
                    nx, ny, nz = x + dx, y + dy, z + dz
                    if not (0 <= nx < sx and 0 <= ny < sy and 0 <= nz < sz):
                        continue
                    if (nx, ny, nz) in visited:
                        continue
                    if world[nx, ny, nz] != 0:
                        continue
                    head = world[nx, ny + 1, nz] if ny + 1 < sy else 0
                    if is_solid(head):  # overhang blocks the 2-tall body, see walkable_flood_fill
                        continue
                    below = world[nx, ny - 1, nz] if ny > 0 else 0
                    if ny == 0 or is_solid(below):
                        visited.add((nx, ny, nz))
                        next_frontier.append((nx, ny, nz))
        if next_frontier:
            farthest = next_frontier[-1]
        frontier = next_frontier
    return farthest, visited


def diameter_endpoints(world, seed, max_states=500_000):
    """
    Double-sweep BFS: from an arbitrary seed, find the farthest reachable
    node A by hop count; then BFS again from A to find the farthest node B
    from A. (A, B) approximates the two most mutually-distant points in the
    connected component -- exact on trees, a standard and good heuristic on
    general graphs, and far better than "far from an arbitrary start" for
    picking a route that actually spans the walkable area.
    """
    a, _ = _bfs_farthest_by_hops(world, seed, max_states=max_states)
    b, visited_from_a = _bfs_farthest_by_hops(world, a, max_states=max_states)
    return a, b, visited_from_a


def largest_walkable_component(world, seeds, max_states_per_component=300_000):
    """
    Given candidate standing-spot seeds scattered across the terrain, flood-
    fill from each not-yet-seen seed and keep the largest resulting
    component. Cost is bounded by total cells visited across all components
    combined (each cell is only ever flood-filled once, via the seen-set
    dedup), not by the number of candidate seeds -- most seeds land in a
    component that's already been measured and are skipped in O(1).

    Returns (seed_used_for_best_component, best_component_set).
    """
    seen_global = set()
    best_seed, best_component = None, set()
    for seed in seeds:
        if seed in seen_global:
            continue
        component = walkable_flood_fill(world, seed, max_states=max_states_per_component)
        seen_global |= component
        if len(component) > len(best_component):
            best_seed, best_component = seed, component
    return best_seed, best_component
