package dev.mcpathfind.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Direct port of pathfind.py's weighted_astar + get_neighbors, faithful to
 * the exact control flow, cost constants, and the diagonal corner-cutting
 * rule. PARKOUR (EdgeRules.parkourEdges) is not reverse-probed by
 * BidirectionalWeightedAStar/ParallelBidirectionalWeightedAStar -- a
 * PARKOUR-only route still gets found via forward search's own guaranteed
 * direct-hit fallback, just without a bidirectional speedup, the same
 * accepted incompleteness already true of BRIDGE_UP's backward chain
 * discovery (see reverseBridgeUpSource's javadoc).
 *
 * Per-action edge legality/cost lives in EdgeRules (shared with
 * BidirectionalWeightedAStar's backward search, so the two can never
 * silently diverge). expand() calls relax() directly per candidate edge --
 * no intermediate (state,cost,action) tuple object per candidate, matching
 * the whole point of this port (Python's generator allocates one such tuple
 * per yield).
 *
 * STATE REPRESENTATION: every packed-long state is assigned a small dense
 * int id (via StateIdMap) the first time it's seen. gScore, cameFrom,
 * actionUsed, and closed are all plain arrays indexed by that id, and the
 * heap (IntIdOpenHeap) stores ids rather than packed longs. This replaced
 * an earlier all-hash-map version (Long2Double/Long2Long/Long2Object maps
 * keyed directly by the packed long) after profiling on a real search
 * region (k2_r_0_0, 368x128x368) showed the old heap -- pushing a new
 * duplicate entry on every relax() rather than updating one in place --
 * had a 2.59x push-to-expansion ratio, ~31% stale (already-closed) pops,
 * and siftUp+siftDown alone were ~70% of total search wall-clock time.
 * A first attempt at decrease-key kept the packed-long keying and added a
 * Long2IntOpenHashMap to track heap position; that was net SLOWER (every
 * heap-internal swap now paid two hashmap put()s), which is why this
 * version routes everything through dense ids and plain arrays instead --
 * array reads/writes during sifting cost the same regardless of how often
 * they happen, unlike hash map operations.
 */
public final class WeightedAStar {

    // --- per-search mutable state (one search() call at a time; not reentrant) ---
    private World world;
    private double toolMultiplier;
    private double epsilon;
    private GoalPoints goal;

    private StateIdMap ids;
    private double[] gScore;      // indexed by state id; Double.POSITIVE_INFINITY = unseen
    private int[] cameFromId;     // indexed by state id; -1 = no predecessor (start state)
    private Action[] actionUsed;  // indexed by state id
    private int[] bridgeUpOriginY; // indexed by state id -- see EdgeRules.bridgeUpEdge's javadoc
    private boolean[] closed;     // indexed by state id
    private IntIdOpenHeap heap;
    private AirPotential airPotential;

    /**
     * Which MINE prune to use, if any -- kept as a simple static toggle so
     * the two independent pruning strategies (AirPotential's chunk-BFS
     * novelty signal, vs. this simpler Manhattan-distance-to-goal check)
     * can be A/B'd without changing search()'s public signature. NONE
     * restores the pre-AirPotential, pre-Manhattan baseline behavior.
     */
    public enum MinePruneMode { NONE, AIR_POTENTIAL, MANHATTAN }
    public static MinePruneMode minePruneMode = MinePruneMode.AIR_POTENTIAL;

    /** Single-point goal, preserved for source compatibility -- delegates to the GoalPoints overload as a 1-element set. */
    public SearchResult search(World world, StateCodec.State start, StateCodec.State goal,
                                int blocksAvailable, double epsilon, double toolMultiplier,
                                int maxExpansions) {
        return search(world, start, GoalPoints.point(goal), blocksAvailable, epsilon, toolMultiplier, maxExpansions);
    }

    public SearchResult search(World world, StateCodec.State start, GoalPoints goal,
                                int blocksAvailable, double epsilon, double toolMultiplier,
                                int maxExpansions) {
        this.world = world;
        this.toolMultiplier = toolMultiplier;
        this.epsilon = epsilon;
        this.goal = goal;

        // Pre-size against the expansion budget, same rationale as before:
        // avoid repeated grow()/rehash on a multi-million-expansion search.
        // Capped so a huge budget on an easy (fast-converging) search doesn't
        // over-allocate for nothing.
        int sizeHint = (int) Math.min(Math.max(1024L, (long) maxExpansions * 4), 20_000_000L);

        this.ids = new StateIdMap(sizeHint);
        this.gScore = newDoubleArray(sizeHint, Double.POSITIVE_INFINITY);
        this.cameFromId = newIntArray(sizeHint, -1);
        this.actionUsed = new Action[sizeHint];
        this.bridgeUpOriginY = new int[sizeHint];
        this.closed = new boolean[sizeHint];
        this.heap = new IntIdOpenHeap(Math.min(sizeHint, 1_000_000), sizeHint);

        if (minePruneMode == MinePruneMode.AIR_POTENTIAL) {
            AirPotentialField field = AirPotentialField.build(world, start.x(), start.y(), start.z());
            this.airPotential = new AirPotential(field, /* scaleBfs */ 3.0);
        } else {
            this.airPotential = null;
        }

        long startState = StateCodec.pack(start.x(), start.y(), start.z(), blocksAvailable, false);
        int startId = internId(startState);
        gScore[startId] = 0.0;
        bridgeUpOriginY[startId] = start.y();
        double h0 = heuristic(start.x(), start.y(), start.z());
        heap.push(epsilon * h0, 0.0, start.y(), startId);

        int expansions = 0;
        while (!heap.isEmpty()) {
            int stateId = heap.popId();
            if (closed[stateId]) {
                continue;
            }
            closed[stateId] = true;
            expansions++;
            if (expansions > maxExpansions) {
                return new SearchResult(false, null, 0.0, null, expansions);
            }

            long state = ids.packedStateOf(stateId);
            int x = StateCodec.unpackX(state);
            int y = StateCodec.unpackY(state);
            int z = StateCodec.unpackZ(state);

            if (goal.contains(x, y, z)) {
                return reconstructPath(stateId, gScore[stateId], expansions);
            }

            expand(state, stateId, gScore[stateId]);
        }
        return new SearchResult(false, null, 0.0, null, expansions);
    }

    /** Looks up (or allocates) this state's dense id, and keeps every id-indexed array/heap big enough to hold it. */
    private int internId(long packedState) {
        int id = ids.idFor(packedState);
        ensureCapacityFor(id);
        return id;
    }

    private void ensureCapacityFor(int id) {
        if (id >= gScore.length) {
            int newCap = Math.max(id + 1, gScore.length + (gScore.length >> 1) + 1);
            gScore = java.util.Arrays.copyOf(gScore, newCap);
            java.util.Arrays.fill(gScore, id, newCap, Double.POSITIVE_INFINITY);
            cameFromId = java.util.Arrays.copyOf(cameFromId, newCap);
            java.util.Arrays.fill(cameFromId, id, newCap, -1);
            actionUsed = java.util.Arrays.copyOf(actionUsed, newCap);
            bridgeUpOriginY = java.util.Arrays.copyOf(bridgeUpOriginY, newCap);
            closed = java.util.Arrays.copyOf(closed, newCap);
        }
        heap.ensureIdCapacity(id + 1);
    }

    private static double[] newDoubleArray(int size, double fillValue) {
        double[] a = new double[size];
        java.util.Arrays.fill(a, fillValue);
        return a;
    }

    private static int[] newIntArray(int size, int fillValue) {
        int[] a = new int[size];
        java.util.Arrays.fill(a, fillValue);
        return a;
    }

    private SearchResult reconstructPath(int goalId, double totalCost, int expansions) {
        List<Integer> idsRev = new ArrayList<>();
        List<Action> actionsRev = new ArrayList<>();
        int s = goalId;
        idsRev.add(s);
        while (cameFromId[s] != -1) {
            actionsRev.add(actionUsed[s]);
            s = cameFromId[s];
            idsRev.add(s);
        }
        Collections.reverse(idsRev);
        Collections.reverse(actionsRev);

        StateCodec.State[] path = new StateCodec.State[idsRev.size()];
        for (int i = 0; i < idsRev.size(); i++) {
            path[i] = StateCodec.toRecord(ids.packedStateOf(idsRev.get(i)));
        }
        return new SearchResult(true, path, totalCost, actionsRev.toArray(new Action[0]), expansions);
    }

    private static final double SQRT2_MINUS_2 = EdgeRules.SQRT2 - 2.0;

    // Distance to the NEAREST goal point -- min of the admissible per-point
    // heuristic is itself admissible for "cost to reach any goal" (see
    // GoalPoints' class javadoc). goal.size() is small (a handful of
    // points), so this loop is cheap.
    private double heuristic(int x, int y, int z) {
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < goal.size(); i++) {
            double dx = Math.abs(goal.x(i) - x);
            double dz = Math.abs(goal.z(i) - z);
            double horiz = (dx + dz) + SQRT2_MINUS_2 * Math.min(dx, dz);
            double vert = Math.abs(goal.y(i) - y);
            double h = (horiz + vert) / EdgeRules.SPRINT_SPEED;
            if (h < best) {
                best = h;
            }
        }
        return best;
    }

    private void relax(long fromState, int fromId, double gFrom, long toState, double cost, Action action) {
        int toId = internId(toState);
        if (closed[toId]) {
            return;
        }
        double tentativeG = gFrom + cost;
        if (tentativeG < gScore[toId]) {
            gScore[toId] = tentativeG;
            cameFromId[toId] = fromId;
            actionUsed[toId] = action;
            int nx = StateCodec.unpackX(toState);
            int ny = StateCodec.unpackY(toState);
            int nz = StateCodec.unpackZ(toState);
            // Inherit the chain's true start if this edge continues an
            // already-committed BRIDGE_UP chain; otherwise this state is a
            // fresh potential chain start -- see EdgeRules.bridgeUpEdge's
            // javadoc for why using the current y instead would erode the
            // vertical margin mid-climb.
            bridgeUpOriginY[toId] = (action == Action.BRIDGE_UP) ? bridgeUpOriginY[fromId] : ny;
            double h = heuristic(nx, ny, nz);
            heap.push(tentativeG + epsilon * h, tentativeG, ny, toId);
        }
    }

    private void expand(long state, int stateId, double gState) {
        int x = StateCodec.unpackX(state);
        int y = StateCodec.unpackY(state);
        int z = StateCodec.unpackZ(state);
        int blocks = StateCodec.unpackBlocks(state);
        boolean crawling = StateCodec.unpackCrawling(state);

        EdgeRules.EdgeConsumer consumer = (toState, cost, action) -> relax(state, stateId, gState, toState, cost, action);

        for (int d = 0; d < 8; d++) {
            EdgeRules.horizontalEdges(world, x, y, z, blocks, crawling, EdgeRules.DIR_DX[d], EdgeRules.DIR_DZ[d], toolMultiplier,
                    airPotential, minePruneMode == MinePruneMode.MANHATTAN, goal, consumer);
        }
        for (int d = 0; d < 8; d++) {
            EdgeRules.climbEdge(world, x, y, z, blocks, EdgeRules.DIR_DX[d], EdgeRules.DIR_DZ[d], consumer);
        }
        EdgeRules.parkourEdges(world, x, y, z, blocks, consumer);
        EdgeRules.mineDownEdge(world, x, y, z, blocks, toolMultiplier, consumer);
        EdgeRules.bridgeUpEdge(world, x, y, z, blocks, toolMultiplier, bridgeUpOriginY[stateId], goal, consumer);
    }
}
