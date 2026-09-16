package dev.netherpathfinder.engine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Weighted-A* search over the flat-voxel state space, faithful to
 * pathfind.py's weighted_astar + get_neighbors: exact control flow, cost
 * constants, and the diagonal corner-cutting rule. PARKOUR
 * (EdgeRules.parkourEdges) has no reverse probe here -- this is a plain
 * forward solver, so there's no backward half to keep in sync.
 *
 * Per-action edge legality/cost lives in EdgeRules, so a future bidirectional
 * search built on the same rules could never silently diverge from this
 * one. expand() calls relax() directly per candidate edge -- no intermediate
 * (state,cost,action) tuple object per candidate.
 *
 * STATE REPRESENTATION: every packed-long state is assigned a small dense
 * int id (via StateIdMap) the first time it's seen. gScore, cameFrom,
 * actionUsed, and closed are all plain arrays indexed by that id, and the
 * heap (IntIdOpenHeap) stores ids rather than packed longs. This avoids an
 * all-hash-map design (Long2Double/Long2Long/Long2Object maps keyed
 * directly by the packed long): profiling a hash-map-keyed heap that pushed
 * a new duplicate entry on every relax() rather than updating one in place
 * showed a large fraction of pops were already-stale (closed) entries, with
 * sifting dominating search wall-clock time. A decrease-key attempt that
 * kept the packed-long keying and added a hash map to track heap position
 * was net SLOWER (every heap-internal swap then paid two hash map put()s),
 * which is why this version routes everything through dense ids and plain
 * arrays instead -- array reads/writes during sifting cost the same
 * regardless of how often they happen, unlike hash map operations.
 */
public final class WeightedAStar {

    // --- per-search mutable state (one search() call at a time; not reentrant) ---
    private World world;
    private MineCostModel tools;
    private double epsilon;
    private GoalPoints goal;
    private boolean[] activeGoals;

    private StateIdMap ids;
    private double[] gScore;      // indexed by state id; Double.POSITIVE_INFINITY = unseen
    private int[] cameFromId;     // indexed by state id; -1 = no predecessor (start state)
    private Action[] actionUsed;  // indexed by state id
    private int[] bridgeUpOriginY; // indexed by state id -- see EdgeRules.bridgeUpEdge's javadoc
    private boolean[] closed;     // indexed by state id
    private IntIdOpenHeap heap;
    private AirPotential airPotential;

    // Published every 1024 expansions (not every one -- a volatile write per
    // expansion would add a memory barrier to the hottest loop in the
    // codebase for no real benefit) so a caller on another thread can poll
    // in-progress search progress, e.g. for a live HUD line. Only meaningful
    // while a search() call from this instance is actually running.
    private volatile int expansionsSoFar;

    public int expansionsSoFar() {
        return expansionsSoFar;
    }

    /**
     * Which MINE prune to use, if any -- kept as a simple static toggle so
     * the two independent pruning strategies (AirPotential's chunk-BFS
     * novelty signal, vs. this simpler Manhattan-distance-to-goal check)
     * can be A/B'd without changing search()'s public signature. NONE
     * restores the pre-AirPotential, pre-Manhattan baseline behavior.
     */
    public enum MinePruneMode { NONE, AIR_POTENTIAL, MANHATTAN }
    public static volatile MinePruneMode minePruneMode = EngineDefaults.minePruneMode();

    /** Results for every reachable member of one explicit multi-goal search. */
    public static final class MultiGoalResult {
        private final SearchResult[] results;
        private final int expansions;
        private final boolean budgetExhausted;

        private MultiGoalResult(SearchResult[] results, int expansions, boolean budgetExhausted) {
            this.results = results;
            this.expansions = expansions;
            this.budgetExhausted = budgetExhausted;
        }

        public SearchResult[] results() { return results; }
        public int expansions() { return expansions; }
        public boolean budgetExhausted() { return budgetExhausted; }

        public int foundCount() {
            int count = 0;
            for (SearchResult result : results) {
                if (result != null && result.found()) count++;
            }
            return count;
        }

        public boolean anyFound() { return foundCount() != 0; }

        public SearchResult firstFound() {
            for (SearchResult result : results) {
                if (result != null && result.found()) return result;
            }
            return null;
        }
    }

    /** Legacy scalar-multiplier entry point; wraps the old MINE_TIME table via MineCostModel.scaled. */
    public SearchResult search(World world, StateCodec.State start, StateCodec.State goal,
                                int blocksAvailable, double epsilon, double toolMultiplier,
                                int maxExpansions) {
        return search(world, start, GoalPoints.point(goal), blocksAvailable, epsilon,
            MineCostModel.scaled(toolMultiplier), maxExpansions);
    }

    /** Legacy scalar-multiplier entry point; wraps the old MINE_TIME table via MineCostModel.scaled. */
    public SearchResult search(World world, StateCodec.State start, GoalPoints goal,
                                int blocksAvailable, double epsilon, double toolMultiplier,
                                int maxExpansions) {
        return search(world, start, goal, blocksAvailable, epsilon,
            MineCostModel.scaled(toolMultiplier), maxExpansions);
    }

    public SearchResult search(World world, StateCodec.State start, GoalPoints goal,
                                int blocksAvailable, double epsilon, MineCostModel tools,
                                int maxExpansions) {
        beginSearch(world, start, goal, blocksAvailable, epsilon, tools, maxExpansions);

        int expansions = 0;
        while (!heap.isEmpty()) {
            if (Thread.currentThread().isInterrupted()) throw new java.util.concurrent.CancellationException();
            int stateId = heap.popId();
            if (closed[stateId]) {
                continue;
            }
            closed[stateId] = true;
            expansions++;
            if ((expansions & 0x3FF) == 0) expansionsSoFar = expansions;
            if (expansions > maxExpansions) {
                expansionsSoFar = expansions;
                return new SearchResult(false, null, 0.0, null, expansions);
            }

            long state = ids.packedStateOf(stateId);
            int x = StateCodec.unpackX(state);
            int y = StateCodec.unpackY(state);
            int z = StateCodec.unpackZ(state);

            if (goal.contains(x, y, z, StateCodec.unpackBlocks(state))) {
                expansionsSoFar = expansions;
                return reconstructPath(stateId, gScore[stateId], expansions);
            }

            expand(state, stateId, gScore[stateId]);
        }
        expansionsSoFar = expansions;
        return new SearchResult(false, null, 0.0, null, expansions);
    }

    /**
     * Continues the same weighted-A* traversal after reaching one target and
     * captures a complete route to every other reachable target. Results stay
     * aligned with the supplied GoalPoints indices; an unreachable target is null.
     */
    public MultiGoalResult searchAll(World world, StateCodec.State start, GoalPoints goal,
                                     int blocksAvailable, double epsilon, double toolMultiplier,
                                     int maxExpansions) {
        return searchAll(world, start, goal, blocksAvailable, epsilon,
            MineCostModel.scaled(toolMultiplier), maxExpansions);
    }

    public MultiGoalResult searchAll(World world, StateCodec.State start, GoalPoints goal,
                                     int blocksAvailable, double epsilon, MineCostModel tools,
                                     int maxExpansions) {
        beginSearch(world, start, goal, blocksAvailable, epsilon, tools, maxExpansions);
        activeGoals = new boolean[goal.size()];
        java.util.Arrays.fill(activeGoals, true);

        SearchResult[] results = new SearchResult[goal.size()];
        int found = 0;
        int expansions = 0;
        while (!heap.isEmpty()) {
            if (Thread.currentThread().isInterrupted()) throw new java.util.concurrent.CancellationException();
            int stateId = heap.popId();
            if (closed[stateId]) continue;
            closed[stateId] = true;
            expansions++;
            if ((expansions & 0x3FF) == 0) expansionsSoFar = expansions;
            if (expansions > maxExpansions) {
                expansionsSoFar = expansions;
                return new MultiGoalResult(results, expansions, true);
            }

            long state = ids.packedStateOf(stateId);
            int x = StateCodec.unpackX(state);
            int y = StateCodec.unpackY(state);
            int z = StateCodec.unpackZ(state);
            int blocks = StateCodec.unpackBlocks(state);
            boolean reachedNewGoal = false;
            for (int i = 0; i < goal.size(); i++) {
                if (results[i] == null && goal.matches(i, x, y, z, blocks)) {
                    results[i] = reconstructPath(stateId, gScore[stateId], expansions);
                    activeGoals[i] = false;
                    found++;
                    reachedNewGoal = true;
                }
            }
            if (found == goal.size()) {
                expansionsSoFar = expansions;
                return new MultiGoalResult(results, expansions, false);
            }

            expand(state, stateId, gScore[stateId]);
            if (reachedNewGoal) rebuildOpenHeapForActiveGoals();
        }
        expansionsSoFar = expansions;
        return new MultiGoalResult(results, expansions, false);
    }

    private void beginSearch(World world, StateCodec.State start, GoalPoints goal,
                             int blocksAvailable, double epsilon, MineCostModel tools,
                             int maxExpansions) {
        if (Thread.currentThread().isInterrupted()) throw new java.util.concurrent.CancellationException();
        this.world = world;
        this.tools = tools;
        this.epsilon = epsilon;
        this.goal = goal;
        this.activeGoals = null;
        this.expansionsSoFar = 0;

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
            // Built once, before any expansion sets a per-state context, so it reads
            // unrevealed terrain relative to the start height. The field is only a
            // pruning heuristic, so this approximation costs optimality at worst.
            world.unknownContext(start.y());
            AirPotentialField field = AirPotentialField.build(world, start.x(), start.y(), start.z());
            this.airPotential = new AirPotential(field, EngineDefaults.airPotentialScale());
        } else {
            this.airPotential = null;
        }

        long startState = StateCodec.pack(start.x(), start.y(), start.z(), blocksAvailable, false);
        int startId = internId(startState);
        gScore[startId] = 0.0;
        bridgeUpOriginY[startId] = start.y();
        double h0 = heuristic(start.x(), start.y(), start.z());
        heap.push(epsilon * h0, 0.0, start.y(), startId);
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

    private double heuristic(int x, int y, int z) {
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < goal.size(); i++) {
            if (activeGoals != null && !activeGoals[i]) continue;
            double dx = Math.abs(goal.x(i) - x);
            double dz = Math.abs(goal.z(i) - z);
            double horiz = (dx + dz) + (Math.sqrt(2) - 2) * Math.min(dx, dz);
            double vert = Math.abs(goal.y(i) - y);
            best = Math.min(best, (horiz + vert) / EdgeRules.SPRINT_SPEED);
        }
        return best;
    }

    /** Re-key the live frontier after one goal stops participating in the heuristic. */
    private void rebuildOpenHeapForActiveGoals() {
        int idCount = ids.idCount();
        IntIdOpenHeap rebuilt = new IntIdOpenHeap(
            Math.max(16, Math.min(idCount, 1_000_000)), Math.max(16, idCount));
        for (int id = 0; id < idCount; id++) {
            if (closed[id] || Double.isInfinite(gScore[id])) continue;
            long state = ids.packedStateOf(id);
            int x = StateCodec.unpackX(state);
            int y = StateCodec.unpackY(state);
            int z = StateCodec.unpackZ(state);
            rebuilt.push(gScore[id] + epsilon * heuristic(x, y, z), gScore[id], y, id);
        }
        heap = rebuilt;
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

        // replan.py's patched_get_neighbors: ground_y = this state's own y, so
        // unrevealed cells below it read as solid ground and cells at/above it
        // read as headroom, for the duration of this expansion.
        world.unknownContext(y);

        EdgeRules.EdgeConsumer consumer = (toState, cost, action) -> relax(state, stateId, gState, toState, cost, action);

        for (int d = 0; d < 8; d++) {
            EdgeRules.horizontalEdges(world, x, y, z, blocks, crawling, EdgeRules.DIR_DX[d], EdgeRules.DIR_DZ[d], tools,
                    airPotential, minePruneMode == MinePruneMode.MANHATTAN, goal, consumer);
        }
        for (int d = 0; d < 8; d++) {
            EdgeRules.climbEdge(world, x, y, z, blocks, EdgeRules.DIR_DX[d], EdgeRules.DIR_DZ[d], consumer);
        }
        EdgeRules.parkourEdges(world, x, y, z, blocks, consumer);
        EdgeRules.mineDownEdge(world, x, y, z, blocks, tools, consumer);
        EdgeRules.bridgeUpEdge(world, x, y, z, blocks, tools,
            bridgeUpOriginY[stateId], goal, consumer);
    }
}
