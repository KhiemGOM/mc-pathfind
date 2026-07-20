package dev.mcpathfind.pearl;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Same algorithm as BidirectionalWeightedAStar, but forward and backward
 * search each run on their own thread for a batch of steps at a time --
 * forked from dev.mcpathfind.core's version for the trimmed action set. See
 * dev.mcpathfind.pearl.BidirectionalWeightedAStar's class javadoc for the
 * simplifications relative to core's BRIDGE-oriented version (no funding
 * estimate/cost-correction machinery, posKey is the identity projection);
 * everything said there applies here too.
 *
 * The reason batching is safe WITHOUT locks or concurrent collections:
 * forward and backward already use fully separate data structures (fwd* vs
 * bwd*). During a batch, each worker thread reads and writes ONLY its own
 * side's structures -- it never touches the other side's maps/heap, and
 * never touches the position-projection structures (fwdPosBestG/
 * fwdPosBestStateId) at all. All cross-side interaction (checking for a
 * meeting, updating those projection structures) happens on the MAIN
 * thread, strictly AFTER both batches have completed via Future.get() --
 * which the Java Memory Model guarantees establishes a happens-before
 * relationship. At no point are two threads ever reading and writing the
 * same structure concurrently.
 */
public final class ParallelBidirectionalWeightedAStar {

    private static final int BATCH_SIZE = 2000;

    private World world;
    private double toolMultiplier;
    private double epsilon;
    private GoalPoints goal;

    private StateIdMap fwdIds;
    private double[] fwdG;
    private int[] fwdCameFromId;
    private Action[] fwdActionUsed;
    private boolean[] fwdClosed;
    private IntIdOpenHeap fwdHeap;

    private StateIdMap bwdIds;
    private double[] bwdG;
    private int[] bwdCameFromId;
    private Action[] bwdActionUsed;
    private boolean[] bwdClosed;
    private IntIdOpenHeap bwdHeap;

    // projection of forward-closed states onto (x,y,z,crawling) -- the
    // IDENTITY projection now (see BidirectionalWeightedAStar's class
    // javadoc), indexed by BWD id, written only during the main-thread
    // merge step.
    private double[] fwdPosBestG;
    private int[] fwdPosBestStateId; // -1 = none yet; else a FORWARD id

    private double mu;
    private int meetForwardId = -1;
    private int meetBackwardId = -1;

    // See BidirectionalWeightedAStar's class javadoc ("MEET-IN-THE-MIDDLE
    // TERMINATION"). Batching (BATCH_SIZE per round) already gives this
    // class some natural "grace" per round; this adds an explicit floor on
    // top so a single lucky round right after mu first improves can't
    // immediately end the search.
    public static int MU_GRACE_EXPANSIONS = 2000;
    private int expansionsSinceMuImproved = 0;

    private int startX, startY, startZ;
    private int startId;

    private AirPotential airPotential;

    private record BatchResult(List<Integer> newlyClosedIds, boolean directGoalHit, int directGoalId, int stepsProcessed) {}

    /** Single-point goal, preserved for source compatibility -- delegates to the GoalPoints overload as a 1-element set. */
    public SearchResult search(World world, StateCodec.State start, StateCodec.State goal,
                                double epsilon, double toolMultiplier, int maxExpansions) {
        return search(world, start, GoalPoints.point(goal), epsilon, toolMultiplier, maxExpansions);
    }

    public SearchResult search(World world, StateCodec.State start, GoalPoints goal,
                                double epsilon, double toolMultiplier, int maxExpansions) {
        this.world = world;
        this.toolMultiplier = toolMultiplier;
        this.epsilon = epsilon;
        this.goal = goal;

        int sizeHint = (int) Math.min(Math.max(1024L, (long) maxExpansions * 4), 20_000_000L);

        fwdIds = new StateIdMap(sizeHint);
        fwdG = newDoubleArray(sizeHint, Double.POSITIVE_INFINITY);
        fwdCameFromId = newIntArray(sizeHint, -1);
        fwdActionUsed = new Action[sizeHint];
        fwdClosed = new boolean[sizeHint];
        fwdHeap = new IntIdOpenHeap(Math.min(sizeHint, 1_000_000), sizeHint);

        bwdIds = new StateIdMap(sizeHint);
        bwdG = newDoubleArray(sizeHint, Double.POSITIVE_INFINITY);
        bwdCameFromId = newIntArray(sizeHint, -1);
        bwdActionUsed = new Action[sizeHint];
        bwdClosed = new boolean[sizeHint];
        bwdHeap = new IntIdOpenHeap(Math.min(sizeHint, 1_000_000), sizeHint);

        fwdPosBestG = newDoubleArray(sizeHint, Double.POSITIVE_INFINITY);
        fwdPosBestStateId = newIntArray(sizeHint, -1);

        mu = Double.POSITIVE_INFINITY;
        meetForwardId = -1;
        meetBackwardId = -1;
        expansionsSinceMuImproved = 0;

        if (WeightedAStar.minePruneMode == WeightedAStar.MinePruneMode.AIR_POTENTIAL) {
            AirPotentialField field = AirPotentialField.build(world, start.x(), start.y(), start.z());
            this.airPotential = new AirPotential(field, /* scaleBfs */ 3.0);
        } else {
            this.airPotential = null;
        }

        this.startX = start.x();
        this.startY = start.y();
        this.startZ = start.z();

        long startState = StateCodec.pack(start.x(), start.y(), start.z(), false);
        int startId = internFwd(startState);
        this.startId = startId;
        fwdG[startId] = 0.0;
        fwdHeap.push(epsilon * heuristicToGoal(start.x(), start.y(), start.z()), 0.0, start.y(), startId);

        // Seed backward search from every goal point -- see
        // BidirectionalWeightedAStar's identical seeding block. Runs
        // single-threaded before the two worker threads start.
        for (int i = 0; i < goal.size(); i++) {
            int gx = goal.x(i), gy = goal.y(i), gz = goal.z(i);
            if (!EdgeRules.isValidRestingSource(world, gx, gy, gz, false)) {
                continue;
            }
            long goalSeed = StateCodec.pack(gx, gy, gz, false);
            int goalSeedId = internBwd(goalSeed);
            bwdG[goalSeedId] = 0.0;
            bwdHeap.push(epsilon * heuristicToStart(gx, gy, gz, start), 0.0, gy, goalSeedId);
        }

        ExecutorService executor = Executors.newFixedThreadPool(2);
        try {
            int expansions = 0;
            while (true) {
                if (fwdHeap.isEmpty() && bwdHeap.isEmpty()) {
                    return new SearchResult(false, null, 0.0, null, expansions);
                }

                Future<BatchResult> fwdFuture = fwdHeap.isEmpty() ? null : executor.submit(() -> forwardBatch(BATCH_SIZE));
                Future<BatchResult> bwdFuture = bwdHeap.isEmpty() ? null : executor.submit(() -> backwardBatch(BATCH_SIZE));

                BatchResult fwdResult = null, bwdResult = null;
                try {
                    if (fwdFuture != null) fwdResult = fwdFuture.get();
                    if (bwdFuture != null) bwdResult = bwdFuture.get();
                } catch (Exception e) {
                    throw new RuntimeException("parallel bidirectional search worker failed", e);
                }

                if (fwdResult != null) {
                    expansions += fwdResult.stepsProcessed();
                    expansionsSinceMuImproved += fwdResult.stepsProcessed();
                    if (fwdResult.directGoalHit()) {
                        int goalId = fwdResult.directGoalId();
                        return reconstructFromForwardId(goalId, fwdG[goalId], expansions);
                    }
                    mergeForwardClosed(fwdResult.newlyClosedIds());
                }
                if (bwdResult != null) {
                    expansions += bwdResult.stepsProcessed();
                    expansionsSinceMuImproved += bwdResult.stepsProcessed();
                    mergeBackwardClosed(bwdResult.newlyClosedIds());
                }

                if (expansions > maxExpansions) {
                    return new SearchResult(false, null, 0.0, null, expansions);
                }

                if (mu < Double.POSITIVE_INFINITY && !fwdHeap.isEmpty() && !bwdHeap.isEmpty()) {
                    if (fwdHeap.peekPriority() + bwdHeap.peekPriority() >= mu
                            && expansionsSinceMuImproved >= MU_GRACE_EXPANSIONS) {
                        return reconstructMeeting(expansions);
                    }
                }
            }
        } finally {
            executor.shutdown();
        }
    }

    private int internFwd(long packedState) {
        int id = fwdIds.idFor(packedState);
        ensureFwdCapacityFor(id);
        return id;
    }

    private int internBwd(long packedState) {
        int id = bwdIds.idFor(packedState);
        ensureBwdCapacityFor(id);
        return id;
    }

    private void ensureFwdCapacityFor(int id) {
        if (id >= fwdG.length) {
            int newCap = Math.max(id + 1, fwdG.length + (fwdG.length >> 1) + 1);
            fwdG = java.util.Arrays.copyOf(fwdG, newCap);
            java.util.Arrays.fill(fwdG, id, newCap, Double.POSITIVE_INFINITY);
            fwdCameFromId = java.util.Arrays.copyOf(fwdCameFromId, newCap);
            java.util.Arrays.fill(fwdCameFromId, id, newCap, -1);
            fwdActionUsed = java.util.Arrays.copyOf(fwdActionUsed, newCap);
            fwdClosed = java.util.Arrays.copyOf(fwdClosed, newCap);
        }
        fwdHeap.ensureIdCapacity(id + 1);
    }

    private void ensureBwdCapacityFor(int id) {
        if (id >= bwdG.length) {
            int newCap = Math.max(id + 1, bwdG.length + (bwdG.length >> 1) + 1);
            bwdG = java.util.Arrays.copyOf(bwdG, newCap);
            java.util.Arrays.fill(bwdG, id, newCap, Double.POSITIVE_INFINITY);
            bwdCameFromId = java.util.Arrays.copyOf(bwdCameFromId, newCap);
            java.util.Arrays.fill(bwdCameFromId, id, newCap, -1);
            bwdActionUsed = java.util.Arrays.copyOf(bwdActionUsed, newCap);
            bwdClosed = java.util.Arrays.copyOf(bwdClosed, newCap);
            fwdPosBestG = java.util.Arrays.copyOf(fwdPosBestG, newCap);
            java.util.Arrays.fill(fwdPosBestG, id, newCap, Double.POSITIVE_INFINITY);
            fwdPosBestStateId = java.util.Arrays.copyOf(fwdPosBestStateId, newCap);
            java.util.Arrays.fill(fwdPosBestStateId, id, newCap, -1);
        }
        bwdHeap.ensureIdCapacity(id + 1);
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

    /** Runs on a worker thread. Touches ONLY fwd-side structures -- no reads of bwd/position-projection state here. */
    private BatchResult forwardBatch(int maxSteps) {
        List<Integer> newlyClosed = new ArrayList<>(maxSteps);
        int steps = 0;
        while (steps < maxSteps && !fwdHeap.isEmpty()) {
            int stateId = fwdHeap.popId();
            if (fwdClosed[stateId]) {
                continue;
            }
            fwdClosed[stateId] = true;
            steps++;

            long state = fwdIds.packedStateOf(stateId);
            int x = StateCodec.unpackX(state);
            int y = StateCodec.unpackY(state);
            int z = StateCodec.unpackZ(state);
            boolean crawling = StateCodec.unpackCrawling(state);

            if (goal.contains(x, y, z)) {
                return new BatchResult(newlyClosed, true, stateId, steps);
            }
            newlyClosed.add(stateId);

            double g = fwdG[stateId];
            EdgeRules.EdgeConsumer consumer = (toState, cost, action) -> relaxForward(stateId, g, toState, cost, action);
            for (int d = 0; d < 8; d++) {
                EdgeRules.horizontalEdges(world, x, y, z, crawling, EdgeRules.DIR_DX[d], EdgeRules.DIR_DZ[d], toolMultiplier,
                        airPotential, WeightedAStar.minePruneMode == WeightedAStar.MinePruneMode.MANHATTAN, goal, consumer);
            }
            for (int d = 0; d < 8; d++) {
                EdgeRules.climbEdge(world, x, y, z, EdgeRules.DIR_DX[d], EdgeRules.DIR_DZ[d], consumer);
            }
            EdgeRules.mineDownEdge(world, x, y, z, toolMultiplier, consumer);
            EdgeRules.pearlEdges(world, x, y, z, consumer);
        }
        return new BatchResult(newlyClosed, false, -1, steps);
    }

    /** Runs on a worker thread. Touches ONLY bwd-side structures -- no reads of fwd/position-projection state here. */
    private BatchResult backwardBatch(int maxSteps) {
        List<Integer> newlyClosed = new ArrayList<>(maxSteps);
        int steps = 0;
        while (steps < maxSteps && !bwdHeap.isEmpty()) {
            int stateId = bwdHeap.popId();
            if (bwdClosed[stateId]) {
                continue;
            }
            bwdClosed[stateId] = true;
            steps++;
            newlyClosed.add(stateId);

            long state = bwdIds.packedStateOf(stateId);
            int x = StateCodec.unpackX(state);
            int y = StateCodec.unpackY(state);
            int z = StateCodec.unpackZ(state);
            boolean crawling = StateCodec.unpackCrawling(state);
            double g = bwdG[stateId];

            EdgeRules.EdgeConsumer consumer = (predecessor, cost, action) -> relaxBackward(stateId, g, predecessor, cost, action);
            if (crawling) {
                EdgeRules.reverseCrawlEdges(world, x, y, z, toolMultiplier, consumer);
            } else {
                EdgeRules.reverseNonCrawlEdges(world, x, y, z, toolMultiplier, airPotential, consumer);
            }
        }
        return new BatchResult(newlyClosed, false, -1, steps);
    }

    /** Runs on the main thread ONLY, strictly after both batches (if any) have completed. */
    private void mergeForwardClosed(List<Integer> newlyClosedFwdIds) {
        for (int fwdId : newlyClosedFwdIds) {
            long state = fwdIds.packedStateOf(fwdId);
            double g = fwdG[fwdId];
            // posKey is the identity projection now (see
            // BidirectionalWeightedAStar's class javadoc) -- the forward
            // state's own packed long IS already backward-shaped.
            int posId = internBwd(state);
            if (g < fwdPosBestG[posId]) {
                fwdPosBestG[posId] = g;
                fwdPosBestStateId[posId] = fwdId;
            }
            if (bwdClosed[posId]) {
                double candidate = g + bwdG[posId];
                if (candidate < mu) {
                    mu = candidate;
                    meetForwardId = fwdId;
                    meetBackwardId = posId;
                    expansionsSinceMuImproved = 0;
                }
            }
        }
    }

    /** Runs on the main thread ONLY, strictly after both batches (if any) have completed. */
    private void mergeBackwardClosed(List<Integer> newlyClosedBwdIds) {
        for (int bwdId : newlyClosedBwdIds) {
            double g = bwdG[bwdId];
            if (fwdPosBestStateId[bwdId] != -1) {
                double candidate = g + fwdPosBestG[bwdId];
                if (candidate < mu) {
                    mu = candidate;
                    meetForwardId = fwdPosBestStateId[bwdId];
                    meetBackwardId = bwdId;
                    expansionsSinceMuImproved = 0;
                }
            }

            // Backward-direct-hit: see BidirectionalWeightedAStar's
            // stepBackward() -- identical logic, just running as part of
            // the main-thread merge step instead of inline per-pop. No
            // funding check needed here (unlike core's version): nothing
            // left costs a consumable resource.
            long state = bwdIds.packedStateOf(bwdId);
            int x = StateCodec.unpackX(state);
            int y = StateCodec.unpackY(state);
            int z = StateCodec.unpackZ(state);
            boolean crawling = StateCodec.unpackCrawling(state);
            if (!crawling && x == startX && y == startY && z == startZ) {
                double candidate = g; // fwdG[startId] is always exactly 0.0
                if (candidate < mu) {
                    mu = candidate;
                    meetForwardId = startId;
                    meetBackwardId = bwdId;
                    expansionsSinceMuImproved = 0;
                }
            }
        }
    }

    private void relaxForward(int fromId, double gFrom, long toState, double cost, Action action) {
        int toId = internFwd(toState);
        if (fwdClosed[toId]) {
            return;
        }
        double tentativeG = gFrom + cost;
        if (tentativeG < fwdG[toId]) {
            fwdG[toId] = tentativeG;
            fwdCameFromId[toId] = fromId;
            fwdActionUsed[toId] = action;
            int toY = StateCodec.unpackY(toState);
            double h = heuristicToGoal(StateCodec.unpackX(toState), toY, StateCodec.unpackZ(toState));
            fwdHeap.push(tentativeG + epsilon * h, tentativeG, toY, toId);
        }
    }

    private void relaxBackward(int fromId, double gFrom, long toState, double cost, Action action) {
        int toId = internBwd(toState);
        if (bwdClosed[toId]) {
            return;
        }
        double tentativeG = gFrom + cost;
        if (tentativeG < bwdG[toId]) {
            bwdG[toId] = tentativeG;
            bwdCameFromId[toId] = fromId;
            bwdActionUsed[toId] = action;
            int toY = StateCodec.unpackY(toState);
            double h = heuristicToStartFromPacked(toState);
            bwdHeap.push(tentativeG + epsilon * h, tentativeG, toY, toId);
        }
    }

    private static final double SQRT2_MINUS_2 = EdgeRules.SQRT2 - 2.0;

    private double heuristicToGoal(int x, int y, int z) {
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

    private int startXForHeuristic, startYForHeuristic, startZForHeuristic;

    private double heuristicToStart(int x, int y, int z, StateCodec.State start) {
        startXForHeuristic = start.x();
        startYForHeuristic = start.y();
        startZForHeuristic = start.z();
        return heuristicToStartFromCoords(x, y, z);
    }

    private double heuristicToStartFromPacked(long state) {
        return heuristicToStartFromCoords(StateCodec.unpackX(state), StateCodec.unpackY(state), StateCodec.unpackZ(state));
    }

    private double heuristicToStartFromCoords(int x, int y, int z) {
        double dx = Math.abs(startXForHeuristic - x);
        double dz = Math.abs(startZForHeuristic - z);
        double horiz = (dx + dz) + SQRT2_MINUS_2 * Math.min(dx, dz);
        double vert = Math.abs(startYForHeuristic - y);
        return (horiz + vert) / EdgeRules.SPRINT_SPEED;
    }

    private SearchResult reconstructFromForwardId(int goalId, double totalCost, int expansions) {
        List<Integer> idsRev = new ArrayList<>();
        List<Action> actionsRev = new ArrayList<>();
        int s = goalId;
        idsRev.add(s);
        while (fwdCameFromId[s] != -1) {
            actionsRev.add(fwdActionUsed[s]);
            s = fwdCameFromId[s];
            idsRev.add(s);
        }
        Collections.reverse(idsRev);
        Collections.reverse(actionsRev);

        StateCodec.State[] path = new StateCodec.State[idsRev.size()];
        for (int i = 0; i < idsRev.size(); i++) {
            path[i] = StateCodec.toRecord(fwdIds.packedStateOf(idsRev.get(i)));
        }
        return new SearchResult(true, path, totalCost, actionsRev.toArray(new Action[0]), expansions);
    }

    /**
     * See BidirectionalWeightedAStar.reconstructMeeting -- identical logic
     * (a plain stitch, no per-step cost correction needed, see class
     * javadoc).
     */
    private SearchResult reconstructMeeting(int expansions) {
        List<Integer> fwdIdsRev = new ArrayList<>();
        List<Action> fwdActionsRev = new ArrayList<>();
        int s = meetForwardId;
        fwdIdsRev.add(s);
        while (fwdCameFromId[s] != -1) {
            fwdActionsRev.add(fwdActionUsed[s]);
            s = fwdCameFromId[s];
            fwdIdsRev.add(s);
        }
        Collections.reverse(fwdIdsRev);
        Collections.reverse(fwdActionsRev);

        List<Integer> bwdIdsFwdOrder = new ArrayList<>();
        List<Action> bwdActionsFwdOrder = new ArrayList<>();
        int b = meetBackwardId;
        bwdIdsFwdOrder.add(b);
        while (bwdCameFromId[b] != -1) {
            bwdActionsFwdOrder.add(bwdActionUsed[b]);
            b = bwdCameFromId[b];
            bwdIdsFwdOrder.add(b);
        }

        List<StateCodec.State> path = new ArrayList<>();
        for (int id : fwdIdsRev) {
            path.add(StateCodec.toRecord(fwdIds.packedStateOf(id)));
        }
        for (int i = 1; i < bwdIdsFwdOrder.size(); i++) {
            path.add(StateCodec.toRecord(bwdIds.packedStateOf(bwdIdsFwdOrder.get(i))));
        }

        List<Action> actions = new ArrayList<>(fwdActionsRev);
        actions.addAll(bwdActionsFwdOrder);

        return new SearchResult(true, path.toArray(new StateCodec.State[0]), mu,
                actions.toArray(new Action[0]), expansions);
    }
}
