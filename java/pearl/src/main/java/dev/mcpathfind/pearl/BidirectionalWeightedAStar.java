package dev.mcpathfind.pearl;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Bidirectional weighted A*, forked from dev.mcpathfind.core's version for
 * the trimmed action set -- see there for the full meet-in-the-middle
 * termination rationale (the grace-period mitigation for the non-rigorous
 * sum-based stopping rule, the direct-hit fallback guarantee), UNCHANGED
 * here.
 *
 * The one big structural difference: dev.mcpathfind.core's version needed
 * an entire "worst-case funding estimate + exact cost-correction at
 * reconstruction time" machinery for BRIDGE, because BRIDGE's cost depends
 * on blockTax(blocksRemaining) -- the ABSOLUTE resource count at placement
 * time, which backward search (working outward from the goal) can't know
 * in advance. NONE of that applies here: StateCodec dropped blocksRemaining
 * entirely (nothing left that consumes it), and PEARL's reverse probe
 * (EdgeRules.reversePearlSources) computes the exact same cost forward
 * would via the exact same solvePearlEdge call -- there is no
 * resource-dependent estimate to correct. So there's no bwdBridgesUsed, no
 * funding check gating the mu update, and reconstructMeeting is a plain
 * stitch of the two segments with no per-step cost correction.
 *
 * A smaller consequence of the same fact: forward and backward states are
 * now the EXACT SAME shape (x,y,z,crawling) -- core's version needed a
 * "posKey projection" step to zero out forward's blocksRemaining before
 * looking it up in backward's key space; that projection is now the
 * identity (posKey == state), so it's just gone.
 */
public final class BidirectionalWeightedAStar {

    public static boolean DEBUG = false;

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
    private int[] bwdCameFromId;  // bwd id -> bwd id; T where forward edge S->T (action bwdActionUsed[S]) was reverse-discovered
    private Action[] bwdActionUsed;
    private boolean[] bwdClosed;
    private IntIdOpenHeap bwdHeap;

    // Projection of forward-closed states onto (x,y,z,crawling) -- now the
    // IDENTITY projection (see class javadoc), so posId is just internBwd
    // of the forward state's own packed long, tracking the best forward g
    // and which full forward state achieved it. Indexed by BWD id.
    private double[] fwdPosBestG;
    private int[] fwdPosBestStateId; // -1 = none yet; else a FORWARD id

    private double mu;
    private int meetForwardId = -1;
    private int meetBackwardId = -1;

    // See dev.mcpathfind.core.BidirectionalWeightedAStar's class javadoc
    // ("MEET-IN-THE-MIDDLE TERMINATION") for the full non-rigorous-stopping-
    // rule rationale, unchanged here.
    public static int MU_GRACE_EXPANSIONS = 2000;
    private int expansionsSinceMuImproved = 0;

    private int startX, startY, startZ;
    private int startId;

    private AirPotential airPotential;

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

        // Seed backward search from EVERY goal point -- see GoalPoints'
        // class javadoc for the "virtual goal node" construction.
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

        int expansions = 0;
        while (true) {
            if (fwdHeap.isEmpty() && bwdHeap.isEmpty()) {
                return new SearchResult(false, null, 0.0, null, expansions);
            }

            if (!fwdHeap.isEmpty()) {
                Boolean directHit = stepForward();
                if (directHit != null) {
                    expansions++;
                    expansionsSinceMuImproved++;
                    if (directHit) {
                        return reconstructDirect(expansions);
                    }
                    if (expansions > maxExpansions) {
                        return new SearchResult(false, null, 0.0, null, expansions);
                    }
                }
            }
            if (!bwdHeap.isEmpty()) {
                boolean didStep = stepBackward();
                if (didStep) {
                    expansions++;
                    expansionsSinceMuImproved++;
                    if (expansions > maxExpansions) {
                        return new SearchResult(false, null, 0.0, null, expansions);
                    }
                }
            }

            if (mu < Double.POSITIVE_INFINITY && !fwdHeap.isEmpty() && !bwdHeap.isEmpty()) {
                if (fwdHeap.peekPriority() + bwdHeap.peekPriority() >= mu
                        && expansionsSinceMuImproved >= MU_GRACE_EXPANSIONS) {
                    if (DEBUG) {
                        System.err.printf("[terminate via meeting] topFwd=%.4f topBwd=%.4f mu=%.4f expansionsSinceMuImproved=%d expansions=%d%n",
                                fwdHeap.peekPriority(), bwdHeap.peekPriority(), mu, expansionsSinceMuImproved, expansions);
                    }
                    return reconstructMeeting(expansions);
                }
            }
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

    /** Returns null if the pop was a stale (already-closed) entry; otherwise true iff this expansion IS the real goal. */
    private Boolean stepForward() {
        int stateId = fwdHeap.popId();
        if (fwdClosed[stateId]) {
            return null;
        }
        fwdClosed[stateId] = true;

        long state = fwdIds.packedStateOf(stateId);
        int x = StateCodec.unpackX(state);
        int y = StateCodec.unpackY(state);
        int z = StateCodec.unpackZ(state);
        boolean crawling = StateCodec.unpackCrawling(state);
        double g = fwdG[stateId];

        if (goal.contains(x, y, z)) {
            currentForwardGoalId = stateId;
            return true;
        }

        // posKey is the identity projection now (see class javadoc) -- the
        // forward state's own packed long IS already backward-shaped.
        int posId = internBwd(state);
        if (g < fwdPosBestG[posId]) {
            fwdPosBestG[posId] = g;
            fwdPosBestStateId[posId] = stateId;
        }
        if (bwdClosed[posId]) {
            double candidate = g + bwdG[posId];
            if (candidate < mu) {
                if (DEBUG) {
                    System.err.printf("[stepForward mu update] mu %.4f -> %.4f at (%d,%d,%d,crawl=%b) fwdG=%.4f bwdG=%.4f%n",
                            mu, candidate, x, y, z, crawling, g, bwdG[posId]);
                }
                mu = candidate;
                meetForwardId = stateId;
                meetBackwardId = posId;
                expansionsSinceMuImproved = 0;
            }
        }

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
        return false;
    }

    private boolean stepBackward() {
        int stateId = bwdHeap.popId();
        if (bwdClosed[stateId]) {
            return false;
        }
        bwdClosed[stateId] = true;

        long state = bwdIds.packedStateOf(stateId);
        int x = StateCodec.unpackX(state);
        int y = StateCodec.unpackY(state);
        int z = StateCodec.unpackZ(state);
        boolean crawling = StateCodec.unpackCrawling(state);
        double g = bwdG[stateId];

        if (fwdPosBestStateId[stateId] != -1) {
            double candidate = g + fwdPosBestG[stateId];
            if (candidate < mu) {
                if (DEBUG) {
                    System.err.printf("[stepBackward mu update] mu %.4f -> %.4f at (%d,%d,%d,crawl=%b) bwdG=%.4f fwdG=%.4f%n",
                            mu, candidate, x, y, z, crawling, g, fwdPosBestG[stateId]);
                }
                mu = candidate;
                meetForwardId = fwdPosBestStateId[stateId];
                meetBackwardId = stateId;
                expansionsSinceMuImproved = 0;
            }
        }

        // Backward-direct-hit: symmetric mirror of stepForward()'s
        // goal.contains(x,y,z) check, for backward reaching the exact
        // START -- see core's class javadoc. No funding check needed here
        // (unlike core's version): nothing left costs a consumable
        // resource, so there's nothing to verify affordability of.
        if (!crawling && x == startX && y == startY && z == startZ) {
            double candidate = g; // fwdG[startId] is always exactly 0.0
            if (candidate < mu) {
                if (DEBUG) {
                    System.err.printf("[stepBackward backward-direct-hit] mu %.4f -> %.4f bwdG=%.4f%n", mu, candidate, g);
                }
                mu = candidate;
                meetForwardId = startId;
                meetBackwardId = stateId;
                expansionsSinceMuImproved = 0;
            }
        }

        EdgeRules.EdgeConsumer consumer = (predecessor, cost, action) -> relaxBackward(stateId, g, predecessor, cost, action);
        if (crawling) {
            EdgeRules.reverseCrawlEdges(world, x, y, z, toolMultiplier, consumer);
        } else {
            EdgeRules.reverseNonCrawlEdges(world, x, y, z, toolMultiplier, airPotential, consumer);
        }
        return true;
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
            bwdCameFromId[toId] = fromId; // fromId is "closer to goal" than toId
            bwdActionUsed[toId] = action;  // forward action toState -> fromState
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

    private SearchResult reconstructDirect(int expansions) {
        return reconstructFromForwardId(currentForwardGoalId, fwdG[currentForwardGoalId], expansions);
    }

    private int currentForwardGoalId;

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
     * Stitches the accepted meeting into a full path. Much simpler than
     * dev.mcpathfind.core's version -- no per-step cost correction needed
     * (see class javadoc: nothing here has a resource-dependent cost the
     * way BRIDGE did), so this is just two ID chains walked and concatenated,
     * with mu as the exact already-correct total cost.
     */
    private SearchResult reconstructMeeting(int expansions) {
        // forward half: start .. meetForwardState (standard forward reconstruction)
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

        // backward half: meetBackwardId .. goal, already in forward chronological order
        // (bwdCameFromId[S] is the node closer to goal, bwdActionUsed[S] is the forward action S -> bwdCameFromId[S]).
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
        // skip index 0 of the backward segment: it's the meeting point, already the last entry of the forward segment
        for (int i = 1; i < bwdIdsFwdOrder.size(); i++) {
            path.add(StateCodec.toRecord(bwdIds.packedStateOf(bwdIdsFwdOrder.get(i))));
        }

        List<Action> actions = new ArrayList<>(fwdActionsRev);
        actions.addAll(bwdActionsFwdOrder);

        return new SearchResult(true, path.toArray(new StateCodec.State[0]), mu,
                actions.toArray(new Action[0]), expansions);
    }
}
