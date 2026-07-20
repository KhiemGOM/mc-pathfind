package dev.mcpathfind.pearl;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Forked from dev.mcpathfind.core.WeightedAStar for the bastion-to-fortress
 * / fortress-to-stronghold action set (SPRINT/MINE/MINE_DOWN/BOAT_CRAWL/
 * CLIMB/FALL/PEARL, no BRIDGE/BRIDGE_UP/PARKOUR) -- see the core version's
 * javadoc for the full state-representation/dense-id rationale, unchanged
 * here. The one structural difference: no bridgeUpOriginY tracking (nothing
 * left that needs a remembered chain-start y), and no blocksAvailable
 * threading (StateCodec dropped that dimension entirely -- see its javadoc).
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
    private boolean[] closed;     // indexed by state id
    private IntIdOpenHeap heap;
    private AirPotential airPotential;

    /** Same role as dev.mcpathfind.core.WeightedAStar.MinePruneMode -- MINE is still an action here. */
    public enum MinePruneMode { NONE, AIR_POTENTIAL, MANHATTAN }
    public static MinePruneMode minePruneMode = MinePruneMode.AIR_POTENTIAL;

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

        this.ids = new StateIdMap(sizeHint);
        this.gScore = newDoubleArray(sizeHint, Double.POSITIVE_INFINITY);
        this.cameFromId = newIntArray(sizeHint, -1);
        this.actionUsed = new Action[sizeHint];
        this.closed = new boolean[sizeHint];
        this.heap = new IntIdOpenHeap(Math.min(sizeHint, 1_000_000), sizeHint);

        if (minePruneMode == MinePruneMode.AIR_POTENTIAL) {
            AirPotentialField field = AirPotentialField.build(world, start.x(), start.y(), start.z());
            this.airPotential = new AirPotential(field, /* scaleBfs */ 3.0);
        } else {
            this.airPotential = null;
        }

        long startState = StateCodec.pack(start.x(), start.y(), start.z(), false);
        int startId = internId(startState);
        gScore[startId] = 0.0;
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
            double h = heuristic(nx, ny, nz);
            heap.push(tentativeG + epsilon * h, tentativeG, ny, toId);
        }
    }

    private void expand(long state, int stateId, double gState) {
        int x = StateCodec.unpackX(state);
        int y = StateCodec.unpackY(state);
        int z = StateCodec.unpackZ(state);
        boolean crawling = StateCodec.unpackCrawling(state);

        EdgeRules.EdgeConsumer consumer = (toState, cost, action) -> relax(state, stateId, gState, toState, cost, action);

        for (int d = 0; d < 8; d++) {
            EdgeRules.horizontalEdges(world, x, y, z, crawling, EdgeRules.DIR_DX[d], EdgeRules.DIR_DZ[d], toolMultiplier,
                    airPotential, minePruneMode == MinePruneMode.MANHATTAN, goal, consumer);
        }
        for (int d = 0; d < 8; d++) {
            EdgeRules.climbEdge(world, x, y, z, EdgeRules.DIR_DX[d], EdgeRules.DIR_DZ[d], consumer);
        }
        EdgeRules.mineDownEdge(world, x, y, z, toolMultiplier, consumer);
        EdgeRules.pearlEdges(world, x, y, z, consumer);
    }
}
