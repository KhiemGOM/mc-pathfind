package dev.mcpathfind.core;

/**
 * A small, explicit set of acceptable goal positions, replacing a single
 * goal point. A single point is just the degenerate 1-element case (see
 * {@link #point}) -- every method below reduces exactly to the old
 * single-point formulas in that case, so callers that only ever used a
 * point see byte-identical behavior.
 *
 * Correctness basis: searching to a SET of goals via "distance to the
 * NEAREST point in the set" as the heuristic, and stopping as soon as any
 * state in the set is popped off the open list, preserves the same
 * optimality/bounded-suboptimality guarantee weighted A* already relies on
 * for a single point -- the standard "virtual goal node with 0-cost edges
 * from every member of the set" argument. Concretely: the min of several
 * admissible per-point heuristics is itself admissible for "cost to reach
 * the nearest goal", since h_i(n) <= trueCost_i(n) for every i implies
 * min_i h_i(n) <= min_i trueCost_i(n).
 *
 * Deliberately NOT a box/region: this is sized for a handful of specific
 * candidate points (e.g. a bastion's known entrances), not a volume, so
 * every method here is a simple O(size()) scan rather than needing any
 * per-axis clamping trick -- correct and cheap as long as size() stays
 * small (single digits to low tens), which is the only use case this is
 * built for.
 */
public final class GoalPoints {
    private final int[] xs, ys, zs;

    public GoalPoints(int[] xs, int[] ys, int[] zs) {
        if (xs.length == 0 || xs.length != ys.length || xs.length != zs.length) {
            throw new IllegalArgumentException("GoalPoints needs at least one point and equal-length x/y/z arrays");
        }
        this.xs = xs;
        this.ys = ys;
        this.zs = zs;
    }

    public static GoalPoints point(int x, int y, int z) {
        return new GoalPoints(new int[]{x}, new int[]{y}, new int[]{z});
    }

    public static GoalPoints point(StateCodec.State s) {
        return point(s.x(), s.y(), s.z());
    }

    public static GoalPoints of(StateCodec.State... states) {
        int n = states.length;
        int[] xs = new int[n], ys = new int[n], zs = new int[n];
        for (int i = 0; i < n; i++) {
            xs[i] = states[i].x();
            ys[i] = states[i].y();
            zs[i] = states[i].z();
        }
        return new GoalPoints(xs, ys, zs);
    }

    public int size() {
        return xs.length;
    }

    public int x(int i) { return xs[i]; }
    public int y(int i) { return ys[i]; }
    public int z(int i) { return zs[i]; }

    public boolean contains(int x, int y, int z) {
        for (int i = 0; i < xs.length; i++) {
            if (xs[i] == x && ys[i] == y && zs[i] == z) {
                return true;
            }
        }
        return false;
    }
}
