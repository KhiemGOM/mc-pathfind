package dev.netherpathfinder.engine;

/** A small set of explicit positions, any one of which completes a search. */
public final class GoalPoints {
    private final int[] xs;
    private final int[] ys;
    private final int[] zs;
    private final int[] blockReserves;

    public GoalPoints(int[] xs, int[] ys, int[] zs) {
        this(xs, ys, zs, new int[xs.length]);
    }

    public GoalPoints(int[] xs, int[] ys, int[] zs, int[] blockReserves) {
        if (xs.length == 0 || xs.length != ys.length || xs.length != zs.length
                || xs.length != blockReserves.length) {
            throw new IllegalArgumentException(
                "GoalPoints needs at least one point and equal-length x/y/z/reserve arrays");
        }
        this.xs = xs;
        this.ys = ys;
        this.zs = zs;
        this.blockReserves = blockReserves;
    }

    public static GoalPoints point(int x, int y, int z) {
        return new GoalPoints(new int[] {x}, new int[] {y}, new int[] {z});
    }

    public static GoalPoints point(StateCodec.State state) {
        return point(state.x(), state.y(), state.z());
    }

    public int size() {
        return xs.length;
    }

    public int x(int index) {
        return xs[index];
    }

    public int y(int index) {
        return ys[index];
    }

    public int z(int index) {
        return zs[index];
    }

    public int blockReserve(int index) {
        return Math.max(0, blockReserves[index]);
    }

    public int minimumBlockReserve() {
        int minimum = Integer.MAX_VALUE;
        for (int reserve : blockReserves) minimum = Math.min(minimum, Math.max(0, reserve));
        return minimum == Integer.MAX_VALUE ? 0 : minimum;
    }

    public boolean contains(int x, int y, int z) {
        return contains(x, y, z, Integer.MAX_VALUE);
    }

    public boolean contains(int x, int y, int z, int blocksRemaining) {
        for (int i = 0; i < xs.length; i++) {
            if (matches(i, x, y, z, blocksRemaining)) return true;
        }
        return false;
    }

    public boolean matches(int index, int x, int y, int z, int blocksRemaining) {
        return xs[index] == x && ys[index] == y && zs[index] == z
            && blocksRemaining >= blockReserve(index);
    }

    public int indexOf(int x, int y, int z) {
        for (int i = 0; i < xs.length; i++) {
            if (xs[i] == x && ys[i] == y && zs[i] == z) return i;
        }
        return -1;
    }
}
