package dev.netherpathfinder.engine;

/** Packs a pathfinding state into one primitive long. */
public final class StateCodec {
    private static final int X_BITS = 16;
    private static final int Z_BITS = 16;
    private static final int Y_BITS = 10;
    private static final int BLOCKS_BITS = 8;

    private static final int X_SHIFT = 35;
    private static final int Z_SHIFT = 19;
    private static final int Y_SHIFT = 9;
    private static final int BLOCKS_SHIFT = 1;

    public static final int MAX_COORD = (1 << X_BITS) - 1;
    public static final int MAX_Y = (1 << Y_BITS) - 1;
    public static final int MAX_BLOCKS = (1 << BLOCKS_BITS) - 1;

    public static long pack(int x, int y, int z, int blocksRemaining, boolean crawling) {
        if (x < 0 || x > MAX_COORD) throw new IllegalArgumentException("x out of range: " + x);
        if (z < 0 || z > MAX_COORD) throw new IllegalArgumentException("z out of range: " + z);
        if (y < 0 || y > MAX_Y) throw new IllegalArgumentException("y out of range: " + y);
        if (blocksRemaining < 0 || blocksRemaining > MAX_BLOCKS) {
            throw new IllegalArgumentException("blocksRemaining out of range: " + blocksRemaining);
        }
        return ((long) x << X_SHIFT)
            | ((long) z << Z_SHIFT)
            | ((long) y << Y_SHIFT)
            | ((long) blocksRemaining << BLOCKS_SHIFT)
            | (crawling ? 1L : 0L);
    }

    public static int unpackX(long state) {
        return (int) ((state >>> X_SHIFT) & ((1L << X_BITS) - 1));
    }

    public static int unpackZ(long state) {
        return (int) ((state >>> Z_SHIFT) & ((1L << Z_BITS) - 1));
    }

    public static int unpackY(long state) {
        return (int) ((state >>> Y_SHIFT) & ((1L << Y_BITS) - 1));
    }

    public static int unpackBlocks(long state) {
        return (int) ((state >>> BLOCKS_SHIFT) & ((1L << BLOCKS_BITS) - 1));
    }

    public static boolean unpackCrawling(long state) {
        return (state & 1L) != 0L;
    }

    public static final class State {
        private final int x;
        private final int y;
        private final int z;
        private final int blocksRemaining;
        private final boolean crawling;

        public State(int x, int y, int z, int blocksRemaining, boolean crawling) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.blocksRemaining = blocksRemaining;
            this.crawling = crawling;
        }

        public int x() { return x; }
        public int y() { return y; }
        public int z() { return z; }
        public int blocksRemaining() { return blocksRemaining; }
        public boolean crawling() { return crawling; }

        public long pack() {
            return StateCodec.pack(x, y, z, blocksRemaining, crawling);
        }

        @Override
        public String toString() {
            return "State[x=" + x + ", y=" + y + ", z=" + z
                + ", blocksRemaining=" + blocksRemaining + ", crawling=" + crawling + "]";
        }
    }

    public static State toRecord(long packed) {
        return new State(unpackX(packed), unpackY(packed), unpackZ(packed),
            unpackBlocks(packed), unpackCrawling(packed));
    }

    private StateCodec() {
    }
}
