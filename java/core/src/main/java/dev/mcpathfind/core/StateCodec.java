package dev.mcpathfind.core;

/**
 * Packs (x, y, z, blocksRemaining, crawling) into a single primitive long so
 * the hot loop never boxes a state (vs. Python's 5-tuple, hashed through a
 * generic tuple-hash routine on every dict/set operation).
 *
 * Bit layout (MSB to LSB), 51 of 64 bits used, 13 reserved (always 0):
 *   [63..51] reserved
 *   [50..35] x                (16 bits, 0..65535)
 *   [34..19] z                (16 bits, 0..65535)
 *   [18..9]  y                (10 bits, 0..1023)
 *   [8..1]   blocksRemaining  (8 bits,  0..255)
 *   [0]      crawling         (1 bit)
 *
 * Coordinates from World.voxelAt/array indices are always >=0 in this search
 * (void/out-of-bounds is a *return value*, never a *state*), so no sign
 * handling is needed. Ranges are generous headroom above every tested world
 * (largest so far: 368x368x128, blocksAvailable typically <=64).
 */
public final class StateCodec {
    private static final int X_BITS = 16;
    private static final int Z_BITS = 16;
    private static final int Y_BITS = 10;
    private static final int BLOCKS_BITS = 8;

    private static final int X_SHIFT = 35;
    private static final int Z_SHIFT = 19;
    private static final int Y_SHIFT = 9;
    private static final int BLOCKS_SHIFT = 1;

    public static final int MAX_COORD = (1 << X_BITS) - 1; // shared bound for x/z
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

    /** Small boundary-only record: start/goal construction, path reconstruction for callers/tests. */
    public record State(int x, int y, int z, int blocksRemaining, boolean crawling) {
        public long pack() {
            return StateCodec.pack(x, y, z, blocksRemaining, crawling);
        }
    }

    public static State toRecord(long packed) {
        return new State(unpackX(packed), unpackY(packed), unpackZ(packed), unpackBlocks(packed), unpackCrawling(packed));
    }

    private StateCodec() {}
}
