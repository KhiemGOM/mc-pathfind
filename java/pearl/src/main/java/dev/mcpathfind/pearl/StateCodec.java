package dev.mcpathfind.pearl;

/**
 * Packs (x, y, z, crawling) into a single primitive long so the hot loop
 * never boxes a state. Forked from dev.mcpathfind.core.StateCodec, which
 * see for the full rationale (unchanged here) -- the one real difference is
 * blocksRemaining is dropped entirely: nothing in this action set (no
 * BRIDGE/BRIDGE_UP) ever consumes a resource that needs tracking in state,
 * so carrying an always-0 field along would just be dead weight.
 *
 * Bit layout (MSB to LSB), 43 of 64 bits used, 21 reserved (always 0):
 *   [63..43] reserved
 *   [42..27] x                (16 bits, 0..65535)
 *   [26..11] z                (16 bits, 0..65535)
 *   [10..1]  y                (10 bits, 0..1023)
 *   [0]      crawling         (1 bit)
 */
public final class StateCodec {
    private static final int X_BITS = 16;
    private static final int Z_BITS = 16;
    private static final int Y_BITS = 10;

    private static final int X_SHIFT = 27;
    private static final int Z_SHIFT = 11;
    private static final int Y_SHIFT = 1;

    public static final int MAX_COORD = (1 << X_BITS) - 1; // shared bound for x/z
    public static final int MAX_Y = (1 << Y_BITS) - 1;

    public static long pack(int x, int y, int z, boolean crawling) {
        if (x < 0 || x > MAX_COORD) throw new IllegalArgumentException("x out of range: " + x);
        if (z < 0 || z > MAX_COORD) throw new IllegalArgumentException("z out of range: " + z);
        if (y < 0 || y > MAX_Y) throw new IllegalArgumentException("y out of range: " + y);
        return ((long) x << X_SHIFT)
                | ((long) z << Z_SHIFT)
                | ((long) y << Y_SHIFT)
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

    public static boolean unpackCrawling(long state) {
        return (state & 1L) != 0L;
    }

    /** Small boundary-only record: start/goal construction, path reconstruction for callers/tests. */
    public record State(int x, int y, int z, boolean crawling) {
        public long pack() {
            return StateCodec.pack(x, y, z, crawling);
        }
    }

    public static State toRecord(long packed) {
        return new State(unpackX(packed), unpackY(packed), unpackZ(packed), unpackCrawling(packed));
    }

    private StateCodec() {}
}
