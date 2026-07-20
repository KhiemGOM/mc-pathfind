package dev.mcpathfind.core;

/**
 * Direct port of world.py's block type codes, MINE_TIME table, and predicates.
 */
public final class BlockType {
    public static final int AIR = 0;
    public static final int DIRT = 1;
    public static final int STONE = 2;
    public static final int OBSIDIAN = 3;
    public static final int BEDROCK = 4; // unbreakable
    public static final int LAVA = 5;    // impassable but not solid

    public static final int VOID = -1;   // out of bounds sentinel, matches voxel_at

    /** hardness -> base seconds to mine; default 2.0 for anything not listed (matches Python's MINE_TIME.get(x, 2.0)). */
    private static final double DEFAULT_MINE_TIME = 2.0;
    private static final double[] MINE_TIME = buildMineTime();

    private static double[] buildMineTime() {
        // Sized to comfortably cover known block codes; grow if new types are added.
        double[] t = new double[16];
        java.util.Arrays.fill(t, DEFAULT_MINE_TIME);
        t[DIRT] = 0.3;
        t[STONE] = 1.2;
        t[OBSIDIAN] = 9.0;
        // BEDROCK, LAVA intentionally left at the default -- they never reach a
        // mine-cost computation (guarded by isSolid && !isUnbreakable elsewhere),
        // this only matters if that guard is ever bypassed.
        return t;
    }

    public static double mineTime(int block) {
        if (block < 0 || block >= MINE_TIME.length) {
            return DEFAULT_MINE_TIME;
        }
        return MINE_TIME[block];
    }

    private static final int SOLID_FLAG = 1;
    private static final int LAVA_FLAG = 2;
    private static final int UNBREAKABLE_FLAG = 4;
    private static final byte[] FLAGS = buildFlags();

    private static byte[] buildFlags() {
        byte[] f = new byte[16];
        f[DIRT] = SOLID_FLAG;
        f[STONE] = SOLID_FLAG;
        f[OBSIDIAN] = SOLID_FLAG;
        f[BEDROCK] = SOLID_FLAG | UNBREAKABLE_FLAG;
        f[LAVA] = LAVA_FLAG;
        return f;
    }

    private static int flagsOf(int block) {
        if (block < 0 || block >= FLAGS.length) {
            return 0;
        }
        return FLAGS[block];
    }

    public static boolean isSolid(int block) {
        return (flagsOf(block) & SOLID_FLAG) != 0;
    }

    public static boolean isLava(int block) {
        return (flagsOf(block) & LAVA_FLAG) != 0;
    }

    public static boolean isUnbreakable(int block) {
        return (flagsOf(block) & UNBREAKABLE_FLAG) != 0;
    }

    public static boolean isVoid(int block) {
        return block == VOID;
    }

    private BlockType() {}
}
