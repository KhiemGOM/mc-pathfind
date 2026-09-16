package dev.netherpathfinder.engine;

/**
 * Block type codes, per-type mine times, and solidity/hazard predicates for the
 * pathfinder's voxel model.
 */
public final class BlockType {
    public static final int AIR = 0;
    public static final int DIRT = 1;
    public static final int STONE = 2;
    public static final int OBSIDIAN = 3;
    public static final int BEDROCK = 4; // unbreakable
    public static final int LAVA = 5;    // impassable but not solid
    public static final int MAGMA = 6;   // solid, but damages an unprotected runner
    public static final int SOUL_SAND = 7; // solid and substantially slows movement
    public static final int FIRE = 8;    // passable, but hazardous
    public static final int NETHERRACK = 9; // solid; very fast to mine with any pickaxe
    // Split out wherever hardness or pickaxe-class actually differs from
    // DIRT/STONE/NETHERRACK's values (verified against minecraft.wiki/w/Breaking).
    // Respawn Anchor is NOT here since it shares OBSIDIAN's exact hardness (50)
    // and harvest level (diamond+) and is classified as OBSIDIAN directly instead.
    public static final int WOOD_TIER = 10;    // Crimson/Warped Stem & Hyphae (+stripped) -- axe-class in vanilla, so a pickaxe gives NO speed bonus here
    public static final int SOFT_ORGANIC = 11; // Nether Wart Block, Warped Wart Block, Shroomlight -- hoe-class in vanilla, same "no pickaxe bonus" treatment
    public static final int CHAIN = 12;        // genuinely pickaxe-class, hardness far above DIRT's assumed value
    public static final int LODESTONE = 13;    // pickaxe-class, hardness 3.5
    public static final int BONE_BLOCK = 14;   // pickaxe-class, hardness 2.0 (same hardness as WOOD_TIER but IS pickaxe-boosted, so needs its own code)

    public static final int VOID = -1;   // out of bounds sentinel
    /**
     * Terrain that has not been revealed yet (an unloaded chunk): distinct from
     * AIR and VOID, and never returned by {@link World#voxelAt} -- it is
     * resolved to an assumed block on read. Storing it as a real type is what
     * lets the snapshot tell "unseen" apart from "seen, and empty".
     */
    public static final int UNKNOWN = -2;

    /** hardness -> base seconds to mine; default 2.0 for anything not listed. */
    private static final double DEFAULT_MINE_TIME = 2.0;
    private static final double[] MINE_TIME = buildMineTime();

    private static double[] buildMineTime() {
        // Sized to comfortably cover known block codes; grow if new types are added.
        double[] t = new double[16];
        java.util.Arrays.fill(t, DEFAULT_MINE_TIME);
        t[DIRT] = 0.3;
        t[STONE] = 1.2;
        t[OBSIDIAN] = 9.0;
        t[MAGMA] = 0.5;
        t[SOUL_SAND] = 0.3;
        t[NETHERRACK] = 0.3;
        // Real hardness values (minecraft.wiki/w/Breaking), same "hand-equivalent"
        // 1.5x-hardness approximation the rest of this table already uses (this
        // legacy table has no pickaxe-tier concept at all -- see MineCostModel for
        // the tier-aware live model, which needs the same values duplicated separately).
        t[WOOD_TIER] = 3.0;     // 1.5 * hardness 2.0
        t[SOFT_ORGANIC] = 1.5;  // 1.5 * hardness 1.0
        t[CHAIN] = 7.5;         // 1.5 * hardness 5.0
        t[LODESTONE] = 5.25;    // 1.5 * hardness 3.5
        t[BONE_BLOCK] = 3.0;    // 1.5 * hardness 2.0
        // BEDROCK and LAVA remain at the default -- they never reach a
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

    public static boolean isSolid(int block) {
        return block == DIRT || block == STONE || block == OBSIDIAN || block == BEDROCK
            || block == MAGMA || block == SOUL_SAND || block == NETHERRACK
            || block == WOOD_TIER || block == SOFT_ORGANIC || block == CHAIN
            || block == LODESTONE || block == BONE_BLOCK;
    }

    public static boolean isLava(int block) {
        return block == LAVA;
    }

    public static boolean isUnbreakable(int block) {
        return block == BEDROCK;
    }

    public static boolean isHazard(int block) {
        return block == MAGMA || block == FIRE || block == LAVA;
    }

    public static boolean isVoid(int block) {
        return block == VOID;
    }

    private BlockType() {}
}
