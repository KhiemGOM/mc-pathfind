package dev.netherpathfinder.engine;

/**
 * Per-block mine times (seconds) for a specific pickaxe tier -- expresses
 * that a diamond pickaxe makes netherrack near-instant while obsidian stays
 * slow, or that soul sand is shovel-class and no pickaxe helps at all.
 *
 * Times follow the vanilla break-time formula:
 *   seconds = 1.5 * hardness / speed   when the tool can harvest the block
 *   seconds = 5.0 * hardness / speed   when it can't (wrong class or tier)
 * where speed is the pickaxe's multiplier only for pickaxe-class blocks
 * (1.0 otherwise). E.g. netherrack (hardness 0.4): 2.0s by hand, 0.3s
 * wooden, 0.075s diamond; obsidian (hardness 50): 250s by hand, 9.4s
 * diamond.
 *
 * The table is precomputed per tier at construction, so EdgeRules' hot
 * MINE cost lookups stay a single array read.
 */
public final class MineCostModel {

    public enum PickaxeTier {
        NONE(1.0, -1),
        WOODEN(2.0, 0),
        GOLDEN(12.0, 0),
        STONE(4.0, 1),
        IRON(6.0, 2),
        DIAMOND(8.0, 3),
        NETHERITE(9.0, 3);

        final double speed;
        final int harvestLevel;

        PickaxeTier(double speed, int harvestLevel) {
            this.speed = speed;
            this.harvestLevel = harvestLevel;
        }

        /** True when this tier mines pickaxe-class blocks strictly faster than other. */
        public boolean betterThan(PickaxeTier other) {
            return speed > other.speed
                || (speed == other.speed && harvestLevel > other.harvestLevel);
        }

        /**
         * Selection priority for picking ONE tier to model an entire search
         * with, when multiple tiers are held at once -- deliberately NOT
         * betterThan's raw mining-speed comparison (which ranks GOLDEN above
         * every other tier unconditionally, since its speed of 12 beats even
         * diamond/netherite's 8/9). A single golden pickaxe is fragile, so
         * ordinary route planning should prefer diamond/iron over it rather
         * than blindly chasing raw speed. preferGolden flips gold to the very
         * top of the ranking for contexts where that fragility risk is no
         * longer the deciding factor.
         */
        public int selectionPriority(boolean preferGolden) {
            if (preferGolden && this == GOLDEN) return 100;
            switch (this) {
                case NETHERITE: return 6;
                case DIAMOND: return 5;
                case IRON: return 4;
                case STONE: return 3;
                case GOLDEN: return 2;
                case WOODEN: return 1;
                default: return 0;
            }
        }

        /** True when this tier should be picked over other for whole-search tier selection -- see selectionPriority. */
        public boolean preferredOver(PickaxeTier other, boolean preferGolden) {
            return selectionPriority(preferGolden) > other.selectionPriority(preferGolden);
        }
    }

    /** Matches the legacy BlockType default for unknown block codes. */
    private static final double DEFAULT_MINE_TIME = 2.0;

    // Per block code: vanilla hardness, whether a pickaxe speeds it up, and
    // the minimum harvestLevel for the fast (1.5x) formula. Non-pickaxe
    // blocks (dirt-like, soul sand) are hand-harvestable, so they always use
    // the 1.5x formula at speed 1.0.
    private static final double[] HARDNESS = buildHardness();
    private static final boolean[] PICKAXE_CLASS = buildPickaxeClass();
    private static final int[] HARVEST_LEVEL = buildHarvestLevel();

    private static double[] buildHardness() {
        double[] h = new double[16];
        java.util.Arrays.fill(h, -1.0); // -1 = unknown, falls back to DEFAULT_MINE_TIME
        h[BlockType.DIRT] = 0.5;
        h[BlockType.NETHERRACK] = 0.4;
        h[BlockType.STONE] = 1.5;      // blackstone/basalt/brick band
        h[BlockType.OBSIDIAN] = 50.0;
        h[BlockType.MAGMA] = 0.5;
        h[BlockType.SOUL_SAND] = 0.5;
        // Verified against minecraft.wiki/w/Breaking (see BlockType's
        // WOOD_TIER/SOFT_ORGANIC/CHAIN/LODESTONE/BONE_BLOCK javadoc for
        // why these needed their own codes instead of falling through to
        // DIRT's 0.5).
        h[BlockType.WOOD_TIER] = 2.0;
        h[BlockType.SOFT_ORGANIC] = 1.0;
        h[BlockType.CHAIN] = 5.0;
        h[BlockType.LODESTONE] = 3.5;
        h[BlockType.BONE_BLOCK] = 2.0;
        return h;
    }

    private static boolean[] buildPickaxeClass() {
        boolean[] p = new boolean[16];
        p[BlockType.NETHERRACK] = true;
        p[BlockType.STONE] = true;
        p[BlockType.OBSIDIAN] = true;
        p[BlockType.MAGMA] = true;
        // Chain, Lodestone, and Bone Block are genuinely pickaxe-class in
        // vanilla. WOOD_TIER (Crimson/Warped Stem & Hyphae) and
        // SOFT_ORGANIC (Nether Wart Block, Warped Wart Block, Shroomlight)
        // are best-tool axe/hoe respectively, NOT pickaxe -- this model
        // has no axe/hoe speed concept at all, so leaving them false
        // (hand-equivalent 1.5x formula, no pickaxe bonus) is the
        // accurate choice, not an omission.
        p[BlockType.CHAIN] = true;
        p[BlockType.LODESTONE] = true;
        p[BlockType.BONE_BLOCK] = true;
        return p;
    }

    private static int[] buildHarvestLevel() {
        int[] l = new int[16];
        l[BlockType.OBSIDIAN] = 3; // diamond or netherite
        // Chain/Lodestone/Bone Block all only require a wooden pickaxe
        // (harvestLevel 0, the array's default) to harvest at the fast
        // 1.5x rate -- no explicit entry needed, listed here for clarity.
        return l;
    }

    private final double[] mineTime; // indexed by block code
    // Nullable -- only set for forTier's real pickaxe tiers, not the
    // scaled() back-compat path (which has no single tier concept). Lets
    // EdgeRules look up a tier-appropriate durability tax without needing a
    // separate parameter threaded through every MINE/BOAT_CRAWL call site.
    private final PickaxeTier tier;

    private MineCostModel(double[] mineTime, PickaxeTier tier) {
        this.mineTime = mineTime;
        this.tier = tier;
    }

    public static MineCostModel forTier(PickaxeTier tier) {
        double[] t = new double[HARDNESS.length];
        for (int block = 0; block < t.length; block++) {
            double hardness = HARDNESS[block];
            if (hardness < 0.0) {
                t[block] = DEFAULT_MINE_TIME;
                continue;
            }
            if (!PICKAXE_CLASS[block]) {
                t[block] = 1.5 * hardness; // hand-harvestable, pickaxe irrelevant
                continue;
            }
            boolean canHarvest = tier.harvestLevel >= HARVEST_LEVEL[block];
            t[block] = (canHarvest ? 1.5 : 5.0) * hardness / tier.speed;
        }
        return new MineCostModel(t, tier);
    }

    /**
     * Back-compat model: the legacy BlockType mine-time table scaled by a
     * flat toolMultiplier, for callers that don't need per-tier fidelity.
     */
    public static MineCostModel scaled(double toolMultiplier) {
        double[] t = new double[HARDNESS.length];
        for (int block = 0; block < t.length; block++) {
            t[block] = BlockType.mineTime(block) * toolMultiplier;
        }
        return new MineCostModel(t, null);
    }

    public double mineTime(int block) {
        if (block < 0 || block >= mineTime.length) {
            return DEFAULT_MINE_TIME;
        }
        return mineTime[block];
    }

    /** Null for the scaled() back-compat path -- callers should treat null as the baseline tier. */
    public PickaxeTier tier() {
        return tier;
    }
}
