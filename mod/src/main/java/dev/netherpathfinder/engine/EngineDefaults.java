package dev.netherpathfinder.engine;

/**
 * Default cost/search-tuning constants for EdgeRules and WeightedAStar.
 * These are plain fixed defaults (not user-configurable at runtime here) --
 * a higher layer (e.g. a /pathfind config command) can still override the
 * EdgeRules volatile fields these seed directly.
 */
public final class EngineDefaults {
    private EngineDefaults() {}

    public static double boatCrawlTax() { return 5.0; }
    public static double boatCrawlSpeed() { return 3.0; }
    public static double sprintSpeed() { return 5.6; }
    public static double walkSpeed() { return 4.3; }
    public static double jumpPenalty() { return 0.15; }
    public static double speedBridgeSpeed() { return 3.2; }
    public static double towerSpeed() { return 2.0; }
    public static double placeTime() { return 0.0; }
    public static double bridgeRiskPenalty() { return 0.08; }
    public static double blockTaxBase() { return 0.04; }
    public static double blockTaxScarcityScale() { return 2.0; }
    public static int preferredRouteBlocks() { return 20; }
    public static double lowBlockPenaltyPerBlock() { return 0.015; }
    public static double soulSandSpeed() { return 2.5; }
    public static double magmaHazardPenalty() { return 3.0; }
    public static double fireHazardPenalty() { return 4.0; }
    public static int clutchThreshold() { return 3; }
    public static double clutchSetupTime() { return 0.4; }
    public static double fallPenaltyPerBlock() { return 0.05; }
    public static double lavaDeathPenalty() { return 1_000_000; }
    public static double airPotentialScale() { return 3.0; }
    public static double minePruneThreshold() { return 0.02; }
    public static double bridgeUpPruneRatio() { return 0.0; }
    public static int maxReverseBridgeDepth() { return 32; }
    public static int maxReverseFallScan() { return 24; }
    public static double mineDurabilityTax() { return 0.2; }
    public static double mineDurabilityTaxIron() { return 0.5; }
    public static double mineDurabilityTaxStone() { return 1.0; }
    public static double mineDurabilityTaxGolden() { return 2.5; }
    public static double parkourSpeed() { return 6.5; }
    public static double parkourRiskBase() { return 0.8; }
    public static double parkourRiskPerBlock() { return 0.15; }
    public static double parkourCheapRiskTax() { return 0.05; }

    public static WeightedAStar.MinePruneMode minePruneMode() {
        return WeightedAStar.MinePruneMode.AIR_POTENTIAL;
    }
}
