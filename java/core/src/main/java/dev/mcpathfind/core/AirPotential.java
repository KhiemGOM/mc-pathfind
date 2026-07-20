package dev.mcpathfind.core;

/**
 * Turns an AirPotentialField lookup into a hard PRUNE decision for MINE
 * candidates. See AirPotentialField's javadoc for the precompute; this
 * class is just the formula, kept separate so the tuning constant
 * (scaleBfs) can be adjusted without touching the BFS itself.
 *
 * HISTORY: this class originally applied potential as a soft cost
 * DISCOUNT on good MINE candidates. Measured effect: essentially none
 * (MINE's improve rate moved from 11.3% to 11.5%) because it left the
 * common case -- most MINE candidates, which are wasteful -- completely
 * untouched at baseline cost; it only sweetened the rare good ones.
 * It was then corrected to a soft PENALTY (expensive by default, relieved
 * only when potential was high): that measurably cut MINE candidate
 * volume (~10%) and expansions (~8%), but added enough per-candidate
 * overhead that wall-clock got WORSE overall (~3.6s vs ~3.0s baseline).
 * This version replaces the soft penalty with a hard PRUNE: skip
 * generating the MINE candidate entirely below a potential threshold,
 * rather than just making it cost more. Net effect on the real target
 * region (k2_r_0_0): MINE candidates 2.39M -> 121K (-95%), MINE's own
 * improve rate nearly doubled (11.3% -> 19.3%, since only the credible
 * candidates survive), expansions and wall-clock both dropped below the
 * pre-AirPotential baseline. The tradeoff, confirmed empirically on a
 * different region (k1_r_0_0): the prune can and does occasionally skip
 * a MINE step that was genuinely part of the optimal route, degrading
 * that region's found path from cost 21.48s to 26.07s (still a VALID
 * path -- PathValidator clean -- just no longer the cheapest one). This
 * is a real, accepted completeness/optimality tradeoff, not a bug.
 *
 * potential = f(delta) * g(euclideanDist), where:
 *   delta = bfsDist[mineTargetChunk] - bfsDist[currentChunk]
 *   f(delta) = 0                                       if delta <= OLD_GROUND_CUTOFF
 *            = 1 - exp(-(delta - OLD_GROUND_CUTOFF) / scaleBfs)   otherwise
 *   g(dist)  = 1 / dist^2                     (classic inverse-square:
 *              only a THIN wall counts; a long tunnel doesn't, even
 *              toward a promising pocket)
 *
 * OLD_GROUND_CUTOFF (default 0, i.e. "old ground" = delta <= 0) is a
 * mutable static tunable, not baked into the formula, because sweeping it
 * found the ORIGINAL cutoff=0 version was effectively a binary on/off
 * switch rather than a dial: since every MINE candidate is exactly one
 * adjacent block away (dist is always 1 or sqrt(2), never a multi-block
 * tunnel) and delta is an integer chunk-BFS-hop count, f(delta) only ever
 * took two kinds of values -- exactly 0 (delta <= 0, ~95% of candidates)
 * or >= ~0.14 (delta >= 1) -- so MINE_PRUNE_THRESHOLD couldn't land
 * "in between" no matter what it was set to; only threshold=0 (fully
 * disabling the prune) changed anything. Shifting the cutoff to a negative
 * value (e.g. -5) instead of the threshold gives an actual dial: it only
 * treats a MINE target as "old ground" once its chunk is meaningfully
 * CLOSER to start than the current position (a genuine backward dig), and
 * ramps f smoothly for every delta above that, including the delta in
 * (-cutoff, 0] band that used to be hard-zeroed regardless of how close it
 * was to being a new pocket.
 *
 * A chunk that was never reached by the start-seeded air BFS
 * (AirPotentialField.UNREACHED -- can happen for the player's OWN chunk
 * too, since chunk-occupancy-only adjacency can miss a real but thin
 * connection) yields potential=0 -- "no signal" is treated as "assume
 * not worth it", i.e. it gets pruned, not given a free pass.
 */
public final class AirPotential {
    /**
     * delta <= this is "old ground" (f=0); see class javadoc for why this
     * replaced a hard threshold sweep. Default -1 (not 0): sweeping this on
     * the real regions found expansions/cost plateau at cutoff=-2 (fully
     * matching the prune-disabled baseline's path quality), but at that
     * point k1_r_neg1_0's wall-clock creeps just over 1s (~1.03s measured).
     * -1 recovers nearly all of the same quality (e.g. k1_r_0_0 cost 22.01
     * vs the true optimum's 21.48; long1_r_neg1_0 actually beats the
     * prune-disabled baseline) while keeping every measured region
     * comfortably under 1s.
     */
    public static int OLD_GROUND_CUTOFF = -1;

    private final AirPotentialField field;
    private final double scaleBfs;

    public AirPotential(AirPotentialField field, double scaleBfs) {
        this.field = field;
        this.scaleBfs = scaleBfs;
    }

    /**
     * @param curX, curY, curZ    the expanding node's position (player's current position)
     * @param targetX, targetY, targetZ  the MINE candidate's target voxel
     */
    public double potential(int curX, int curY, int curZ, int targetX, int targetY, int targetZ) {
        int curBfs = field.bfsDistAt(curX, curY, curZ);
        int targetBfs = field.bfsDistAt(targetX, targetY, targetZ);
        if (curBfs == AirPotentialField.UNREACHED || targetBfs == AirPotentialField.UNREACHED) {
            return 0.0;
        }

        int delta = targetBfs - curBfs;
        int effectiveDelta = delta - OLD_GROUND_CUTOFF;
        double f = (effectiveDelta <= 0) ? 0.0 : (1.0 - Math.exp(-effectiveDelta / scaleBfs));

        int dx = targetX - curX;
        int dy = targetY - curY;
        int dz = targetZ - curZ;
        int euclidDistSq = dx * dx + dy * dy + dz * dz; // always >= 1 (MINE targets are never the current cell)
        double g = (euclidDistSq == 1) ? 1.0 : (euclidDistSq == 2) ? 0.5 : 1.0 / euclidDistSq;

        return f * g;
    }

    /**
     * Hard prune check: true iff potential is below pruneThreshold, meaning
     * the caller should skip generating this MINE candidate ENTIRELY.
     * See the class javadoc for the completeness/optimality tradeoff this
     * accepts.
     */
    public boolean shouldPruneMine(double pruneThreshold, int curX, int curY, int curZ, int targetX, int targetY, int targetZ) {
        return potential(curX, curY, curZ, targetX, targetY, targetZ) < pruneThreshold;
    }
}
