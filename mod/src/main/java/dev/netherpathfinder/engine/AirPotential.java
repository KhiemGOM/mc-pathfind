package dev.netherpathfinder.engine;

/**
 * Turns an AirPotentialField lookup into a hard PRUNE decision for MINE
 * candidates: below the threshold, the candidate is skipped entirely rather
 * than just made more expensive. See AirPotentialField's javadoc for the
 * precompute; this class is just the formula, kept separate so the tuning
 * constant (scaleBfs) can be adjusted without touching the BFS itself.
 *
 * potential = f(delta) * g(euclideanDist), where:
 *   delta = bfsDist[mineTargetChunk] - bfsDist[currentChunk]
 *   f(delta) = 0                                       if delta <= OLD_GROUND_CUTOFF
 *            = 1 - exp(-(delta - OLD_GROUND_CUTOFF) / scaleBfs)   otherwise
 *   g(dist)  = 1 / dist^2                     (classic inverse-square:
 *              only a THIN wall counts; a long tunnel doesn't, even
 *              toward a promising pocket)
 *
 * A chunk that was never reached by the start-seeded air BFS
 * (AirPotentialField.UNREACHED -- can happen for the player's OWN chunk
 * too, since chunk-occupancy-only adjacency can miss a real but thin
 * connection) yields potential=0 -- "no signal" is treated as "assume
 * not worth it", i.e. it gets pruned, not given a free pass.
 */
public final class AirPotential {
    /**
     * delta <= this is "old ground" (f=0). Default -1: recovers nearly all
     * achievable path quality on real terrain while keeping search time
     * comfortably bounded (see the mine-prune experiments documented
     * alongside this engine's benchmark suite for the sweep behind this
     * default).
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

        double dx = targetX - curX;
        double dy = targetY - curY;
        double dz = targetZ - curZ;
        double euclidDistSq = dx * dx + dy * dy + dz * dz; // always >= 1 (MINE targets are never the current cell)
        double g = 1.0 / euclidDistSq;

        return f * g;
    }

    /**
     * Hard prune check: true iff potential is below pruneThreshold, meaning
     * the caller should skip generating this MINE candidate ENTIRELY.
     */
    public boolean shouldPruneMine(double pruneThreshold, int curX, int curY, int curZ, int targetX, int targetY, int targetZ) {
        return potential(curX, curY, curZ, targetX, targetY, targetZ) < pruneThreshold;
    }
}
