package dev.mcpathfind.core;

/**
 * Naive "obvious first idea" alternative to AirPotential: prune a MINE
 * candidate unless open space is physically close by, measured as straight
 * Manhattan distance from the dig TARGET to the nearest non-solid voxel --
 * NOT a maze-aware check. Unlike AirPotential's chunk-BFS (which only
 * counts air actually reachable by walking/digging from the search's
 * start), this has no notion of connectivity or novelty at all: a voxel
 * counts as "close" purely by straight-line distance, even if there is no
 * possible path to it without digging through more rock than the "thin
 * wall" this is supposed to detect. Kept here to be A/B'd against
 * AirPotential and MANHATTAN-to-goal pruning, not as a third production
 * option.
 *
 * DEGENERACY THIS HAD TO BE DESIGNED AROUND: every MINE candidate's target
 * is by construction exactly one step from the player's OWN current
 * position (x,y,z), which is always open (you're standing in it). A naive
 * "distance to nearest air" scan from the target would therefore almost
 * always immediately find that adjacent source cell and return 1,
 * regardless of whether there is any OTHER open space nearby -- the exact
 * same "every candidate is one block away, so raw distance is a
 * degenerate on/off switch" trap AirPotential's own javadoc documents for
 * why it uses chunk-hop BFS distance instead of raw adjacency. The fix
 * applied here is the minimum needed to make the idea non-degenerate: the
 * known source cell is excluded from the scan (treated as if solid), so
 * the search is actually asking "is there open space beyond where I'm
 * already standing", not "am I currently standing next to open space"
 * (always true).
 */
public final class NearestAirDistance {
    /** Sentinel: no non-solid voxel found within maxRadius (excluding the source cell). */
    public static final int NOT_FOUND = Integer.MAX_VALUE;

    private NearestAirDistance() {}

    /**
     * Manhattan distance from (x,y,z) to the nearest non-solid voxel, or
     * NOT_FOUND if none within maxRadius. (excludeX,excludeY,excludeZ) is
     * skipped even if non-solid -- see class javadoc for why this exclusion
     * is required, not optional. Shells are scanned in increasing radius
     * order so this returns as soon as anything is found; worst case (no
     * air within maxRadius) is O(maxRadius^3).
     */
    public static int distanceToAir(World world, int x, int y, int z,
                                     int excludeX, int excludeY, int excludeZ, int maxRadius) {
        if (!(x == excludeX && y == excludeY && z == excludeZ) && !BlockType.isSolid(world.voxelAt(x, y, z))) {
            return 0;
        }
        for (int r = 1; r <= maxRadius; r++) {
            for (int dx = -r; dx <= r; dx++) {
                int remX = r - Math.abs(dx);
                for (int dy = -remX; dy <= remX; dy++) {
                    int dz = remX - Math.abs(dy);
                    if (isOpen(world, x + dx, y + dy, z + dz, excludeX, excludeY, excludeZ)) {
                        return r;
                    }
                    if (dz != 0 && isOpen(world, x + dx, y + dy, z - dz, excludeX, excludeY, excludeZ)) {
                        return r;
                    }
                }
            }
        }
        return NOT_FOUND;
    }

    private static boolean isOpen(World world, int x, int y, int z, int excludeX, int excludeY, int excludeZ) {
        if (x == excludeX && y == excludeY && z == excludeZ) {
            return false;
        }
        return !BlockType.isSolid(world.voxelAt(x, y, z));
    }

    /** True iff the target is farther than pruneRadius from any (non-excluded) open space -- skip generating the MINE candidate. */
    public static boolean shouldPrune(World world, int targetX, int targetY, int targetZ,
                                       int sourceX, int sourceY, int sourceZ, int maxRadius, int pruneRadius) {
        int d = distanceToAir(world, targetX, targetY, targetZ, sourceX, sourceY, sourceZ, maxRadius);
        return d > pruneRadius;
    }
}
