package dev.netherpathfinder.engine;

/**
 * Flat voxel array, C-order (matches numpy's default layout for shape
 * (sizeX, sizeY, sizeZ)): index = (x*sizeY + y)*sizeZ + z.
 */
public final class World {
    /** No ground context set: unresolved UNKNOWN reads fall back to a flat AIR assumption. */
    public static final int NO_GROUND_CONTEXT = Integer.MIN_VALUE;
    private static final int[] NO_STANDABLE_LEVELS = new int[0];

    public final int sizeX, sizeY, sizeZ;
    public final byte[] blocks;

    private int unknownGroundY = NO_GROUND_CONTEXT;

    // Lazily-built cache of standable Y-levels per (x,z) column, flat-
    // indexed by x*sizeZ+z -- lets repeated per-column queries against the
    // same handful of columns from many different source states across a
    // real search skip an O(sizeY) rescan each time. AtomicReferenceArray
    // gives lock-free thread safety if a World is ever shared across
    // multiple search threads; a benign race on first population is fine
    // since the result is a pure function of static terrain (see
    // standableColumnYs' javadoc).
    private final java.util.concurrent.atomic.AtomicReferenceArray<int[]> standableColumnCache;

    public World(int sizeX, int sizeY, int sizeZ, byte[] blocks) {
        if (blocks.length != (long) sizeX * sizeY * sizeZ) {
            throw new IllegalArgumentException(
                "blocks.length=" + blocks.length + " does not match sizeX*sizeY*sizeZ=" + ((long) sizeX * sizeY * sizeZ));
        }
        this.sizeX = sizeX;
        this.sizeY = sizeY;
        this.sizeZ = sizeZ;
        this.blocks = blocks;
        this.standableColumnCache = new java.util.concurrent.atomic.AtomicReferenceArray<>(sizeX * sizeZ);
    }

    /**
     * Sets the y-level that subsequent UNKNOWN reads treat as standing height.
     * The search sets this to the y of the state it is expanding, so
     * unrevealed terrain looks like normal ground underfoot with headroom
     * above -- rather than bottomless void (which provokes a bridge-storm)
     * or solid rock (which provokes a mine-storm, and with an unbreakable
     * filler walls the goal off entirely).
     *
     * <p>Scoping this on the world instead of threading it through every
     * voxelAt call keeps EdgeRules unaware of the known/unknown distinction.
     * Safe only because a search is single-threaded and non-reentrant.
     */
    public void unknownContext(int groundY) {
        this.unknownGroundY = groundY;
    }

    /**
     * Bounds-checked lookup; returns BlockType.VOID (-1) out of bounds, and
     * never returns UNKNOWN -- see {@link #unknownContext}.
     */
    public int voxelAt(int x, int y, int z) {
        if (x < 0 || x >= sizeX || y < 0 || y >= sizeY || z < 0 || z >= sizeZ) {
            return BlockType.VOID;
        }
        int block = blocks[(x * sizeY + y) * sizeZ + z];
        if (block != BlockType.UNKNOWN) {
            return block;
        }
        if (unknownGroundY == NO_GROUND_CONTEXT) {
            return BlockType.AIR;
        }
        return y < unknownGroundY ? BlockType.DIRT : BlockType.AIR;
    }

    /** True iff (x,y,z) is a legal standing position: solid non-lava floor, clear non-lava feet/head. */
    public boolean isStandableColumn(int x, int y, int z) {
        int below = voxelAt(x, y - 1, z);
        int feet = voxelAt(x, y, z);
        int head = voxelAt(x, y + 1, z);
        return BlockType.isSolid(below) && !BlockType.isLava(below)
            && !BlockType.isSolid(feet) && !BlockType.isLava(feet)
            && !BlockType.isSolid(head) && !BlockType.isLava(head);
    }

    /**
     * All standable Y-levels in the (x,z) column, ascending and cached.
     * Racing threads may compute the same pure terrain result once; either
     * array is correct. The returned array is an immutable-by-contract
     * internal view so repeated lookups don't allocate hundreds of clones;
     * callers must not modify it.
     */
    public int[] standableColumnYs(int x, int z) {
        if (x < 0 || x >= sizeX || z < 0 || z >= sizeZ) {
            return NO_STANDABLE_LEVELS;
        }
        int index = x * sizeZ + z;
        int[] cached = standableColumnCache.get(index);
        if (cached != null) {
            return cached;
        }
        int count = 0;
        for (int y = 1; y < sizeY - 1; y++) {
            if (isStandableColumn(x, y, z)) count++;
        }
        int[] ys = new int[count];
        int i = 0;
        for (int y = 1; y < sizeY - 1; y++) {
            if (isStandableColumn(x, y, z)) ys[i++] = y;
        }
        standableColumnCache.set(index, ys);
        return ys;
    }

    /**
     * Nearest standable Y in the (x,z) column to preferredY (ties broken
     * toward the LOWER Y, matching the original outward-scan's below-
     * before-above check order), or -1 if none -- backed by
     * standableColumnYs' cached per-column list.
     */
    public int nearestStandableColumnY(int x, int z, int preferredY) {
        if (x < 0 || x >= sizeX || z < 0 || z >= sizeZ) {
            return -1;
        }
        int[] ys = standableColumnYs(x, z);
        int best = -1;
        int bestDist = Integer.MAX_VALUE;
        for (int y : ys) {
            int dist = Math.abs(y - preferredY);
            if (dist < bestDist) {
                bestDist = dist;
                best = y;
            }
        }
        return best;
    }
}
