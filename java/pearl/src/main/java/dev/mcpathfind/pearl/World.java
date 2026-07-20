package dev.mcpathfind.pearl;

/**
 * Flat voxel array, C-order (matches numpy's default layout for shape
 * (sizeX, sizeY, sizeZ)): index = (x*sizeY + y)*sizeZ + z. This lets
 * WorldBinFormat load an exported world with zero reshaping.
 */
public final class World {
    public final int sizeX, sizeY, sizeZ;
    public final byte[] blocks;

    // Lazily-built cache of standable Y-levels per (x,z) column, flat-
    // indexed by x*sizeZ+z rather than a hash map -- a ConcurrentHashMap
    // keyed by boxed Long profiled as a real cost on its own (~16% of
    // total solve time, mostly computeIfAbsent's bucket lookup/locking)
    // once the O(sizeY) rescans it replaced were gone. Column coordinates
    // are small, dense, bounded integers, so a flat array indexed
    // directly avoids both the key boxing and the hash/bucket walk
    // entirely. AtomicReferenceArray gives the same thread-safety
    // (needed since ParallelBidirectionalWeightedAStar shares one World
    // across threads) via a lock-free get/set instead of per-bucket
    // locking -- see standableColumnYs' javadoc for why a benign race on
    // first population is fine here.
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

    /** Bounds-checked lookup; returns BlockType.VOID (-1) out of bounds, matching voxel_at in world.py. */
    public int voxelAt(int x, int y, int z) {
        if (x < 0 || x >= sizeX || y < 0 || y >= sizeY || z < 0 || z >= sizeZ) {
            return BlockType.VOID;
        }
        return blocks[(x * sizeY + y) * sizeZ + z];
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
     * All standable Y-levels in the (x,z) column, ascending -- computed
     * once per column and cached, since PEARL's coarse-grid corner search
     * (EdgeRules.pearlCellCorners) re-queries the same handful of columns
     * from many different source states across a real search: profiling
     * found isStandableColumn/nearestStandableColumnY were ~38% of total
     * solve time before this cache, almost entirely repeated re-scans of
     * terrain that never changes within a search.
     *
     * Racing threads (ParallelBidirectionalWeightedAStar shares one World)
     * may both miss the cache and both compute+set this for the same
     * column -- harmless: the result is a pure function of static terrain,
     * so either thread's array is equally correct and a duplicate
     * computation just costs a little redundant work once, never an
     * incorrect cached value.
     */
    private int[] standableColumnYs(int x, int z) {
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
     * standableColumnYs' cached per-column list, so repeated queries
     * against the same column cost a scan of an already-computed
     * (typically short) list instead of re-deriving it from scratch.
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
