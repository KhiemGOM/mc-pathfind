package dev.mcpathfind.pearl;

/**
 * One-time, per-search precompute: BFS distance (in CHUNK hops, chunk size
 * CHUNK = 4) over pure air connectivity, seeded at the search's start
 * position. "Pure air" means only BlockType.isSolid() is checked -- no
 * walkability rules (floor-below, head-clearance, legal-action checks).
 * This is deliberately the cheapest possible connectivity signal: it is
 * NOT "can an agent legally walk here", it's "is there an unbroken chain
 * of non-solid voxels from start to here, at chunk granularity".
 *
 * PURPOSE: MINE profiling on a real region (k2_r_0_0) showed MINE offers
 * ~2.39M candidates per search but only ~11.3% ever improve a gScore --
 * most MINE attempts are wasted because they tunnel toward space that's
 * already open and reachable some other way (SPRINT/CLIMB/etc. already
 * cover it), rather than toward a genuinely separate, unopened air pocket.
 * This field lets MINE's cost be adjusted downward specifically when the
 * dig target is BOTH (a) chunk-BFS-far from wherever the player currently
 * is -- a cheap proxy for "probably a different pocket, not just adjacent
 * corridor" -- AND (b) Euclidean-close (a thin wall, not a long tunnel).
 * See AirPotential.combinedPotential() for the actual formula; this class
 * only owns the precomputed field and the lookup.
 *
 * Chunk coarsening (CHUNK=4, i.e. 4x4x4 blocks per chunk) keeps both the
 * BFS itself and its memory footprint cheap: a 368x128x368 region is
 * ~17.4M voxels but only ~272K chunks.
 */
public final class AirPotentialField {
    public static final int CHUNK = 4;
    public static final int UNREACHED = -1;

    private final int chunksX, chunksY, chunksZ;
    private final int[] chunkBfsDist; // flat, C-order like World; UNREACHED if never touched by the BFS

    private AirPotentialField(int chunksX, int chunksY, int chunksZ, int[] chunkBfsDist) {
        this.chunksX = chunksX;
        this.chunksY = chunksY;
        this.chunksZ = chunksZ;
        this.chunkBfsDist = chunkBfsDist;
    }

    /** Chunk-hop BFS distance for the chunk containing voxel (x,y,z), or UNREACHED. */
    public int bfsDistAt(int x, int y, int z) {
        int cx = Math.floorDiv(x, CHUNK);
        int cy = Math.floorDiv(y, CHUNK);
        int cz = Math.floorDiv(z, CHUNK);
        if (cx < 0 || cx >= chunksX || cy < 0 || cy >= chunksY || cz < 0 || cz >= chunksZ) {
            return UNREACHED;
        }
        return chunkBfsDist[(cx * chunksY + cy) * chunksZ + cz];
    }

    /**
     * Builds the field via chunk-level BFS: two chunks are adjacent iff at
     * least one pair of touching-face voxels between them is non-solid on
     * both sides (a cheap over-approximation -- doesn't require a full
     * shared open face, just one open voxel-to-voxel connection, since this
     * only needs to be a rough "different pocket" signal, not exact).
     *
     * To keep this genuinely cheap, adjacency is approximated from CHUNK
     * OCCUPANCY only: a chunk is "open" if it contains at least one
     * non-solid voxel, and two face-adjacent open chunks are considered
     * BFS-connected. This is coarser than checking the actual boundary
     * voxels, but avoids an O(chunk_face_area) check per chunk-pair --
     * a reasonable approximation for a heuristic, not correctness-critical.
     *
     * chunkContainsAir is computed LAZILY (on first visit by the BFS
     * below), not eagerly for every chunk in the snapshot up front: for a
     * real search the snapshot is sized for its full search-space margin,
     * but the reachable-by-air-BFS region from one start point is
     * typically a small fraction of that -- profiling on k2_r_0_0 (a
     * 368x128x368 snapshot, ~271k chunks) found the eager whole-grid scan
     * was the single biggest hotspot in the entire solve (~25% of total
     * time) purely from calling chunkContainsAir on ~271k chunks the BFS
     * often never even reaches, each costing up to CHUNK^3=64 voxelAt
     * calls. Same final dist[] result either way -- this only changes
     * WHEN each chunk's air-occupancy gets computed, not what the BFS
     * visits or in what order.
     */
    public static AirPotentialField build(World world, int startX, int startY, int startZ) {
        int chunksX = ceilDiv(world.sizeX, CHUNK);
        int chunksY = ceilDiv(world.sizeY, CHUNK);
        int chunksZ = ceilDiv(world.sizeZ, CHUNK);
        int numChunks = chunksX * chunksY * chunksZ;

        // 0 = not yet computed, 1 = has air, 2 = solid/no air.
        byte[] chunkHasAirState = new byte[numChunks];

        int[] dist = new int[numChunks];
        java.util.Arrays.fill(dist, UNREACHED);

        int startCx = Math.floorDiv(startX, CHUNK);
        int startCy = Math.floorDiv(startY, CHUNK);
        int startCz = Math.floorDiv(startZ, CHUNK);
        if (startCx < 0 || startCx >= chunksX || startCy < 0 || startCy >= chunksY || startCz < 0 || startCz >= chunksZ) {
            // start itself out of bounds (shouldn't normally happen) -- return an all-UNREACHED field
            return new AirPotentialField(chunksX, chunksY, chunksZ, dist);
        }

        java.util.ArrayDeque<int[]> queue = new java.util.ArrayDeque<>();
        int startIdx = (startCx * chunksY + startCy) * chunksZ + startCz;
        if (hasAirLazy(world, chunkHasAirState, startCx, startCy, startCz, startIdx)) {
            dist[startIdx] = 0;
            queue.add(new int[]{startCx, startCy, startCz});
        }

        int[][] neighbors6 = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};

        while (!queue.isEmpty()) {
            int[] c = queue.poll();
            int cx = c[0], cy = c[1], cz = c[2];
            int curIdx = (cx * chunksY + cy) * chunksZ + cz;
            int curDist = dist[curIdx];

            for (int[] n : neighbors6) {
                int nx = cx + n[0], ny = cy + n[1], nz = cz + n[2];
                if (nx < 0 || nx >= chunksX || ny < 0 || ny >= chunksY || nz < 0 || nz >= chunksZ) {
                    continue;
                }
                int nIdx = (nx * chunksY + ny) * chunksZ + nz;
                if (dist[nIdx] != UNREACHED) {
                    continue;
                }
                if (!hasAirLazy(world, chunkHasAirState, nx, ny, nz, nIdx)) {
                    continue;
                }
                dist[nIdx] = curDist + 1;
                queue.add(new int[]{nx, ny, nz});
            }
        }

        return new AirPotentialField(chunksX, chunksY, chunksZ, dist);
    }

    private static boolean hasAirLazy(World world, byte[] state, int cx, int cy, int cz, int idx) {
        byte s = state[idx];
        if (s == 0) {
            s = chunkContainsAir(world, cx, cy, cz) ? (byte) 1 : (byte) 2;
            state[idx] = s;
        }
        return s == 1;
    }

    private static boolean chunkContainsAir(World world, int cx, int cy, int cz) {
        int x0 = cx * CHUNK, y0 = cy * CHUNK, z0 = cz * CHUNK;
        int x1 = Math.min(x0 + CHUNK, world.sizeX);
        int y1 = Math.min(y0 + CHUNK, world.sizeY);
        int z1 = Math.min(z0 + CHUNK, world.sizeZ);
        for (int x = x0; x < x1; x++) {
            for (int y = y0; y < y1; y++) {
                for (int z = z0; z < z1; z++) {
                    if (!BlockType.isSolid(world.voxelAt(x, y, z))) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    private static int ceilDiv(int a, int b) {
        return (a + b - 1) / b;
    }
}
