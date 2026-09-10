package dev.mcpathfind.core;

/**
 * One-time precompute for the EXPERIMENTAL HYBRID mine-prune mode: chunk-
 * coarsened (CHUNK=4, same granularity as AirPotentialField) fraction of
 * STONE-type voxels per chunk, giving O(1) lookups afterward -- the same
 * "precompute once, cheap per-candidate lookup" shape as AirPotentialField,
 * for the same reason: NearestAirDistance's per-candidate O(radius^3) scan
 * is fine to pay occasionally but not on every MINE candidate.
 *
 * DELIBERATELY STONE-SPECIFIC, NOT A GENERIC isSolid() DENSITY. Ordinary
 * Nether terrain -- netherrack floors, ceilings, cave walls, all DIRT-
 * mapped in this project's block scheme (see ../../../../../mca_convert.py's
 * block-type table) -- is locally solid almost everywhere, so a generic
 * solid-voxel density would read "high" constantly and carry no signal.
 * STONE specifically means blackstone/basalt/polished-blackstone-brick
 * family: real bastion-remnant wall material. Its density is what actually
 * distinguishes "near a structure" from "normal open terrain", which is the
 * whole point of gating on it.
 */
public final class StoneDensityField {
    public static final int CHUNK = AirPotentialField.CHUNK;

    private final int chunksX, chunksY, chunksZ;
    private final float[] chunkStoneFraction; // flat, C-order like World

    private StoneDensityField(int chunksX, int chunksY, int chunksZ, float[] chunkStoneFraction) {
        this.chunksX = chunksX;
        this.chunksY = chunksY;
        this.chunksZ = chunksZ;
        this.chunkStoneFraction = chunkStoneFraction;
    }

    /** Fraction of STONE voxels in the chunk containing (x,y,z), or 0.0 out of bounds. */
    public double densityAt(int x, int y, int z) {
        int cx = Math.floorDiv(x, CHUNK);
        int cy = Math.floorDiv(y, CHUNK);
        int cz = Math.floorDiv(z, CHUNK);
        if (cx < 0 || cx >= chunksX || cy < 0 || cy >= chunksY || cz < 0 || cz >= chunksZ) {
            return 0.0;
        }
        return chunkStoneFraction[(cx * chunksY + cy) * chunksZ + cz];
    }

    public static StoneDensityField build(World world) {
        int chunksX = ceilDiv(world.sizeX, CHUNK);
        int chunksY = ceilDiv(world.sizeY, CHUNK);
        int chunksZ = ceilDiv(world.sizeZ, CHUNK);
        int numChunks = chunksX * chunksY * chunksZ;
        float[] fraction = new float[numChunks];

        for (int cx = 0; cx < chunksX; cx++) {
            int x0 = cx * CHUNK, x1 = Math.min(x0 + CHUNK, world.sizeX);
            for (int cy = 0; cy < chunksY; cy++) {
                int y0 = cy * CHUNK, y1 = Math.min(y0 + CHUNK, world.sizeY);
                for (int cz = 0; cz < chunksZ; cz++) {
                    int z0 = cz * CHUNK, z1 = Math.min(z0 + CHUNK, world.sizeZ);
                    int total = 0, stone = 0;
                    for (int x = x0; x < x1; x++) {
                        for (int y = y0; y < y1; y++) {
                            for (int z = z0; z < z1; z++) {
                                total++;
                                if (world.voxelAt(x, y, z) == BlockType.STONE) {
                                    stone++;
                                }
                            }
                        }
                    }
                    fraction[(cx * chunksY + cy) * chunksZ + cz] = total > 0 ? (float) stone / total : 0f;
                }
            }
        }
        return new StoneDensityField(chunksX, chunksY, chunksZ, fraction);
    }

    private static int ceilDiv(int a, int b) {
        return (a + b - 1) / b;
    }
}
