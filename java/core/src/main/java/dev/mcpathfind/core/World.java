package dev.mcpathfind.core;

/**
 * Flat voxel array, C-order (matches numpy's default layout for shape
 * (sizeX, sizeY, sizeZ)): index = (x*sizeY + y)*sizeZ + z. This lets
 * WorldBinFormat load an exported world with zero reshaping.
 */
public final class World {
    public final int sizeX, sizeY, sizeZ;
    public final byte[] blocks;

    public World(int sizeX, int sizeY, int sizeZ, byte[] blocks) {
        if (blocks.length != (long) sizeX * sizeY * sizeZ) {
            throw new IllegalArgumentException(
                "blocks.length=" + blocks.length + " does not match sizeX*sizeY*sizeZ=" + ((long) sizeX * sizeY * sizeZ));
        }
        this.sizeX = sizeX;
        this.sizeY = sizeY;
        this.sizeZ = sizeZ;
        this.blocks = blocks;
    }

    /** Bounds-checked lookup; returns BlockType.VOID (-1) out of bounds, matching voxel_at in world.py. */
    public int voxelAt(int x, int y, int z) {
        if (x < 0 || x >= sizeX || y < 0 || y >= sizeY || z < 0 || z >= sizeZ) {
            return BlockType.VOID;
        }
        return blocks[(x * sizeY + y) * sizeZ + z];
    }
}
