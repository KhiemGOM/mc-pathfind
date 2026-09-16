package dev.netherpathfinder.terrain;

import dev.netherpathfinder.engine.BlockType;
import dev.netherpathfinder.engine.World;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.block.state.BlockState;

/**
 * Reads a cube of real terrain around an origin into the engine's flat
 * voxel World, for the search to run against. Positions in a chunk that
 * isn't currently loaded are stored as BlockType.UNKNOWN rather than
 * guessed at here -- World.unknownContext (set per-expansion by the
 * search itself, relative to the y of the state being expanded) resolves
 * those lazily at read time.
 *
 * Safe to run off the client thread: ClientLevel's chunk storage tolerates
 * concurrent reads, and this class never mutates the level. Only the
 * player-position/level snapshot needed to know WHERE to capture should be
 * grabbed on the client thread beforehand.
 */
public final class TerrainCapture {
    private TerrainCapture() {}

    public static final class Result {
        public final World world;
        public final int originX, originY, originZ;

        Result(World world, int originX, int originY, int originZ) {
            this.world = world;
            this.originX = originX;
            this.originY = originY;
            this.originZ = originZ;
        }

        public int toLocalX(int worldX) { return worldX - originX; }
        public int toLocalY(int worldY) { return worldY - originY; }
        public int toLocalZ(int worldZ) { return worldZ - originZ; }
        public int toWorldX(int localX) { return localX + originX; }
        public int toWorldY(int localY) { return localY + originY; }
        public int toWorldZ(int localZ) { return localZ + originZ; }
    }

    /** Captures a column spanning the level's full build height, horizontalRadius blocks out on X/Z. */
    public static Result capture(ClientLevel level, BlockPos centerPos, int horizontalRadius) {
        int fullVerticalRadius = level.getMaxY() - level.getMinY();
        return capture(level, centerPos, horizontalRadius, fullVerticalRadius);
    }

    /**
     * Captures a box centered on centerPos: horizontalRadius blocks out on
     * X/Z, verticalRadius blocks out on Y (clamped to the level's real
     * build-height range).
     */
    public static Result capture(ClientLevel level, BlockPos centerPos, int horizontalRadius, int verticalRadius) {
        int minY = level.getMinY();
        int maxYExclusive = level.getMaxY();

        int originX = centerPos.getX() - horizontalRadius;
        int originZ = centerPos.getZ() - horizontalRadius;
        int originY = Math.max(minY, centerPos.getY() - verticalRadius);
        int endYExclusive = Math.min(maxYExclusive, centerPos.getY() + verticalRadius + 1);

        int sizeX = horizontalRadius * 2 + 1;
        int sizeZ = horizontalRadius * 2 + 1;
        int sizeY = Math.max(1, endYExclusive - originY);

        byte[] blocks = new byte[sizeX * sizeY * sizeZ];
        BlockPos.MutableBlockPos cursor = new BlockPos.MutableBlockPos();

        for (int lx = 0; lx < sizeX; lx++) {
            int worldX = originX + lx;
            int chunkX = worldX >> 4;
            for (int lz = 0; lz < sizeZ; lz++) {
                int worldZ = originZ + lz;
                int chunkZ = worldZ >> 4;
                boolean chunkLoaded = level.hasChunk(chunkX, chunkZ);
                for (int ly = 0; ly < sizeY; ly++) {
                    int worldY = originY + ly;
                    int index = (lx * sizeY + ly) * sizeZ + lz;
                    if (!chunkLoaded) {
                        blocks[index] = (byte) BlockType.UNKNOWN;
                        continue;
                    }
                    cursor.set(worldX, worldY, worldZ);
                    BlockState state = level.getBlockState(cursor);
                    blocks[index] = (byte) BlockClassifier.classify(state);
                }
            }
        }

        World world = new World(sizeX, sizeY, sizeZ, blocks);
        return new Result(world, originX, originY, originZ);
    }
}
