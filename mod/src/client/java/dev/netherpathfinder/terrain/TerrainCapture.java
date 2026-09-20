package dev.netherpathfinder.terrain;

import dev.netherpathfinder.engine.BlockType;
import dev.netherpathfinder.engine.World;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.ChunkPos;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.chunk.LevelChunk;

import java.util.Map;

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

    /** One loaded chunk's block reads; null from BlockSource.chunk means "not loaded" (captured as UNKNOWN). */
    public interface ChunkView {
        BlockState state(BlockPos pos);
    }

    /** Where blocks come from: the client's loaded chunks, or chunks the integrated server generated. */
    public interface BlockSource {
        int minY();
        int maxY();
        ChunkView chunk(int chunkX, int chunkZ);
    }

    public static BlockSource of(ClientLevel level) {
        return new BlockSource() {
            public int minY() { return level.getMinY(); }
            public int maxY() { return level.getMaxY(); }
            public ChunkView chunk(int cx, int cz) {
                return level.hasChunk(cx, cz) ? level::getBlockState : null;
            }
        };
    }

    /** Fully generated server chunks held by the caller; reads never touch the server thread. */
    public static BlockSource of(Map<Long, LevelChunk> chunks, int minY, int maxY) {
        return new BlockSource() {
            public int minY() { return minY; }
            public int maxY() { return maxY; }
            public ChunkView chunk(int cx, int cz) {
                LevelChunk chunk = chunks.get(ChunkPos.pack(cx, cz));
                return chunk == null ? null : chunk::getBlockState;
            }
        };
    }

    /** Captures a column spanning the level's full build height, horizontalRadius blocks out on X/Z. */
    public static Result capture(ClientLevel level, BlockPos centerPos, int horizontalRadius) {
        return capture(of(level), centerPos, horizontalRadius);
    }

    public static Result capture(BlockSource source, BlockPos centerPos, int horizontalRadius) {
        int fullVerticalRadius = source.maxY() - source.minY();
        return capture(source, centerPos, horizontalRadius, fullVerticalRadius);
    }

    /**
     * Captures a box centered on centerPos: horizontalRadius blocks out on
     * X/Z, verticalRadius blocks out on Y (clamped to the level's real
     * build-height range).
     */
    public static Result capture(ClientLevel level, BlockPos centerPos, int horizontalRadius, int verticalRadius) {
        return capture(of(level), centerPos, horizontalRadius, verticalRadius);
    }

    public static Result capture(BlockSource source, BlockPos centerPos, int horizontalRadius, int verticalRadius) {
        int minY = source.minY();
        int maxYExclusive = source.maxY();

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
                ChunkView view = source.chunk(chunkX, chunkZ);
                for (int ly = 0; ly < sizeY; ly++) {
                    int worldY = originY + ly;
                    int index = (lx * sizeY + ly) * sizeZ + lz;
                    if (view == null) {
                        blocks[index] = (byte) BlockType.UNKNOWN;
                        continue;
                    }
                    cursor.set(worldX, worldY, worldZ);
                    BlockState state = view.state(cursor);
                    blocks[index] = (byte) BlockClassifier.classify(state);
                }
            }
        }

        World world = new World(sizeX, sizeY, sizeZ, blocks);
        return new Result(world, originX, originY, originZ);
    }
}
