package dev.mcpathfind.core.io;

import dev.mcpathfind.core.World;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Loader for the flat binary world-export format written by
 * export_world_bin.py (repo root, Python side). Fixed 60-byte little-endian
 * header + raw int8 voxel bytes in the same C-order layout World uses, so
 * loading is a single readAllBytes + array copy, no reshaping.
 *
 * Layout:
 *   0  "MCPW" magic (4 bytes)
 *   4  int32 version (=1)
 *   8  int32 sizeX
 *  12  int32 sizeY
 *  16  int32 sizeZ
 *  20  int32 originX      (world-coordinate origin, metadata only)
 *  24  int32 originY
 *  28  int32 originZ
 *  32  int32 startX       (-1 sentinel if not specified)
 *  36  int32 startY
 *  40  int32 startZ
 *  44  int32 goalX
 *  48  int32 goalY
 *  52  int32 goalZ
 *  56  int32 blocksAvailable
 *  60  sizeX*sizeY*sizeZ raw int8 voxel bytes, C-order: idx = (x*sizeY+y)*sizeZ+z
 */
public final class WorldBinFormat {
    private static final byte[] MAGIC = {'M', 'C', 'P', 'W'};
    private static final int VERSION = 1;
    private static final int HEADER_SIZE = 60;
    private static final int NO_COORD = -1;

    public record Coord(int x, int y, int z) {}

    public record Loaded(World world, Coord origin, Coord start, Coord goal, int blocksAvailable) {}

    public static Loaded load(Path path) throws IOException {
        byte[] all = Files.readAllBytes(path);
        if (all.length < HEADER_SIZE) {
            throw new IOException("file too short for a " + HEADER_SIZE + "-byte header: " + path);
        }
        ByteBuffer buf = ByteBuffer.wrap(all).order(ByteOrder.LITTLE_ENDIAN);

        byte[] magic = new byte[4];
        buf.get(magic);
        if (magic[0] != MAGIC[0] || magic[1] != MAGIC[1] || magic[2] != MAGIC[2] || magic[3] != MAGIC[3]) {
            throw new IOException("bad magic in " + path + ": " + new String(magic));
        }
        int version = buf.getInt();
        if (version != VERSION) {
            throw new IOException("unsupported .wbin version " + version + " in " + path);
        }
        int sizeX = buf.getInt();
        int sizeY = buf.getInt();
        int sizeZ = buf.getInt();
        int originX = buf.getInt();
        int originY = buf.getInt();
        int originZ = buf.getInt();
        int startX = buf.getInt();
        int startY = buf.getInt();
        int startZ = buf.getInt();
        int goalX = buf.getInt();
        int goalY = buf.getInt();
        int goalZ = buf.getInt();
        int blocksAvailable = buf.getInt();

        long expectedVoxels = (long) sizeX * sizeY * sizeZ;
        long actualVoxels = all.length - HEADER_SIZE;
        if (expectedVoxels != actualVoxels) {
            throw new IOException("voxel payload size mismatch in " + path + ": header says "
                    + expectedVoxels + ", file has " + actualVoxels);
        }

        byte[] voxels = new byte[(int) expectedVoxels];
        System.arraycopy(all, HEADER_SIZE, voxels, 0, voxels.length);
        World world = new World(sizeX, sizeY, sizeZ, voxels);

        Coord origin = new Coord(originX, originY, originZ);
        Coord start = (startX == NO_COORD) ? null : new Coord(startX, startY, startZ);
        Coord goal = (goalX == NO_COORD) ? null : new Coord(goalX, goalY, goalZ);
        return new Loaded(world, origin, start, goal, blocksAvailable);
    }

    private WorldBinFormat() {}
}
