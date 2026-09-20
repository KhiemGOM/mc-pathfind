package dev.netherpathfinder.client;

import dev.netherpathfinder.NetherPathfinderMod;
import dev.netherpathfinder.engine.GoalPoints;
import dev.netherpathfinder.terrain.TerrainCapture;
import net.fabricmc.loader.api.FabricLoader;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Writes the exact terrain of a leg that failed to a .wbin file (the repo's world-export format, see
 * java/.../WorldBinFormat.java) plus a text note with the start and every goal cell, so the failure can be
 * replayed offline with the Java benchmark tools instead of guessed at.
 */
final class LegDump {
    private LegDump() {}

    private static final int MAX_DUMPS_PER_SESSION = 6;
    private static int written = 0;

    static synchronized void write(String worldKey, int leg, int retry, TerrainCapture.Result capture,
                                   int startX, int startY, int startZ, GoalPoints goal, int blocks, String note) {
        if (written >= MAX_DUMPS_PER_SESSION) return;
        written++;
        try {
            Path dir = FabricLoader.getInstance().getConfigDir().resolve("nether-pathfinder-debug");
            Files.createDirectories(dir);
            String safe = worldKey.replaceAll("[^A-Za-z0-9._-]", "_");
            String base = safe + "-leg" + leg + "-try" + retry;

            int gx = 0, gy = 0, gz = 0;
            if (goal != null) { gx = goal.x(0); gy = goal.y(0); gz = goal.z(0); }
            ByteBuffer header = ByteBuffer.allocate(60).order(ByteOrder.LITTLE_ENDIAN);
            header.put(new byte[] {'M', 'C', 'P', 'W'}).putInt(1)
                .putInt(capture.world.sizeX).putInt(capture.world.sizeY).putInt(capture.world.sizeZ)
                .putInt(capture.originX).putInt(capture.originY).putInt(capture.originZ)
                .putInt(startX).putInt(startY).putInt(startZ)
                .putInt(gx).putInt(gy).putInt(gz)
                .putInt(blocks);
            Path wbin = dir.resolve(base + ".wbin");
            try (var out = Files.newOutputStream(wbin)) {
                out.write(header.array());
                out.write(capture.world.blocks);
            }

            StringBuilder text = new StringBuilder();
            text.append(note).append('\n');
            text.append("origin (world) = ").append(capture.originX).append(' ').append(capture.originY)
                .append(' ').append(capture.originZ).append('\n');
            text.append("start (local)  = ").append(startX).append(' ').append(startY).append(' ').append(startZ).append('\n');
            text.append("blocks         = ").append(blocks).append('\n');
            if (goal != null) {
                text.append("goals (local)  = ").append(goal.size()).append('\n');
                for (int i = 0; i < goal.size(); i++) {
                    text.append("  ").append(goal.x(i)).append(' ').append(goal.y(i)).append(' ').append(goal.z(i)).append('\n');
                }
            }
            Files.writeString(dir.resolve(base + ".txt"), text.toString());
            NetherPathfinderMod.LOGGER.info("Wrote failed-leg dump {}", wbin);
        } catch (IOException e) {
            NetherPathfinderMod.LOGGER.warn("Could not write failed-leg dump", e);
        }
    }
}
