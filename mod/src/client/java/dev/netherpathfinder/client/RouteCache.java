package dev.netherpathfinder.client;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import dev.netherpathfinder.NetherPathfinderMod;
import dev.netherpathfinder.engine.Action;
import net.fabricmc.loader.api.FabricLoader;

import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Per-world cache of solved Nether routes, keyed by the Nether-side portal the
 * player arrived at, so re-entering through the same portal replays the same
 * route instead of searching again. Stored as JSON under
 * config/nether-pathfinder-cache/<world folder>.json and survives restarts.
 */
final class RouteCache {
    private RouteCache() {}

    /** How close (blocks) an arrival must be to a cached portal to count as "the same portal". */
    private static final int PORTAL_MATCH_RADIUS = 6;
    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();

    static final class Entry {
        int portalX, portalY, portalZ;
        int bastionX, bastionY, bastionZ;
        double[][] points;
        String[] actions;
        double weight;
        double cost;
        int expansions;
    }

    private static final class File {
        int version = 1;
        List<Entry> entries = new ArrayList<>();
    }

    static Entry newEntry(int px, int py, int pz, int bx, int by, int bz, double[][] points, Action[] actions,
                          double weight, double cost, int expansions) {
        Entry e = new Entry();
        e.portalX = px; e.portalY = py; e.portalZ = pz;
        e.bastionX = bx; e.bastionY = by; e.bastionZ = bz;
        e.points = points;
        e.actions = new String[actions.length];
        for (int i = 0; i < actions.length; i++) e.actions[i] = actions[i].name();
        e.weight = weight;
        e.cost = cost;
        e.expansions = expansions;
        return e;
    }

    static Action[] actionsOf(Entry e) {
        Action[] out = new Action[e.actions.length];
        for (int i = 0; i < out.length; i++) {
            try { out[i] = Action.valueOf(e.actions[i]); } catch (IllegalArgumentException ex) { out[i] = Action.SPRINT; }
        }
        return out;
    }

    static synchronized Entry find(String worldKey, int x, int y, int z) {
        for (Entry e : load(worldKey).entries) {
            if (Math.abs(e.portalX - x) <= PORTAL_MATCH_RADIUS
                && Math.abs(e.portalZ - z) <= PORTAL_MATCH_RADIUS
                && Math.abs(e.portalY - y) <= PORTAL_MATCH_RADIUS
                && e.points != null && e.points.length > 0) {
                return e;
            }
        }
        return null;
    }

    static synchronized void put(String worldKey, Entry entry) {
        File file = load(worldKey);
        file.entries.removeIf(e -> Math.abs(e.portalX - entry.portalX) <= PORTAL_MATCH_RADIUS
            && Math.abs(e.portalZ - entry.portalZ) <= PORTAL_MATCH_RADIUS
            && Math.abs(e.portalY - entry.portalY) <= PORTAL_MATCH_RADIUS);
        file.entries.add(entry);
        save(worldKey, file);
    }

    /** Returns how many cached routes were removed. */
    static synchronized int clear(String worldKey) {
        File file = load(worldKey);
        int removed = file.entries.size();
        try {
            Files.deleteIfExists(pathFor(worldKey));
        } catch (IOException e) {
            NetherPathfinderMod.LOGGER.warn("Could not delete route cache for {}", worldKey, e);
        }
        return removed;
    }

    private static Path pathFor(String worldKey) {
        String safe = worldKey.replaceAll("[^A-Za-z0-9._-]", "_");
        return FabricLoader.getInstance().getConfigDir().resolve("nether-pathfinder-cache").resolve(safe + ".json");
    }

    private static File load(String worldKey) {
        Path path = pathFor(worldKey);
        if (!Files.isRegularFile(path)) return new File();
        try (Reader reader = Files.newBufferedReader(path)) {
            File file = GSON.fromJson(reader, File.class);
            return file == null || file.entries == null ? new File() : file;
        } catch (Exception e) {
            NetherPathfinderMod.LOGGER.warn("Ignoring unreadable route cache {}", path, e);
            return new File();
        }
    }

    private static void save(String worldKey, File file) {
        Path path = pathFor(worldKey);
        try {
            Files.createDirectories(path.getParent());
            Path tmp = path.resolveSibling(path.getFileName() + ".tmp");
            try (Writer writer = Files.newBufferedWriter(tmp)) { GSON.toJson(file, writer); }
            Files.move(tmp, path, java.nio.file.StandardCopyOption.REPLACE_EXISTING);
        } catch (IOException e) {
            NetherPathfinderMod.LOGGER.warn("Could not save route cache {}", path, e);
        }
    }
}
