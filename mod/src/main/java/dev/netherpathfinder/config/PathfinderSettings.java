package dev.netherpathfinder.config;

import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Properties;

/** Persistent integration settings. Engine tuning is applied only between searches. */
public final class PathfinderSettings {
    public record Rule(double initial, double min, double max, boolean integer) {}
    public static final Map<String, Rule> RULES;
    static {
        var rules = new LinkedHashMap<String, Rule>();
        rules.put("searchWeight", new Rule(1.6, 1, 5, false));
        rules.put("timeoutSeconds", new Rule(5, 0.1, 60, false));
        rules.put("maxExpansions", new Rule(500_000, 1000, 2_000_000, true));
        rules.put("snapshotRadius", new Rule(64, 32, 128, true));
        rules.put("sprintSpeed", new Rule(5.6, 0.1, 20, false));
        rules.put("speedBridgeSpeed", new Rule(3.2, 0.1, 20, false));
        rules.put("placeTime", new Rule(0, 0, 10, false));
        // Nether demo (auto-plan on Nether arrival): retry ladder + assumed loadout.
        rules.put("demoAutoRun", new Rule(1, 0, 1, true));
        rules.put("ladderStartWeight", new Rule(1.6, 1, 5, false));
        rules.put("demoMaxExpansions", new Rule(3_000_000, 100_000, 20_000_000, true));
        rules.put("demoTimeoutSeconds", new Rule(900, 5, 7200, false));
        rules.put("demoBridgeBlocks", new Rule(32, 0, 255, true));
        rules.put("demoPickaxeTier", new Rule(4, 0, 6, true)); // NONE,WOODEN,GOLDEN,STONE,IRON,DIAMOND,NETHERITE
        // Route icons only draw between these distances (blocks) from the camera.
        rules.put("iconMinDistance", new Rule(2.5, 0, 32, false));
        rules.put("iconMaxDistance", new Rule(16, 4, 256, false));
        rules.put("iconThroughWalls", new Rule(1, 0, 1, true)); // 1 = icons draw through terrain
        rules.put("routeThroughWalls", new Rule(1, 0, 2, true)); // 0 = normal depth; 1 = pieces are see-through only while falling in; 2 = always see-through
        RULES = java.util.Collections.unmodifiableMap(rules);
    }

    private final Path file;
    private final Map<String, Double> values = new LinkedHashMap<>();

    public PathfinderSettings(Path file) throws IOException {
        this.file = file;
        RULES.forEach((key, rule) -> values.put(key, rule.initial()));
        if (Files.isRegularFile(file)) {
            Properties saved = new Properties();
            try (Reader reader = Files.newBufferedReader(file)) { saved.load(reader); }
            for (String key : RULES.keySet()) {
                if (saved.containsKey(key)) {
                    try { values.put(key, validate(key, saved.getProperty(key))); }
                    catch (IllegalArgumentException ignored) { /* Invalid saved entry uses its default. */ }
                }
            }
        }
    }

    public static double validate(String key, String raw) {
        Rule rule = RULES.get(key);
        if (rule == null) throw new IllegalArgumentException("Unknown setting: " + key);
        final double value;
        try { value = Double.parseDouble(raw); }
        catch (NumberFormatException e) { throw new IllegalArgumentException("Enter a number for " + key); }
        if (!Double.isFinite(value) || value < rule.min() || value > rule.max()
                || (rule.integer() && value != Math.rint(value))) {
            throw new IllegalArgumentException(key + " must be " + (rule.integer() ? "an integer " : "")
                + "between " + rule.min() + " and " + rule.max());
        }
        return value;
    }

    public void set(String key, String raw) throws IOException {
        double value = validate(key, raw);
        var next = new LinkedHashMap<>(values);
        next.put(key, value);
        Files.createDirectories(file.toAbsolutePath().getParent());
        Path temporary = file.resolveSibling(file.getFileName() + ".tmp");
        Properties saved = new Properties();
        next.forEach((name, number) -> saved.setProperty(name, number.toString()));
        try (Writer writer = Files.newBufferedWriter(temporary)) {
            saved.store(writer, "Nether Pathfinder settings; also editable with /pathfind config");
        }
        Files.move(temporary, file, StandardCopyOption.REPLACE_EXISTING);
        values.clear();
        values.putAll(next);
    }

    public double get(String key) { return values.get(key); }
    public String describe() { return values.toString(); }
    public Snapshot snapshot() {
        return new Snapshot(get("searchWeight"), get("timeoutSeconds"), (int) get("maxExpansions"),
            (int) get("snapshotRadius"), get("sprintSpeed"), get("speedBridgeSpeed"), get("placeTime"),
            get("demoAutoRun") >= 1, get("ladderStartWeight"), (int) get("demoMaxExpansions"),
            get("demoTimeoutSeconds"), (int) get("demoBridgeBlocks"), (int) get("demoPickaxeTier"));
    }

    public record Snapshot(double weight, double timeoutSeconds, int maxExpansions, int radius,
                           double sprintSpeed, double bridgeSpeed, double placeTime,
                           boolean demoAutoRun, double ladderStartWeight, int demoMaxExpansions,
                           double demoTimeoutSeconds, int demoBridgeBlocks, int demoPickaxeTier) {}
}
