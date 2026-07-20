package dev.mcpathfind.bench;

import dev.mcpathfind.core.Action;
import dev.mcpathfind.core.BidirectionalWeightedAStar;
import dev.mcpathfind.core.ParallelBidirectionalWeightedAStar;
import dev.mcpathfind.core.PathValidator;
import dev.mcpathfind.core.SearchResult;
import dev.mcpathfind.core.StateCodec;
import dev.mcpathfind.core.WeightedAStar;
import dev.mcpathfind.core.World;
import dev.mcpathfind.core.io.WorldBinFormat;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

/**
 * Loads .wbin world(s) and times WeightedAStar.search only (world load is
 * excluded), mirroring solve_with_epsilon_ladder's stop-at-first-success
 * pattern from build_visualization.py.
 *
 * Java's throughput (~600k-1M expansions/sec measured, vs Python's
 * ~1,100-2,500/sec) affords a much deeper default ladder than the Python
 * side ever could -- starting near-optimal (eps=1.0) with multi-million
 * expansion budgets is now routinely a few seconds, not tens of minutes.
 *
 * Usage:
 *   BenchmarkCli <world.wbin> [--start x,y,z] [--goal x,y,z] [--blocks N]
 *                [--epsilons ...] [--max-expansions ...] [--warmup]
 *   BenchmarkCli <directory>   -- batch mode: every *.wbin in the directory,
 *                header-embedded start/goal/blocks (no per-file overrides),
 *                prints a summary table.
 */
public final class BenchmarkCli {

    private static final double[] DEFAULT_EPSILONS = {1.0, 1.5, 2.5, 4.0};
    private static final int[] DEFAULT_BUDGETS = {2_000_000, 3_000_000, 5_000_000, 8_000_000};

    private record RunOutcome(String label, boolean found, double epsilonUsed, int pathNodes,
                               double simCost, int expansions, double succeedMs, double totalMs,
                               int violations) {}

    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            System.err.println("usage: BenchmarkCli <world.wbin | directory> [--start x,y,z] [--goal x,y,z] "
                    + "[--blocks N] [--epsilons ...] [--max-expansions ...] [--warmup]");
            System.exit(2);
        }
        Path target = Path.of(args[0]);

        StateCodec.State startOverride = null, goalOverride = null;
        Integer blocksOverride = null;
        double[] epsilons = DEFAULT_EPSILONS;
        int[] budgets = DEFAULT_BUDGETS;
        boolean warmup = false;
        String solverMode = "forward";

        for (int i = 1; i < args.length; i++) {
            switch (args[i]) {
                case "--start" -> startOverride = parseState(args[++i]);
                case "--goal" -> goalOverride = parseState(args[++i]);
                case "--blocks" -> blocksOverride = Integer.parseInt(args[++i]);
                case "--epsilons" -> epsilons = parseDoubles(args[++i]);
                case "--max-expansions" -> budgets = parseInts(args[++i]);
                case "--warmup" -> warmup = true;
                case "--bidirectional" -> solverMode = "bidirectional";
                case "--parallel" -> solverMode = "parallel";
                default -> throw new IllegalArgumentException("unknown arg: " + args[i]);
            }
        }
        if (epsilons.length != budgets.length) {
            throw new IllegalArgumentException("--epsilons and --max-expansions must have the same length");
        }

        if (Files.isDirectory(target)) {
            if (startOverride != null || goalOverride != null || blocksOverride != null) {
                throw new IllegalArgumentException("--start/--goal/--blocks aren't supported in directory batch mode "
                        + "(each file uses its own header-embedded values)");
            }
            runBatch(target, epsilons, budgets, warmup, solverMode);
        } else {
            runOne("(single)", target, startOverride, goalOverride, blocksOverride, epsilons, budgets, warmup, true, solverMode);
        }
    }

    private static void runBatch(Path dir, double[] epsilons, int[] budgets, boolean warmup, String solverMode) throws IOException {
        List<Path> files;
        try (Stream<Path> s = Files.list(dir)) {
            files = s.filter(p -> p.toString().endsWith(".wbin")).sorted().toList();
        }
        if (files.isEmpty()) {
            System.out.println("no .wbin files found in " + dir);
            return;
        }

        List<RunOutcome> outcomes = new ArrayList<>();
        for (Path f : files) {
            String label = f.getFileName().toString();
            System.out.println("\n=== " + label + " ===");
            RunOutcome o = runOne(label, f, null, null, null, epsilons, budgets, warmup, true, solverMode);
            outcomes.add(o);
        }

        // succeed_ms = just the epsilon rung that found a path; total_ms = every
        // rung tried including earlier failed (looser-budget) attempts, per request.
        System.out.println("\n=== summary (" + outcomes.size() + " regions) ===");
        System.out.printf("%-28s %-6s %-5s %-8s %-10s %-10s %-12s %-12s %s%n",
                "region", "found", "eps", "nodes", "sim_cost", "expansions", "succeed_ms", "total_ms", "validator");
        for (RunOutcome o : outcomes) {
            if (o.found()) {
                System.out.printf("%-28s %-6s %-5s %-8d %-10.2f %-10d %-12.1f %-12.1f %s%n",
                        o.label(), "yes", o.epsilonUsed(), o.pathNodes(), o.simCost(), o.expansions(),
                        o.succeedMs(), o.totalMs(), o.violations() == 0 ? "OK" : (o.violations() + " VIOLATIONS"));
            } else {
                System.out.printf("%-28s %-6s %-5s %-8s %-10s %-10d %-12s %-12.1f %s%n",
                        o.label(), "NO", "-", "-", "-", o.expansions(), "-", o.totalMs(), "-");
            }
        }
    }

    private static RunOutcome runOne(String label, Path wbinPath, StateCodec.State startOverride,
                                      StateCodec.State goalOverride, Integer blocksOverride,
                                      double[] epsilons, int[] budgets, boolean warmup, boolean report,
                                      String solverMode) throws IOException {
        long tLoad0 = System.nanoTime();
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbinPath);
        double loadMs = (System.nanoTime() - tLoad0) / 1e6;
        World world = loaded.world();
        if (report) {
            System.out.printf("loaded in %.1f ms: shape=%dx%dx%d%n", loadMs, world.sizeX, world.sizeY, world.sizeZ);
        }

        StateCodec.State start = startOverride != null ? startOverride : fromCoord(loaded.start());
        StateCodec.State goal = goalOverride != null ? goalOverride : fromCoord(loaded.goal());
        int blocksAvailable = blocksOverride != null ? blocksOverride : loaded.blocksAvailable();
        if (start == null || goal == null) {
            throw new IllegalArgumentException(wbinPath + ": no start/goal in header and none given via --start/--goal");
        }
        if (report) {
            System.out.println("start=" + start + " goal=" + goal + " blocksAvailable=" + blocksAvailable
                    + " solver=" + solverMode);
        }

        if (warmup) {
            if (report) System.out.println("--- warmup (discarded) ---");
            runLadder(world, start, goal, blocksAvailable, epsilons, budgets, false, "", solverMode);
        }

        if (report) System.out.println("--- timed run ---");
        return runLadder(world, start, goal, blocksAvailable, epsilons, budgets, report, label, solverMode);
    }

    private static RunOutcome runLadder(World world, StateCodec.State start, StateCodec.State goal,
                                         int blocksAvailable, double[] epsilons, int[] budgets, boolean report,
                                         String label, String solverMode) {
        SearchResult result = null;
        double usedEpsilon = Double.NaN;
        double succeedMs = 0;   // time of just the attempt that found a path
        double totalMs = 0;     // time across every attempt, including earlier failed rungs
        int lastExpansions = 0;
        int totalExpansions = 0;
        int attemptsMade = 0;

        for (int i = 0; i < epsilons.length; i++) {
            long t0 = System.nanoTime();
            SearchResult r = switch (solverMode) {
                case "bidirectional" -> new BidirectionalWeightedAStar().search(world, start, goal, blocksAvailable, epsilons[i], 1.0, budgets[i]);
                case "parallel" -> new ParallelBidirectionalWeightedAStar().search(world, start, goal, blocksAvailable, epsilons[i], 1.0, budgets[i]);
                default -> new WeightedAStar().search(world, start, goal, blocksAvailable, epsilons[i], 1.0, budgets[i]);
            };
            double ms = (System.nanoTime() - t0) / 1e6;
            lastExpansions = r.expansions();
            totalMs += ms;
            totalExpansions += r.expansions();
            attemptsMade++;
            if (report) {
                System.out.printf("  eps=%-5s budget=%-9d solve_time=%9.3fms  expansions=%-9d found=%-5s  rate=%.0f exp/s%n",
                        epsilons[i], budgets[i], ms, r.expansions(), r.found(), r.expansions() / (ms / 1000.0));
            }
            if (r.found()) {
                result = r;
                usedEpsilon = epsilons[i];
                succeedMs = ms;
                break;
            }
        }

        if (result == null) {
            if (report) System.out.println("NO PATH FOUND at any epsilon in the ladder");
            return new RunOutcome(label, false, Double.NaN, 0, 0, lastExpansions, 0, totalMs, -1);
        }

        if (report) {
            System.out.printf("path found: eps=%s nodes=%d cost=%.2fs(sim) expansions=%d "
                            + "succeed_search_time=%.1fms (%.0f exp/s)  total_search_time=%.1fms (across %d attempt(s), %d expansions)%n",
                    usedEpsilon, result.path().length, result.totalCost(), result.expansions(), succeedMs,
                    result.expansions() / (succeedMs / 1000.0), totalMs, attemptsMade, totalExpansions);
        }

        List<PathValidator.Violation> violations = PathValidator.validate(world, result.path(), result.actions());
        if (report) {
            if (violations.isEmpty()) {
                System.out.println("PathValidator: OK (" + (result.path().length - 1) + " steps independently re-checked)");
            } else {
                System.out.println("PathValidator: " + violations.size() + " VIOLATION(S):");
                for (var v : violations) {
                    System.out.println("  step " + v.stepIndex() + ": " + v.reason());
                }
            }

            java.util.Map<Action, Integer> counts = new java.util.EnumMap<>(Action.class);
            for (Action a : result.actions()) {
                counts.merge(a, 1, Integer::sum);
            }
            System.out.println("action counts: " + counts);
        }

        return new RunOutcome(label, true, usedEpsilon, result.path().length, result.totalCost(),
                result.expansions(), succeedMs, totalMs, violations.size());
    }

    private static StateCodec.State fromCoord(WorldBinFormat.Coord c) {
        return c == null ? null : new StateCodec.State(c.x(), c.y(), c.z(), 0, false);
    }

    private static StateCodec.State parseState(String s) {
        String[] parts = s.split(",");
        return new StateCodec.State(Integer.parseInt(parts[0]), Integer.parseInt(parts[1]), Integer.parseInt(parts[2]), 0, false);
    }

    private static double[] parseDoubles(String s) {
        String[] parts = s.split(",");
        double[] out = new double[parts.length];
        for (int i = 0; i < parts.length; i++) out[i] = Double.parseDouble(parts[i]);
        return out;
    }

    private static int[] parseInts(String s) {
        String[] parts = s.split(",");
        int[] out = new int[parts.length];
        for (int i = 0; i < parts.length; i++) out[i] = Integer.parseInt(parts[i]);
        return out;
    }
}
