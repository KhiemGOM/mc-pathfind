package dev.mcpathfind.pearl.bench;

import dev.mcpathfind.pearl.Action;
import dev.mcpathfind.pearl.BidirectionalWeightedAStar;
import dev.mcpathfind.pearl.ParallelBidirectionalWeightedAStar;
import dev.mcpathfind.pearl.PathValidator;
import dev.mcpathfind.pearl.SearchResult;
import dev.mcpathfind.pearl.StateCodec;
import dev.mcpathfind.pearl.WeightedAStar;
import dev.mcpathfind.pearl.World;
import dev.mcpathfind.pearl.io.WorldBinFormat;

import java.nio.file.Path;

/**
 * Ad-hoc driver for benchmarking the PEARL-augmented solver (dev.mcpathfind.pearl,
 * which folds EdgeRules.pearlEdges into ordinary walking moves) against the same
 * .wbin regions BenchmarkCli uses for the plain walking-only core solver. Reuses
 * core's WorldBinFormat-compatible header via pearl's own io copy; blocksAvailable
 * from the header is ignored since this module's action set never consumes blocks.
 */
public final class PearlBenchmarkCli {

    private static final double[] DEFAULT_EPSILONS = {1.0, 1.5, 2.5, 4.0};
    private static final int[] DEFAULT_BUDGETS = {2_000_000, 3_000_000, 5_000_000, 8_000_000};

    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            System.err.println("usage: PearlBenchmarkCli <world.wbin> [--warmup]");
            System.exit(2);
        }
        Path wbinPath = Path.of(args[0]);
        boolean warmup = false;
        for (int i = 1; i < args.length; i++) {
            if (args[i].equals("--warmup")) warmup = true;
        }

        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbinPath);
        World world = loaded.world();
        System.out.printf("loaded: shape=%dx%dx%d%n", world.sizeX, world.sizeY, world.sizeZ);

        if (loaded.start() == null || loaded.goal() == null) {
            throw new IllegalArgumentException("no start/goal in header");
        }
        StateCodec.State start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), false);
        StateCodec.State goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), false);
        System.out.println("start=" + start + " goal=" + goal);

        for (String mode : new String[]{"forward", "bidirectional", "parallel"}) {
            System.out.println("\n############ " + mode.toUpperCase() + " ############");
            if (warmup) {
                System.out.println("--- warmup (discarded) ---");
                runLadder(world, start, goal, mode, false);
            }
            System.out.println("--- timed run ---");
            runLadder(world, start, goal, mode, true);
        }
    }

    private static void runLadder(World world, StateCodec.State start, StateCodec.State goal, String mode, boolean report) {
        double totalMs = 0;
        int totalExpansions = 0;
        int attempts = 0;
        for (int i = 0; i < DEFAULT_EPSILONS.length; i++) {
            double eps = DEFAULT_EPSILONS[i];
            int budget = DEFAULT_BUDGETS[i];
            long t0 = System.nanoTime();
            SearchResult r = switch (mode) {
                case "bidirectional" -> new BidirectionalWeightedAStar().search(world, start, goal, eps, 1.0, budget);
                case "parallel" -> new ParallelBidirectionalWeightedAStar().search(world, start, goal, eps, 1.0, budget);
                default -> new WeightedAStar().search(world, start, goal, eps, 1.0, budget);
            };
            double ms = (System.nanoTime() - t0) / 1e6;
            totalMs += ms;
            totalExpansions += r.expansions();
            attempts++;
            if (report) {
                System.out.printf("  eps=%-5s budget=%-9d solve_time=%9.3fms  expansions=%-9d found=%-5s  rate=%.0f exp/s%n",
                        eps, budget, ms, r.expansions(), r.found(), r.expansions() / (ms / 1000.0));
            }
            if (r.found()) {
                if (report) {
                    java.util.Map<Action, Integer> counts = new java.util.EnumMap<>(Action.class);
                    for (Action a : r.actions()) counts.merge(a, 1, Integer::sum);
                    System.out.printf("path found: eps=%s nodes=%d cost=%.2fs(sim) expansions=%d "
                                    + "succeed_search_time=%.1fms  total_search_time=%.1fms (across %d attempt(s), %d expansions)%n",
                            eps, r.path().length, r.totalCost(), r.expansions(), ms, totalMs, attempts, totalExpansions);
                    System.out.println("action counts: " + counts);
                    java.util.List<PathValidator.Violation> violations = PathValidator.validate(world, r.path(), r.actions());
                    if (violations.isEmpty()) {
                        System.out.println("PathValidator: OK");
                    } else {
                        System.out.println("PathValidator: " + violations.size() + " VIOLATION(S):");
                        for (var v : violations) {
                            System.out.println("  step " + v.stepIndex() + ": " + v.reason());
                        }
                    }
                }
                return;
            }
        }
        if (report) System.out.println("NO PATH FOUND at any epsilon in the ladder");
    }
}
