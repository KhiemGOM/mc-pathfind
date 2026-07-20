import dev.mcpathfind.pearl.*;
import dev.mcpathfind.pearl.io.WorldBinFormat;
import java.nio.file.Path;

/**
 * Loops the pearl-augmented solve many times against a real region so a
 * sampling profiler (JFR) gets enough duration/samples to find real
 * hotspots -- mirrors ProfileK2LoopHarness's role for the plain walking
 * solver. A single pearl solve is only ~0.5s, too short for reliable
 * sampling on its own.
 */
public class PearlProfileLoopHarness {
    public static void main(String[] args) throws Exception {
        Path wbin = Path.of(args[0]);
        int iterations = args.length > 1 ? Integer.parseInt(args[1]) : 200;
        String mode = args.length > 2 ? args[2] : "forward";

        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), false);

        System.out.println("Running " + iterations + " iterations, mode=" + mode);
        long t0 = System.nanoTime();
        int totalExpansions = 0;
        for (int i = 0; i < iterations; i++) {
            SearchResult r = switch (mode) {
                case "bidirectional" -> new BidirectionalWeightedAStar().search(world, start, goal, 1.0, 1.0, 2_000_000);
                case "parallel" -> new ParallelBidirectionalWeightedAStar().search(world, start, goal, 1.0, 1.0, 2_000_000);
                default -> new WeightedAStar().search(world, start, goal, 1.0, 1.0, 2_000_000);
            };
            totalExpansions += r.expansions();
            if (!r.found()) throw new RuntimeException("no path found on iteration " + i);
        }
        double ms = (System.nanoTime() - t0) / 1e6;
        System.out.printf("done: %d iterations, %.1fms total, %d total expansions, %.0f exp/s avg%n",
                iterations, ms, totalExpansions, totalExpansions / (ms / 1000.0));
    }
}
