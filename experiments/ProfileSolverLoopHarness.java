import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;

/** Loops one named solver on one region for dense JFR sampling. Args: wbin mode[forward|bidirectional|parallel] eps iterations */
public class ProfileSolverLoopHarness {
    public static void main(String[] args) throws Exception {
        Path wbin = Path.of(args[0]);
        String mode = args.length > 1 ? args[1] : "forward";
        double eps = args.length > 2 ? Double.parseDouble(args[2]) : 1.5;
        int iterations = args.length > 3 ? Integer.parseInt(args[3]) : 8;

        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
        int blocks = loaded.blocksAvailable();

        System.out.println("Running " + iterations + " iterations of " + mode + " at eps=" + eps);
        long t0 = System.nanoTime();
        int totalExpansions = 0;
        for (int i = 0; i < iterations; i++) {
            SearchResult r = switch (mode) {
                case "forward" -> new WeightedAStar().search(world, start, goal, blocks, eps, 1.0, 5_000_000);
                case "bidirectional" -> new BidirectionalWeightedAStar().search(world, start, goal, blocks, eps, 1.0, 5_000_000);
                case "parallel" -> new ParallelBidirectionalWeightedAStar().search(world, start, goal, blocks, eps, 1.0, 5_000_000);
                default -> throw new IllegalArgumentException("unknown mode " + mode);
            };
            totalExpansions += r.expansions();
            if (!r.found()) throw new RuntimeException("no path found on iteration " + i);
        }
        double ms = (System.nanoTime() - t0) / 1e6;
        System.out.printf("done: %d iterations, %.1fms total, %d total expansions, %.0f exp/s avg%n",
                iterations, ms, totalExpansions, totalExpansions / (ms/1000.0));
    }
}
