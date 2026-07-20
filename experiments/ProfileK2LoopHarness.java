import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;

public class ProfileK2LoopHarness {
    public static void main(String[] args) throws Exception {
        Path wbin = Path.of(args[0]);
        int iterations = args.length > 1 ? Integer.parseInt(args[1]) : 8;

        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
        int blocks = loaded.blocksAvailable();

        System.out.println("Running " + iterations + " iterations at eps=1.5 (the rung that actually succeeds)");
        long t0 = System.nanoTime();
        int totalExpansions = 0;
        for (int i = 0; i < iterations; i++) {
            SearchResult r = new WeightedAStar().search(world, start, goal, blocks, 1.5, 1.0, 3_000_000);
            totalExpansions += r.expansions();
            if (!r.found()) throw new RuntimeException("no path found on iteration " + i);
        }
        double ms = (System.nanoTime() - t0) / 1e6;
        System.out.printf("done: %d iterations, %.1fms total, %d total expansions, %.0f exp/s avg%n",
                iterations, ms, totalExpansions, totalExpansions / (ms/1000.0));
    }
}
