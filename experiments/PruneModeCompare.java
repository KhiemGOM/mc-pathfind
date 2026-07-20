import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;

public class PruneModeCompare {
    public static void main(String[] args) throws Exception {
        Path wbin = Path.of(args[0]);
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
        int blocks = loaded.blocksAvailable();

        for (WeightedAStar.MinePruneMode mode : WeightedAStar.MinePruneMode.values()) {
            WeightedAStar.minePruneMode = mode;
            // warmup
            new WeightedAStar().search(world, start, goal, blocks, 1.5, 1.0, 3_000_000);
            // timed, averaged over a few iterations
            int iters = 5;
            long t0 = System.nanoTime();
            SearchResult r = null;
            int totalExp = 0;
            for (int i = 0; i < iters; i++) {
                r = new WeightedAStar().search(world, start, goal, blocks, 1.5, 1.0, 3_000_000);
                totalExp += r.expansions();
            }
            double ms = (System.nanoTime() - t0) / 1e6;
            System.out.printf("%-15s found=%-6s expansions=%-8d avg_ms=%-8.1f cost=%.2f nodes=%d%n",
                    mode, r.found(), r.expansions(), ms/iters, r.totalCost(), r.path() != null ? r.path().length : -1);
            java.util.Map<Action, Integer> counts = new java.util.EnumMap<>(Action.class);
            if (r.actions() != null) {
                for (Action a : r.actions()) counts.merge(a, 1, Integer::sum);
            }
            System.out.println("  action counts: " + counts);
        }
    }
}
