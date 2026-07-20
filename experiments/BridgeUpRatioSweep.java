import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;

public class BridgeUpRatioSweep {
    public static void main(String[] args) throws Exception {
        String[] files = {
            "benchmark/data/lab/ledge_bedrock_world.wbin",
            "benchmark/data/lab/floating_islands_world.wbin",
        };
        double[] ratios = {1.0, 0.75, 0.5, 0.25, 0.1, 0.0};

        for (String f : files) {
            Path wbin = Path.of(f);
            WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
            World world = loaded.world();
            var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
            var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
            int blocks = loaded.blocksAvailable();

            System.out.println("\n=== " + f + " ===");
            for (double ratio : ratios) {
                EdgeRules.BRIDGE_UP_PRUNE_RATIO = ratio;
                SearchResult r = new BidirectionalWeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
                java.util.Map<Action, Integer> counts = new java.util.EnumMap<>(Action.class);
                if (r.actions() != null) {
                    for (Action a : r.actions()) counts.merge(a, 1, Integer::sum);
                }
                System.out.printf("ratio=%-5s found=%-6s expansions=%-8d cost=%-8s actions=%s%n",
                        ratio, r.found(), r.expansions(), r.found() ? String.format("%.2f", r.totalCost()) : "-", counts);
            }
        }
    }
}
