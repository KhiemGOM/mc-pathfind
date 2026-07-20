import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;

public class BridgeDepthSweep {
    public static void main(String[] args) throws Exception {
        Path wbin = Path.of("benchmark/data/k1_r_neg1_0.wbin");
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
        int blocks = loaded.blocksAvailable();
        WeightedAStar.minePruneMode = WeightedAStar.MinePruneMode.AIR_POTENTIAL;

        int[] depths = {0, 1, 2, 3, 4, 8};
        for (int d : depths) {
            EdgeRules.MAX_REVERSE_BRIDGE_DEPTH = d;
            long t0 = System.nanoTime();
            SearchResult r = new BidirectionalWeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
            double ms = (System.nanoTime() - t0) / 1e6;
            System.out.printf("depth=%-3d found=%-6s ms=%-9.1f expansions=%-9d cost=%.2f%n",
                    d, r.found(), ms, r.expansions(), r.found() ? r.totalCost() : -1);
        }
    }
}
