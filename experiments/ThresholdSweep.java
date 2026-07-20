import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;

/**
 * Sweeps AirPotential.OLD_GROUND_CUTOFF across BidirectionalWeightedAStar on
 * a fixed set of regions. Unlike MINE_PRUNE_THRESHOLD (which turned out to
 * be a near-binary switch -- see AirPotential's class javadoc), shifting the
 * cutoff should give an actual continuous dial between "prune everything
 * delta<=0" (cutoff=0, current default) and "prune nothing" (cutoff very
 * negative).
 */
public class ThresholdSweep {
    public static void main(String[] args) throws Exception {
        String[] files = {
            "benchmark/data/k1_r_0_0.wbin",
            "benchmark/data/k1_r_neg1_0.wbin",
            "benchmark/data/k2_r_0_0.wbin",
            "benchmark/data/long1_r_neg1_0.wbin",
        };
        int[] cutoffs = {0, -1, -2, -3, -5, -8, -12, -20};

        for (int c : cutoffs) {
            AirPotential.OLD_GROUND_CUTOFF = c;
            System.out.println("=== OLD_GROUND_CUTOFF=" + c + " ===");
            for (String f : files) {
                Path wbin = Path.of(f);
                WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
                World world = loaded.world();
                var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
                var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
                int blocks = loaded.blocksAvailable();

                // warmup
                new BidirectionalWeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
                int iters = 3;
                long t0 = System.nanoTime();
                SearchResult r = null;
                for (int i = 0; i < iters; i++) {
                    r = new BidirectionalWeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
                }
                double ms = (System.nanoTime() - t0) / 1e6 / iters;
                System.out.printf("  %-28s found=%-6s ms=%-8.1f expansions=%-8d cost=%.2f nodes=%d%n",
                        Path.of(f).getFileName(), r.found(), ms, r.expansions(), r.totalCost(),
                        r.path() != null ? r.path().length : -1);
            }
        }
    }
}
