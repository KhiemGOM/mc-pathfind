import dev.mcpathfind.core.EdgeRules;
import dev.mcpathfind.core.PathValidator;
import dev.mcpathfind.core.SearchResult;
import dev.mcpathfind.core.StateCodec;
import dev.mcpathfind.core.WeightedAStar;
import dev.mcpathfind.core.io.WorldBinFormat;

import java.nio.file.Path;
import java.util.Locale;

/** Light sweep of NEAREST_AIR_PRUNE_RADIUS to pick a reasonable value before the real A/B run -- not deep tuning, just avoiding an obviously bad default. */
public class NearestAirThresholdPick {
    public static void main(String[] args) throws Exception {
        String[] regions = {"benchmark/data/k1_r_0_neg1.wbin", "benchmark/data/long1_r_0_0.wbin"};
        int[] radii = {1, 2, 3, 4, 6};
        for (String region : regions) {
            WorldBinFormat.Loaded loaded = WorldBinFormat.load(Path.of(region));
            var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
            var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
            for (int radius : radii) {
                EdgeRules.NEAREST_AIR_PRUNE_RADIUS = radius;
                WeightedAStar.minePruneMode = WeightedAStar.MinePruneMode.NEAREST_AIR;
                long t0 = System.nanoTime();
                SearchResult r = new WeightedAStar().search(loaded.world(), start, goal, loaded.blocksAvailable(), 1.5, 1.0, 6_000_000);
                double ms = (System.nanoTime() - t0) / 1e6;
                String v = "-";
                if (r.found()) {
                    v = PathValidator.validate(loaded.world(), r.path(), r.actions()).isEmpty() ? "OK" : "VIOLATION";
                }
                System.out.printf(Locale.ROOT, "%-25s radius=%d  found=%b cost=%.4f expansions=%d ms=%.1f validator=%s%n",
                        region, radius, r.found(), r.found() ? r.totalCost() : -1.0, r.expansions(), ms, v);
            }
            System.out.println();
        }
    }
}
