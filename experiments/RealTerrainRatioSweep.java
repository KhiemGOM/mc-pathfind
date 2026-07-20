import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;
import java.nio.file.Files;
import java.util.List;

public class RealTerrainRatioSweep {
    public static void main(String[] args) throws Exception {
        double[] ratios = {1.0, 0.5, 0.0};
        List<Path> files;
        try (var s = Files.list(Path.of("benchmark/data"))) {
            files = s.filter(p -> p.toString().endsWith(".wbin")).sorted().toList();
        }
        for (double ratio : ratios) {
            EdgeRules.BRIDGE_UP_PRUNE_RATIO = ratio;
            System.out.println("\n=== ratio=" + ratio + " ===");
            for (Path f : files) {
                WorldBinFormat.Loaded loaded = WorldBinFormat.load(f);
                World world = loaded.world();
                var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
                var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
                int blocks = loaded.blocksAvailable();
                long t0 = System.nanoTime();
                SearchResult r = new BidirectionalWeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
                double ms = (System.nanoTime() - t0) / 1e6;
                System.out.printf("%-28s found=%-6s ms=%-9.1f expansions=%-9d cost=%s%n",
                        f.getFileName(), r.found(), ms, r.expansions(), r.found() ? String.format("%.2f", r.totalCost()) : "-");
            }
        }
    }
}
