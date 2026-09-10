import dev.mcpathfind.core.AirPotential;
import dev.mcpathfind.core.PathValidator;
import dev.mcpathfind.core.SearchResult;
import dev.mcpathfind.core.StateCodec;
import dev.mcpathfind.core.WeightedAStar;
import dev.mcpathfind.core.io.WorldBinFormat;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * A/B/C comparison of WeightedAStar.minePruneMode across every real-terrain
 * region in benchmark/data/: NONE (uninformed -- every legal MINE candidate
 * generated, the traditional baseline), MANHATTAN (prune a MINE candidate
 * unless it strictly decreases Manhattan distance to goal -- the obvious
 * greedy heuristic), AIR_POTENTIAL (the chunk-BFS connectivity heuristic).
 *
 * Two epsilon passes per region/mode: 1.5 (this project's usual practical
 * operating point -- speed/expansions comparison) and 1.0 (true-optimal --
 * optimality-GAP comparison: does the prune actually cost path quality, and
 * how much). A mode that exhausts maxExpansions without finding a path is
 * reported as DNF rather than silently skipped -- for the uninformed/
 * Manhattan baselines at eps=1.0 on real terrain, that is itself a real
 * result, not a bug.
 */
public class MineHeuristicShowdown {
    public static void main(String[] args) throws Exception {
        Path dataDir = Path.of(args.length > 0 ? args[0] : "benchmark/data");
        int maxExpansions = args.length > 1 ? Integer.parseInt(args[1]) : 6_000_000;

        List<Path> regions = new ArrayList<>();
        try (var stream = Files.list(dataDir)) {
            stream.filter(p -> p.toString().endsWith(".wbin"))
                  .sorted(Comparator.comparing(Path::toString))
                  .forEach(regions::add);
        }

        System.out.println("region,epsilon,mode,found,cost,expansions,ms,actions,validator");
        for (Path region : regions) {
            WorldBinFormat.Loaded loaded = WorldBinFormat.load(region);
            var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
            var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);

            for (double eps : new double[] {1.5, 1.0}) {
                for (WeightedAStar.MinePruneMode mode : WeightedAStar.MinePruneMode.values()) {
                    WeightedAStar.minePruneMode = mode;
                    AirPotential.OLD_GROUND_CUTOFF = -1; // repo default, unchanged across arms

                    long t0 = System.nanoTime();
                    WeightedAStar solver = new WeightedAStar();
                    SearchResult r = solver.search(loaded.world(), start, goal,
                            loaded.blocksAvailable(), eps, 1.0, maxExpansions);
                    double ms = (System.nanoTime() - t0) / 1e6;

                    String validator = "-";
                    int actionCount = -1;
                    if (r.found()) {
                        var violations = PathValidator.validate(loaded.world(), r.path(), r.actions());
                        validator = violations.isEmpty() ? "OK" : (violations.size() + "_VIOLATIONS");
                        actionCount = r.actions().length;
                    }
                    System.out.printf(Locale.ROOT, "%s,%.1f,%s,%b,%.4f,%d,%.1f,%d,%s%n",
                            region.getFileName(), eps, mode, r.found(),
                            r.found() ? r.totalCost() : -1.0, r.expansions(), ms, actionCount, validator);
                    System.out.flush();
                }
            }
        }
    }
}
