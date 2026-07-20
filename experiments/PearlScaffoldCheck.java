import dev.mcpathfind.pearl.*;
import dev.mcpathfind.pearl.io.WorldBinFormat;
import java.nio.file.Path;

/**
 * Smoke test for the newly-scaffolded pearl package: runs forward AND
 * bidirectional against existing lab/real-region .wbin files, independently
 * re-checks every returned path with PathValidator (which itself
 * independently re-derives and re-checks every PEARL trajectory -- see its
 * PEARL case), and cross-checks bidirectional's cost against forward's --
 * bidirectional must never report a WORSE cost than forward at the same
 * epsilon, same discipline this project has used since forward/bidirectional
 * both first existed.
 */
public class PearlScaffoldCheck {
    public static void main(String[] args) throws Exception {
        String[] worlds = {
            "benchmark/data/lab/wall_world.wbin",
            "benchmark/data/lab/mine_vs_crawl_short_world.wbin",
            "benchmark/data/lab/mine_vs_crawl_long_world.wbin",
            "benchmark/data/lab/rolling_terrain_world.wbin",
            "benchmark/data/k2_r_0_0.wbin",
        };
        boolean allOk = true;
        for (String path : worlds) {
            allOk &= runOne(path);
        }
        System.out.println();
        System.out.println(allOk ? "ALL PASS" : "SOME FAILED");
    }

    private static boolean runOne(String path) throws Exception {
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(Path.of(path));
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), false);

        Result fwd = runForward(world, start, goal);
        Result bidir = runBidirectional(world, start, goal);
        Result par = runParallel(world, start, goal);

        System.out.printf("%-55s fwd:    %s%n", path, fwd.summary());
        System.out.printf("%-55s bidir:  %s%n", "", bidir.summary());
        System.out.printf("%-55s par:    %s%n", "", par.summary());

        boolean ok = fwd.ok() && bidir.ok() && par.ok();
        if (fwd.found && bidir.found && bidir.cost > fwd.cost + 1e-6) {
            System.out.printf("  ** bidirectional cost %.4f > forward cost %.4f -- REGRESSION **%n", bidir.cost, fwd.cost);
            ok = false;
        }
        if (fwd.found && par.found && par.cost > fwd.cost + 1e-6) {
            System.out.printf("  ** parallel cost %.4f > forward cost %.4f -- REGRESSION **%n", par.cost, fwd.cost);
            ok = false;
        }
        return ok;
    }

    private static Result runForward(World world, StateCodec.State start, StateCodec.State goal) {
        long t0 = System.nanoTime();
        WeightedAStar solver = new WeightedAStar();
        SearchResult r = solver.search(world, start, goal, 1.0, 1.0, 2_000_000);
        double ms = (System.nanoTime() - t0) / 1e6;
        return Result.from(world, r, ms);
    }

    private static Result runBidirectional(World world, StateCodec.State start, StateCodec.State goal) {
        long t0 = System.nanoTime();
        BidirectionalWeightedAStar solver = new BidirectionalWeightedAStar();
        SearchResult r = solver.search(world, start, goal, 1.0, 1.0, 2_000_000);
        double ms = (System.nanoTime() - t0) / 1e6;
        return Result.from(world, r, ms);
    }

    private static Result runParallel(World world, StateCodec.State start, StateCodec.State goal) {
        long t0 = System.nanoTime();
        ParallelBidirectionalWeightedAStar solver = new ParallelBidirectionalWeightedAStar();
        SearchResult r = solver.search(world, start, goal, 1.0, 1.0, 2_000_000);
        double ms = (System.nanoTime() - t0) / 1e6;
        return Result.from(world, r, ms);
    }

    private record Result(boolean found, double cost, int nodes, int expansions, double ms,
                           java.util.List<PathValidator.Violation> violations,
                           java.util.Map<Action, Integer> counts) {
        static Result from(World world, SearchResult r, double ms) {
            if (!r.found()) {
                return new Result(false, 0, 0, r.expansions(), ms, java.util.List.of(), java.util.Map.of());
            }
            var violations = PathValidator.validate(world, r.path(), r.actions());
            java.util.Map<Action, Integer> counts = new java.util.EnumMap<>(Action.class);
            for (Action a : r.actions()) counts.merge(a, 1, Integer::sum);
            return new Result(true, r.totalCost(), r.path().length, r.expansions(), ms, violations, counts);
        }

        boolean ok() {
            return found && violations.isEmpty();
        }

        String summary() {
            if (!found) {
                return "NOT FOUND (expansions=" + expansions + ")";
            }
            return String.format(java.util.Locale.ROOT,
                "found cost=%.2f nodes=%d expansions=%d time=%.1fms validator=%s actions=%s",
                cost, nodes, expansions, ms,
                violations.isEmpty() ? "OK" : violations.size() + " VIOLATION(S): " + violations.get(0),
                counts);
        }
    }
}
