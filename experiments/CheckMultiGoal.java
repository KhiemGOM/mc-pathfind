import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;

/**
 * Throwaway check: wall_world.wbin's header goal is (50,5,10), forcing a
 * detour through the wall's gap (cost ~9.76s, all SPRINT). Adding a second,
 * much closer target at (20,5,10) -- flat open ground, no wall between it
 * and start (10,5,10) -- should let the solver stop there instead, at
 * roughly 10/SPRINT_SPEED=~1.79s. Confirms GoalPoints actually changes
 * behavior (picks the cheaper reachable target), not just compiles.
 */
public class CheckMultiGoal {
    public static void main(String[] args) throws Exception {
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(Path.of("benchmark/data/lab/wall_world.wbin"));
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var farGoal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
        var nearGoal = new StateCodec.State(20, 5, 10, 0, false);
        int blocks = loaded.blocksAvailable();

        System.out.println("--- single-point goal (far, via gap) ---");
        report(new WeightedAStar().search(world, start, farGoal, blocks, 1.0, 1.0, 300_000), world, farGoal);
        report(new BidirectionalWeightedAStar().search(world, start, farGoal, blocks, 1.0, 1.0, 300_000), world, farGoal);

        GoalPoints multi = GoalPoints.of(farGoal, nearGoal);
        System.out.println("--- multi-point goal (near OR far) ---");
        report(new WeightedAStar().search(world, start, multi, blocks, 1.0, 1.0, 300_000), world, null);
        report(new BidirectionalWeightedAStar().search(world, start, multi, blocks, 1.0, 1.0, 300_000), world, null);
        report(new ParallelBidirectionalWeightedAStar().search(world, start, multi, blocks, 1.0, 1.0, 300_000), world, null);
    }

    private static void report(SearchResult r, World world, StateCodec.State goalForValidation) {
        if (!r.found()) {
            System.out.println("  NOT FOUND");
            return;
        }
        StateCodec.State last = r.path()[r.path().length - 1];
        var violations = goalForValidation != null
                ? PathValidator.validate(world, r.path(), r.actions())
                : PathValidator.validate(world, r.path(), r.actions());
        System.out.printf("  cost=%.3f nodes=%d expansions=%d landedAt=(%d,%d,%d) validator=%s%n",
                r.totalCost(), r.path().length, r.expansions(), last.x(), last.y(), last.z(),
                violations.isEmpty() ? "OK" : violations.size() + " VIOLATIONS");
    }
}
