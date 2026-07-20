import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

/**
 * Runs the hand-built "lab room" synthetic worlds (world.py's
 * generate_wall_world / generate_bridge_world / generate_cave_shortcut_world
 * / generate_world, exported via experiments/export_lab_worlds.py) through
 * all three Java solvers under three prune configs -- NONE (pre-AirPotential
 * baseline), AIR_POTENTIAL with the OLD cutoff=0, and AIR_POTENTIAL with the
 * NEW default cutoff=-1 -- to check whether this session's pruning changes
 * lost any basic fidelity on scenarios simple enough to reason about by
 * hand. cave_shortcut_world.wbin is the important one: it's built so mining
 * through a 3-block stone plug is unambiguously cheaper than detouring
 * around, so if the prune wrongly skips the shortcut, cost will visibly
 * jump instead of just drifting a little the way it can on real terrain.
 */
public class LabFidelityCheck {
    public static void main(String[] args) throws Exception {
        Path dir = Path.of(args.length > 0 ? args[0] : "benchmark/data/lab");
        List<Path> files = Files.list(dir).filter(p -> p.toString().endsWith(".wbin")).sorted().toList();

        record Config(String label, WeightedAStar.MinePruneMode mode, int cutoff) {}
        Config[] configs = {
            new Config("NONE (prune off)", WeightedAStar.MinePruneMode.NONE, 0),
            new Config("AIR_POTENTIAL cutoff=0 (old)", WeightedAStar.MinePruneMode.AIR_POTENTIAL, 0),
            new Config("AIR_POTENTIAL cutoff=-1 (new default)", WeightedAStar.MinePruneMode.AIR_POTENTIAL, -1),
        };

        for (Path f : files) {
            WorldBinFormat.Loaded loaded = WorldBinFormat.load(f);
            World world = loaded.world();
            var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
            var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
            int blocks = loaded.blocksAvailable();

            System.out.println("\n########## " + f.getFileName() + " ##########");
            for (Config c : configs) {
                WeightedAStar.minePruneMode = c.mode();
                AirPotential.OLD_GROUND_CUTOFF = c.cutoff();
                System.out.println("--- " + c.label() + " ---");

                SearchResult fwd = new WeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
                report("forward     ", world, fwd);

                SearchResult bidir = new BidirectionalWeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
                report("bidirectional", world, bidir);

                SearchResult par = new ParallelBidirectionalWeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
                report("parallel     ", world, par);
            }
        }
    }

    private static void report(String label, World world, SearchResult r) {
        if (!r.found()) {
            System.out.printf("  %s NOT FOUND (expansions=%d)%n", label, r.expansions());
            return;
        }
        List<PathValidator.Violation> violations = PathValidator.validate(world, r.path(), r.actions());
        java.util.Map<Action, Integer> counts = new java.util.EnumMap<>(Action.class);
        for (Action a : r.actions()) counts.merge(a, 1, Integer::sum);
        System.out.printf("  %s cost=%-8.2f nodes=%-5d expansions=%-8d validator=%-20s actions=%s%n",
                label, r.totalCost(), r.path().length, r.expansions(),
                violations.isEmpty() ? "OK" : (violations.size() + " VIOLATIONS"), counts);
    }
}
