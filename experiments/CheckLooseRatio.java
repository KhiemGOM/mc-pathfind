import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;
import java.util.List;

public class CheckLooseRatio {
    public static void main(String[] args) throws Exception {
        Path wbin = Path.of("benchmark/data/lab/floating_islands_world.wbin");
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
        int blocks = loaded.blocksAvailable();

        EdgeRules.BRIDGE_UP_PRUNE_RATIO = 0.5;
        SearchResult r = new BidirectionalWeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
        System.out.println("found=" + r.found() + " cost=" + r.totalCost());
        List<PathValidator.Violation> violations = PathValidator.validate(world, r.path(), r.actions());
        System.out.println(violations.isEmpty() ? "PathValidator: OK" : "VIOLATIONS: " + violations);
    }
}
