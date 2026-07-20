import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;

public class DebugBridgeSearch {
    public static void main(String[] args) throws Exception {
        Path wbin = Path.of("benchmark/data/lab/rolling_terrain_world.wbin");
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
        int blocks = loaded.blocksAvailable();

        WeightedAStar.minePruneMode = WeightedAStar.MinePruneMode.NONE;
        BidirectionalWeightedAStar.DEBUG = true;
        SearchResult r = new BidirectionalWeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
        System.out.println("RESULT: found=" + r.found() + " cost=" + r.totalCost() + " nodes=" + r.path().length);
    }
}
