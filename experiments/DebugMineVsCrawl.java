import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;

/** Traces BidirectionalWeightedAStar's mu updates on mine_vs_crawl_long_world.wbin to diagnose the mixed-technique bug. */
public class DebugMineVsCrawl {
    public static void main(String[] args) throws Exception {
        BidirectionalWeightedAStar.DEBUG = true;
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(Path.of("benchmark/data/lab/mine_vs_crawl_long_world.wbin"));
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
        int blocks = loaded.blocksAvailable();

        SearchResult r = new BidirectionalWeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 300_000);
        System.out.println("final cost=" + r.totalCost() + " expansions=" + r.expansions());
        StringBuilder sb = new StringBuilder();
        for (Action a : r.actions()) sb.append(a).append(" ");
        System.out.println("actions: " + sb);
    }
}
