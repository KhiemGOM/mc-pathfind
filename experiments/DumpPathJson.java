import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;

/** Dumps a solved path as JSON: {"path":[[x,y,z,blocks],...],"actions":[...],"cost":N,"expansions":N} */
public class DumpPathJson {
    public static void main(String[] args) throws Exception {
        Path wbin = Path.of(args[0]);
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
        int blocks = loaded.blocksAvailable();

        SearchResult r = new BidirectionalWeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
        if (!r.found()) {
            System.out.println("{\"found\":false}");
            return;
        }

        StringBuilder sb = new StringBuilder();
        sb.append("{\"found\":true,\"cost\":").append(r.totalCost()).append(",\"expansions\":").append(r.expansions());
        sb.append(",\"path\":[");
        for (int i = 0; i < r.path().length; i++) {
            if (i > 0) sb.append(",");
            StateCodec.State s = r.path()[i];
            sb.append("[").append(s.x()).append(",").append(s.y()).append(",").append(s.z()).append(",").append(s.blocksRemaining()).append("]");
        }
        sb.append("],\"actions\":[");
        for (int i = 0; i < r.actions().length; i++) {
            if (i > 0) sb.append(",");
            sb.append("\"").append(r.actions()[i]).append("\"");
        }
        sb.append("]}");
        System.out.println(sb);
    }
}
