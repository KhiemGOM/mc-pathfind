import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;

/**
 * Dumps a solved path as JSON: {"path":[[x,y,z,blocks],...],"actions":[...],"cost":N,"expansions":N}
 * Usage: DumpPathJson <wbin> [--epsilon E] [--max-expansions N]
 * Defaults (1.5, 8_000_000) match this project's usual practical operating
 * point -- fast enough in the Java port to not need the Python epsilon-
 * retry-ladder real terrain requires (see build_visualization.py's module
 * docstring for why that ladder exists there).
 */
public class DumpPathJson {
    public static void main(String[] args) throws Exception {
        Path wbin = Path.of(args[0]);
        double epsilon = 1.5;
        int maxExpansions = 8_000_000;
        for (int i = 1; i < args.length; i++) {
            switch (args[i]) {
                case "--epsilon" -> epsilon = Double.parseDouble(args[++i]);
                case "--max-expansions" -> maxExpansions = Integer.parseInt(args[++i]);
                default -> throw new IllegalArgumentException("unknown arg: " + args[i]);
            }
        }

        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
        int blocks = loaded.blocksAvailable();

        SearchResult r = new BidirectionalWeightedAStar().search(world, start, goal, blocks, epsilon, 1.0, maxExpansions);
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
