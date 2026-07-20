import dev.mcpathfind.core.*;
import dev.mcpathfind.core.io.WorldBinFormat;
import java.nio.file.Path;
import java.util.concurrent.atomic.AtomicInteger;

public class DebugBridgeReverse {
    public static void main(String[] args) throws Exception {
        Path wbin = Path.of("benchmark/data/lab/rolling_terrain_world.wbin");
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(wbin);
        World world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);
        int blocks = loaded.blocksAvailable();
        System.out.println("start=" + start + " goal=" + goal + " blocks=" + blocks);

        WeightedAStar.minePruneMode = WeightedAStar.MinePruneMode.NONE;

        // How many (x,y,z) positions across the whole world does
        // reverseBridgeSources emit ANY candidate for, at blocksNeeded=0?
        AtomicInteger totalCandidates = new AtomicInteger(0);
        AtomicInteger positionsWithCandidates = new AtomicInteger(0);
        for (int x = 0; x < world.sizeX; x++) {
            for (int y = 0; y < world.sizeY; y++) {
                for (int z = 0; z < world.sizeZ; z++) {
                    AtomicInteger here = new AtomicInteger(0);
                    EdgeRules.reverseBridgeSources(world, x, y, z, 1.0, 0, (toState, cost, action) -> here.incrementAndGet());
                    if (here.get() > 0) {
                        positionsWithCandidates.incrementAndGet();
                        totalCandidates.addAndGet(here.get());
                    }
                }
            }
        }
        System.out.println("reverseBridgeSources: positions with >=1 candidate = " + positionsWithCandidates.get()
                + ", total candidates = " + totalCandidates.get());

        // Now run forward-only to get the TRUE optimal path and see exactly
        // which (x,y,z) BRIDGE is used at.
        SearchResult fwd = new WeightedAStar().search(world, start, goal, blocks, 1.0, 1.0, 3_000_000);
        System.out.println("forward: found=" + fwd.found() + " cost=" + fwd.totalCost());
        for (int i = 12; i < Math.min(fwd.actions().length, 24); i++) {
            System.out.println("  step " + i + ": " + fwd.actions()[i] + "  " + fwd.path()[i] + " -> " + fwd.path()[i + 1]);
        }
        for (int i = 0; i < fwd.actions().length; i++) {
            if (fwd.actions()[i] == Action.BRIDGE) {
                StateCodec.State from = fwd.path()[i];
                StateCodec.State to = fwd.path()[i + 1];
                System.out.println("  BRIDGE at step " + i + ": " + from + " -> " + to);
            }
        }

        // Check: does reverseBridgeSources correctly find the FROM state as
        // a predecessor of the TO state, at blocksNeeded=0?
        for (int i = 0; i < fwd.actions().length; i++) {
            if (fwd.actions()[i] == Action.BRIDGE) {
                StateCodec.State from = fwd.path()[i];
                StateCodec.State to = fwd.path()[i + 1];
                AtomicInteger found = new AtomicInteger(0);
                EdgeRules.reverseBridgeSources(world, to.x(), to.y(), to.z(), 1.0, 0, (toState, cost, action) -> {
                    int sx = StateCodec.unpackX(toState), sy = StateCodec.unpackY(toState), sz = StateCodec.unpackZ(toState);
                    int needed = StateCodec.unpackBlocks(toState);
                    System.out.println("    candidate predecessor: (" + sx + "," + sy + "," + sz + ") blocksNeeded=" + needed + " cost=" + cost);
                    if (sx == from.x() && sy == from.y() && sz == from.z()) {
                        found.incrementAndGet();
                    }
                });
                System.out.println("  reverse probe from target " + to + " found true source " + (found.get() > 0 ? "YES" : "NO"));
            }
        }
    }
}
