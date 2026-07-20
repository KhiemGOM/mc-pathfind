package dev.mcpathfind.core.io;

import dev.mcpathfind.core.*;

import java.nio.file.Path;

public final class DebugViolation {
    public static void main(String[] args) throws Exception {
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(Path.of(args[0]));
        var world = loaded.world();
        var start = new StateCodec.State(loaded.start().x(), loaded.start().y(), loaded.start().z(), 0, false);
        var goal = new StateCodec.State(loaded.goal().x(), loaded.goal().y(), loaded.goal().z(), 0, false);

        WeightedAStar solver = new WeightedAStar();
        SearchResult r = solver.search(world, start, goal, loaded.blocksAvailable(), 5.0, 1.0, 300_000);
        System.out.println("found=" + r.found() + " nodes=" + r.path().length + " expansions=" + r.expansions());

        int from = Math.max(0, 210);
        int to = Math.min(r.path().length, 220);
        for (int i = from; i < to; i++) {
            var s = r.path()[i];
            String action = i > 0 ? String.valueOf(r.actions()[i - 1]) : "START";
            System.out.printf("[%d] (%d,%d,%d) blocks=%d crawl=%s action=%s%n",
                    i, s.x(), s.y(), s.z(), s.blocksRemaining(), s.crawling(), action);
            int target = world.voxelAt(s.x(), s.y(), s.z());
            int head = world.voxelAt(s.x(), s.y() + 1, s.z());
            int below = world.voxelAt(s.x(), s.y() - 1, s.z());
            System.out.printf("      below=%d target=%d head=%d%n", below, target, head);
        }
    }
}
