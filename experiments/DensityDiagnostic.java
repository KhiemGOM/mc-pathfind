import dev.mcpathfind.core.StoneDensityField;
import dev.mcpathfind.core.World;
import dev.mcpathfind.core.io.WorldBinFormat;

import java.nio.file.Path;
import java.util.Locale;

/** One-off diagnostic: how much of the world actually crosses HYBRID_STONE_DENSITY_THRESHOLD at CHUNK=4 granularity? */
public class DensityDiagnostic {
    public static void main(String[] args) throws Exception {
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(Path.of(args[0]));
        World world = loaded.world();
        StoneDensityField field = StoneDensityField.build(world);

        int[] goal = {57, 41, 140};
        System.out.printf(Locale.ROOT, "density at goal (57,41,140) = %.3f%n", field.densityAt(goal[0], goal[1], goal[2]));

        // sample a small cube around the goal
        System.out.println("density in a 12-block cube around the goal:");
        double max = 0;
        int above35 = 0, total = 0;
        for (int dx = -12; dx <= 12; dx += 2) {
            for (int dy = -12; dy <= 12; dy += 2) {
                for (int dz = -12; dz <= 12; dz += 2) {
                    double d = field.densityAt(goal[0] + dx, goal[1] + dy, goal[2] + dz);
                    max = Math.max(max, d);
                    total++;
                    if (d >= 0.35) above35++;
                }
            }
        }
        System.out.printf(Locale.ROOT, "  max density nearby = %.3f, fraction of nearby samples >= 0.35 = %d/%d (%.1f%%)%n",
                max, above35, total, 100.0 * above35 / total);

        // whole-world histogram at chunk granularity
        int[] hist = new int[11]; // 0.0-0.1, 0.1-0.2, ..., 1.0
        int wholeMax = 0;
        double globalMax = 0;
        for (int x = 0; x < world.sizeX; x += StoneDensityField.CHUNK) {
            for (int y = 0; y < world.sizeY; y += StoneDensityField.CHUNK) {
                for (int z = 0; z < world.sizeZ; z += StoneDensityField.CHUNK) {
                    double d = field.densityAt(x, y, z);
                    globalMax = Math.max(globalMax, d);
                    int bucket = Math.min(10, (int) (d * 10));
                    hist[bucket]++;
                }
            }
        }
        System.out.printf(Locale.ROOT, "%nwhole-world chunk density histogram (global max=%.3f):%n", globalMax);
        for (int i = 0; i < hist.length; i++) {
            System.out.printf(Locale.ROOT, "  [%.1f-%.1f): %d chunks%n", i / 10.0, (i + 1) / 10.0, hist[i]);
        }
    }
}
