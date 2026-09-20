package dev.netherpathfinder.client;

import java.util.HashSet;
import java.util.Set;

/**
 * Merges a set of unit cells (blocks to dig, or blocks to place) into ONE surface instead of N
 * separate cubes: only the union's silhouette is drawn -- an edge is kept where the surface turns a
 * corner, and dropped where it continues flat or lies inside the volume -- plus filled faces only on
 * the exposed boundary. A 2-tall tunnel of 30 blocks becomes a single tube outline; a diagonal
 * bridge becomes one ribbon.
 */
final class RouteVolumes {
    private RouteVolumes() {}

    /** Outward nudge, in blocks, so the outline and faces don't z-fight the real blocks they hug. */
    private static final float INFLATE = 0.004f;

    /** Immutable merged geometry in world coordinates. */
    static final class Mesh {
        static final Mesh EMPTY = new Mesh(new float[0], new float[0]);
        /** 6 floats per edge: x0,y0,z0,x1,y1,z1. */
        final float[] edges;
        /** 12 floats per quad: four xyz corners. */
        final float[] quads;

        Mesh(float[] edges, float[] quads) {
            this.edges = edges;
            this.quads = quads;
        }

        boolean isEmpty() { return edges.length == 0 && quads.length == 0; }
    }

    static long key(int x, int y, int z) {
        return ((long) (x & 0x3FFFFFF) << 38) | ((long) (z & 0x3FFFFFF) << 12) | ((y + 2048) & 0xFFF);
    }

    static Mesh build(Set<Long> cells) {
        if (cells.isEmpty()) return Mesh.EMPTY;
        // Decode once; the packed key is only for set membership.
        int[][] list = new int[cells.size()][];
        int n = 0;
        for (long k : cells) {
            int x = (int) (k >> 38);
            if ((x & 0x2000000) != 0) x |= 0xFC000000; // sign-extend 26 bits
            int z = (int) ((k >> 12) & 0x3FFFFFF);
            if ((z & 0x2000000) != 0) z |= 0xFC000000;
            int y = (int) (k & 0xFFF) - 2048;
            list[n++] = new int[] {x, y, z};
        }

        // ---- feature edges ----
        @SuppressWarnings("unchecked")
        Set<Long>[] seenEdges = new Set[] {new HashSet<Long>(), new HashSet<Long>(), new HashSet<Long>()};
        float[] edges = new float[cells.size() * 12 * 6];
        int edgeFloats = 0;
        for (int[] c : list) {
            for (int axis = 0; axis < 3; axis++) {
                for (int a = 0; a <= 1; a++) {
                    for (int b = 0; b <= 1; b++) {
                        // Grid corner of this edge: base cell corner + (a,b) on the two non-axis axes.
                        int ex = c[0], ey = c[1], ez = c[2];
                        if (axis == 0) { ey += a; ez += b; }
                        else if (axis == 1) { ex += a; ez += b; }
                        else { ex += a; ey += b; }
                        if (!seenEdges[axis].add(key(ex, ey, ez))) continue; // one entry per (axis, corner)
                        int written = emitEdge(cells, axis, ex, ey, ez, edges, edgeFloats);
                        edgeFloats += written;
                    }
                }
            }
        }

        // ---- exposed faces ----
        float[] quads = new float[cells.size() * 6 * 12];
        int quadFloats = 0;
        for (int[] c : list) {
            int x = c[0], y = c[1], z = c[2];
            float x0 = x, x1 = x + 1, y0 = y, y1 = y + 1, z0 = z, z1 = z + 1;
            if (!cells.contains(key(x + 1, y, z))) quadFloats = quad(quads, quadFloats,
                x1 + INFLATE, y0, z0, x1 + INFLATE, y1, z0, x1 + INFLATE, y1, z1, x1 + INFLATE, y0, z1);
            if (!cells.contains(key(x - 1, y, z))) quadFloats = quad(quads, quadFloats,
                x0 - INFLATE, y0, z0, x0 - INFLATE, y0, z1, x0 - INFLATE, y1, z1, x0 - INFLATE, y1, z0);
            if (!cells.contains(key(x, y + 1, z))) quadFloats = quad(quads, quadFloats,
                x0, y1 + INFLATE, z0, x0, y1 + INFLATE, z1, x1, y1 + INFLATE, z1, x1, y1 + INFLATE, z0);
            if (!cells.contains(key(x, y - 1, z))) quadFloats = quad(quads, quadFloats,
                x0, y0 - INFLATE, z0, x1, y0 - INFLATE, z0, x1, y0 - INFLATE, z1, x0, y0 - INFLATE, z1);
            if (!cells.contains(key(x, y, z + 1))) quadFloats = quad(quads, quadFloats,
                x0, y0, z1 + INFLATE, x1, y0, z1 + INFLATE, x1, y1, z1 + INFLATE, x0, y1, z1 + INFLATE);
            if (!cells.contains(key(x, y, z - 1))) quadFloats = quad(quads, quadFloats,
                x0, y0, z0 - INFLATE, x0, y1, z0 - INFLATE, x1, y1, z0 - INFLATE, x1, y0, z0 - INFLATE);
        }
        return new Mesh(java.util.Arrays.copyOf(edges, edgeFloats), java.util.Arrays.copyOf(quads, quadFloats));
    }

    /**
     * Looks at the four cells around one grid edge and writes the edge only if the surface really
     * turns there. Returns floats written (0 or 6).
     */
    private static int emitEdge(Set<Long> cells, int axis, int ex, int ey, int ez, float[] out, int at) {
        // The two axes perpendicular to `axis`; (u,v) index the four cells around the edge as -1/0.
        boolean[][] filled = new boolean[2][2]; // [u][v]: 0 -> cell at corner-1, 1 -> cell at corner
        int count = 0;
        for (int u = 0; u <= 1; u++) {
            for (int v = 0; v <= 1; v++) {
                int cu = u - 1, cv = v - 1; // offsets -1 or 0
                int cx = ex, cy = ey, cz = ez;
                if (axis == 0) { cy += cu; cz += cv; }
                else if (axis == 1) { cx += cu; cz += cv; }
                else { cx += cu; cy += cv; }
                boolean f = cells.contains(key(cx, cy, cz));
                filled[u][v] = f;
                if (f) count++;
            }
        }
        if (count == 0 || count == 4) return 0;
        if (count == 2) {
            boolean flatU = (filled[0][0] && filled[0][1]) || (filled[1][0] && filled[1][1]);
            boolean flatV = (filled[0][0] && filled[1][0]) || (filled[0][1] && filled[1][1]);
            if (flatU || flatV) return 0; // the surface continues straight through this edge
        }

        // Outward direction in the (u,v) plane: away from filled cells, toward empty ones.
        double ou = 0, ov = 0;
        for (int u = 0; u <= 1; u++) {
            for (int v = 0; v <= 1; v++) {
                double sign = filled[u][v] ? -1 : 1;
                ou += sign * (u == 0 ? -0.5 : 0.5);
                ov += sign * (v == 0 ? -0.5 : 0.5);
            }
        }
        double len = Math.hypot(ou, ov);
        float du = len < 1.0e-6 ? 0 : (float) (ou / len * INFLATE);
        float dv = len < 1.0e-6 ? 0 : (float) (ov / len * INFLATE);

        float x0 = ex, y0 = ey, z0 = ez, x1 = ex, y1 = ey, z1 = ez;
        if (axis == 0) { x1 += 1; y0 += du; y1 += du; z0 += dv; z1 += dv; }
        else if (axis == 1) { y1 += 1; x0 += du; x1 += du; z0 += dv; z1 += dv; }
        else { z1 += 1; x0 += du; x1 += du; y0 += dv; y1 += dv; }
        out[at] = x0; out[at + 1] = y0; out[at + 2] = z0;
        out[at + 3] = x1; out[at + 4] = y1; out[at + 5] = z1;
        return 6;
    }

    private static int quad(float[] out, int at, float ax, float ay, float az, float bx, float by, float bz,
                            float cx, float cy, float cz, float dx, float dy, float dz) {
        out[at] = ax; out[at + 1] = ay; out[at + 2] = az;
        out[at + 3] = bx; out[at + 4] = by; out[at + 5] = bz;
        out[at + 6] = cx; out[at + 7] = cy; out[at + 8] = cz;
        out[at + 9] = dx; out[at + 10] = dy; out[at + 11] = dz;
        return at + 12;
    }
}
