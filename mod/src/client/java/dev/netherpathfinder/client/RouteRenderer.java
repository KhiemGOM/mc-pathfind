package dev.netherpathfinder.client;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import dev.netherpathfinder.engine.Action;
import net.minecraft.resources.Identifier;
import net.fabricmc.fabric.api.client.rendering.v1.level.LevelRenderContext;
import net.fabricmc.fabric.api.client.rendering.v1.level.LevelRenderEvents;
import net.minecraft.client.renderer.rendertype.RenderTypes;
import net.minecraft.world.phys.Vec3;
import org.joml.Matrix4f;

/**
 * Draws the active route (see RouteState) as a camera-relative wireframe
 * overlay: a line strip through every waypoint, plus a small box at each
 * one, matching the GPS-style "here's the route, walk it yourself" overlay
 * this mod is built around -- this never moves the player, it only draws.
 *
 * Submitted during Fabric's modern render-state collection phase. The 26.2
 * renderer owns flushing; this class only submits immutable route geometry.
 */
public final class RouteRenderer {
    private RouteRenderer() {}

    private static final float LINE_WIDTH = 3.0f;
    private static final float LINE_A = 0.95f;
    private static final float PAD_A = 0.85f;
    private static final float DIG_A = 0.9f;
    private static final float FILL_A = 0.16f;

    // Colour per action, so the route reads at a glance: what you sprint, dig, bridge, climb, or drop.
    private static final float[] COLOR_SPRINT = {0.20f, 0.90f, 1.00f};
    private static final float[] COLOR_DIG = {1.00f, 0.75f, 0.10f};
    private static final float[] COLOR_BRIDGE = {1.00f, 0.25f, 0.85f};
    private static final float[] COLOR_CLIMB = {0.30f, 1.00f, 0.40f};
    private static final float[] COLOR_FALL = {0.32f, 0.58f, 1.00f};
    private static final float[] COLOR_CRAWL = {0.72f, 0.42f, 1.00f};
    private static final float[] COLOR_PARKOUR = {1.00f, 0.55f, 0.15f};
    private static final float[] COLOR_OTHER = {1.00f, 1.00f, 1.00f};

    private static float[] colorOf(Action action) {
        if (action == null) return COLOR_SPRINT;
        return switch (action) {
            case MINE, MINE_DOWN -> COLOR_DIG;
            case BRIDGE, BRIDGE_UP -> COLOR_BRIDGE;
            case CLIMB -> COLOR_CLIMB;
            case FALL -> COLOR_FALL;
            case BOAT_CRAWL -> COLOR_CRAWL;
            case PARKOUR, SPRINT_JUMP_PEARL -> COLOR_PARKOUR;
            case SPRINT, START -> COLOR_SPRINT;
            default -> COLOR_OTHER;
        };
    }

    /** Blocks a step digs through at its destination: a 2-tall tunnel for MINE, 1 for MINE_DOWN / crawling. */
    private static int digHeight(Action action) {
        if (action == Action.MINE) return 2;
        if (action == Action.MINE_DOWN || action == Action.BOAT_CRAWL) return 1;
        return 0;
    }

    public static void register() {
        LevelRenderEvents.COLLECT_SUBMITS.register(RouteRenderer::render);
    }

    // Reveal animation (see RouteState.Route.animStartNanos): waypoints appear one by one along
    // the route, each dropping in from above and settling, with a bright head at the leading edge.
    private static final double DROP_BLOCKS = 10.0;
    private static final double SETTLE_SECONDS = 0.9;
    private static final float HEAD_R = 1.0f, HEAD_G = 1.0f, HEAD_B = 1.0f, HEAD_A = 1.0f;
    private static final double HEAD_HALF_SIZE = 0.35;

    private static double revealSeconds(int points) {
        return Math.max(3.0, Math.min(8.0, points * 0.035));
    }

    // Merged dig/bridge volumes for the settled part of the route; rebuilt only when that part grows.
    private static RouteState.Route volumesRoute;
    private static int volumesSettled = -1;
    private static RouteVolumes.Mesh digMesh = RouteVolumes.Mesh.EMPTY;
    private static RouteVolumes.Mesh bridgeMesh = RouteVolumes.Mesh.EMPTY;

    private static void updateVolumes(RouteState.Route route, int settled) {
        if (route == volumesRoute && settled == volumesSettled) return;
        java.util.Set<Long> dig = new java.util.HashSet<>();
        java.util.Set<Long> bridge = new java.util.HashSet<>();
        for (int i = 1; i < settled && i <= route.actions.length; i++) {
            Action arriving = route.actions[i - 1];
            int x = (int) Math.floor(route.points[i][0]);
            int y = (int) Math.floor(route.points[i][1]);
            int z = (int) Math.floor(route.points[i][2]);
            for (int k = 0; k < digHeight(arriving); k++) dig.add(RouteVolumes.key(x, y + k, z));
            if (arriving == Action.BRIDGE || arriving == Action.BRIDGE_UP) bridge.add(RouteVolumes.key(x, y - 1, z));
        }
        digMesh = RouteVolumes.build(dig);
        bridgeMesh = RouteVolumes.build(bridge);
        volumesRoute = route;
        volumesSettled = settled;
    }

    private static void render(LevelRenderContext context) {
        RouteState.Route route = RouteState.INSTANCE.get();
        if (route == null || route.status != RouteState.Status.FOUND || route.points.length < 1) {
            return;
        }

        final int n = route.points.length;
        final double elapsed = route.animStartNanos == 0
            ? Double.POSITIVE_INFINITY : (System.nanoTime() - route.animStartNanos) / 1.0e9;
        final double reveal = revealSeconds(n);
        final boolean revealing = elapsed < reveal;
        final boolean settling = elapsed < reveal + SETTLE_SECONDS;
        final int shown = !revealing ? n
            : Math.min(n, (int) Math.floor(elapsed / reveal * Math.max(1, n - 1)) + 1);
        // Nodes that have finished dropping into place; merged volumes and icons only use these.
        final int settled = !settling ? n
            : Math.max(0, Math.min(n, (int) Math.floor((elapsed - SETTLE_SECONDS) / reveal * Math.max(1, n - 1)) + 1));

        final double[][] pts = new double[shown][];
        final boolean[] falling = new boolean[shown]; // still dropping in: not yet on the ground
        for (int i = 0; i < shown; i++) {
            double offset = 0;
            if (settling) {
                double appearsAt = reveal * i / Math.max(1, n - 1);
                double t = Math.max(0, Math.min(1, (elapsed - appearsAt) / SETTLE_SECONDS));
                double inv = 1 - t;
                offset = DROP_BLOCKS * inv * inv * inv;
            }
            falling[i] = offset > 0.02;
            double[] p = route.points[i];
            pts[i] = new double[] {p[0], p[1] + offset, p[2]};
        }
        updateVolumes(route, settled);

        Vec3 camera = context.levelState().cameraRenderState.pos;
        PoseStack poseStack = context.poseStack();
        final Action[] actions = route.actions;

        var settings = PathfindCommands.settings();
        // 0 = always normal; 1 = see-through only while a piece is falling in, normal once it lands; 2 = always see-through
        final int mode = settings == null ? 0 : (int) settings.get("routeThroughWalls");
        if (mode > 0) updateXrayScale();

        if (mode < 2) {
            poseStack.pushPose();
            poseStack.translate(-camera.x, -camera.y, -camera.z);
            // Translucent fill of the merged dig / bridge volumes (one surface each, not one box per block).
            if (!digMesh.isEmpty() || !bridgeMesh.isEmpty()) {
                context.submitNodeCollector().submitCustomGeometry(poseStack, RenderTypes.debugQuads(), (pose, quads) -> {
                    Matrix4f matrix = pose.pose();
                    addQuads(quads, matrix, digMesh.quads, COLOR_DIG, FILL_A);
                    addQuads(quads, matrix, bridgeMesh.quads, COLOR_BRIDGE, FILL_A);
                });
            }
            context.submitNodeCollector().submitCustomGeometry(poseStack, RenderTypes.LINES, (pose, lines) -> {
                Matrix4f matrix = pose.pose();
                for (int i = 0; i < pts.length; i++) {
                    // In "falling" mode a piece, and any segment touching one, is drawn see-through instead.
                    boolean nodeHere = !(mode == 1 && falling[i]);
                    boolean segHere = i + 1 < pts.length && !(mode == 1 && (falling[i] || falling[i + 1]));
                    Action step = i < actions.length ? actions[i] : null;
                    float[] c = colorOf(step);
                    if (segHere) {
                        double[] from = pts[i], to = pts[i + 1];
                        addLine(lines, matrix, new double[] {from[0], from[1] + 0.08, from[2]},
                            new double[] {to[0], to[1] + 0.08, to[2]}, c[0], c[1], c[2], LINE_A);
                    }
                    if (nodeHere) {
                        // A flat pad marks each node, coloured by the step that leaves it.
                        double x = pts[i][0], y = pts[i][1], z = pts[i][2];
                        addBox(lines, matrix, x - 0.22, y + 0.02, z - 0.22, x + 0.22, y + 0.10, z + 0.22,
                            c[0], c[1], c[2], PAD_A);
                    }
                }
                addEdges(lines, matrix, digMesh.edges, COLOR_DIG, DIG_A);
                addEdges(lines, matrix, bridgeMesh.edges, COLOR_BRIDGE, DIG_A);
                if (revealing && mode == 0 && pts.length > 0) {
                    double[] head = pts[pts.length - 1];
                    addBox(lines, matrix, head[0] - 0.35, head[1], head[2] - 0.35, head[0] + 0.35, head[1] + 0.7,
                        head[2] + 0.35, 1.0f, 1.0f, 1.0f, 1.0f);
                }
            });
            poseStack.popPose();
        }
        if (mode >= 1) renderXray(context, camera, pts, actions, falling, mode == 2, revealing);

        RouteIcons.render(context, route, settled);
    }

    // ---- see-through route --------------------------------------------------------------------
    // No line pipeline in the game can ignore depth, so in this mode everything is drawn with the text
    // see-through pipeline (depth test off, flat color, no fog): lines and edges become camera-facing
    // ribbons, pads and volume faces become plain quads, all on a solid white tile tinted by vertex color.

    private static final Identifier WHITE_TILE = Identifier.withDefaultNamespace("textures/block/white_concrete.png");
    private static final int XRAY_LIGHT = 0xF000F0;
    private static final float XRAY_FILL_A = 0.22f;
    private static final float XRAY_PAD_A = 0.85f;

    /** World size of one screen pixel at one block of distance, so ribbons keep a constant on-screen width. */
    private static double xrayPxScale = 0.0013;

    private static void updateXrayScale() {
        var mc = net.minecraft.client.Minecraft.getInstance();
        try {
            double fov = Math.toRadians(mc.options.fov().get());
            int height = Math.max(1, mc.getWindow().getHeight());
            xrayPxScale = 2.0 * Math.tan(fov / 2.0) / height;
        } catch (RuntimeException ignored) {
            // keep the previous scale
        }
    }

    /**
     * @param all false = only pieces that are still falling (and segments touching one) are drawn here; the
     *            settled rest is drawn by the normal depth-tested path. true = everything, volumes included.
     */
    private static void renderXray(LevelRenderContext context, Vec3 cam, double[][] pts, Action[] actions,
                                   boolean[] falling, boolean all, boolean revealing) {
        if (!all && !revealing) {
            boolean anyFalling = false;
            for (boolean f : falling) if (f) { anyFalling = true; break; }
            if (!anyFalling) return;
        }
        PoseStack poseStack = context.poseStack();
        poseStack.pushPose();
        poseStack.translate(-cam.x, -cam.y, -cam.z);
        context.submitNodeCollector().submitCustomGeometry(poseStack, RenderTypes.textSeeThrough(WHITE_TILE), (pose, buf) -> {
            Matrix4f m = pose.pose();
            if (all) {
                // Translucent volume faces first, so lines and edges draw over them.
                xrayFaces(buf, m, digMesh.quads, COLOR_DIG, XRAY_FILL_A);
                xrayFaces(buf, m, bridgeMesh.quads, COLOR_BRIDGE, XRAY_FILL_A);
            }
            for (int i = 0; i < pts.length; i++) {
                Action step = i < actions.length ? actions[i] : null;
                float[] c = colorOf(step);
                if (i + 1 < pts.length && (all || falling[i] || falling[i + 1])) {
                    double[] a = pts[i], b = pts[i + 1];
                    ribbon(buf, m, cam, a[0], a[1] + 0.08, a[2], b[0], b[1] + 0.08, b[2], 1.0f, c, LINE_A);
                }
                if (all || falling[i]) xrayPad(buf, m, cam, pts[i][0], pts[i][1] + 0.10, pts[i][2], c);
            }
            if (all) {
                xrayEdges(buf, m, cam, digMesh.edges, COLOR_DIG, DIG_A);
                xrayEdges(buf, m, cam, bridgeMesh.edges, COLOR_BRIDGE, DIG_A);
            }
            if (revealing && pts.length > 0) {
                double[] h = pts[pts.length - 1];
                xrayBox(buf, m, cam, h[0] - 0.35, h[1], h[2] - 0.35, h[0] + 0.35, h[1] + 0.7, h[2] + 0.35);
            }
        });
        poseStack.popPose();
    }

    /** A node's flat pad: the same outline as the normal render, plus a faint fill so it reads as a tile. */
    private static void xrayPad(VertexConsumer b, Matrix4f m, Vec3 cam, double x, double y, double z, float[] c) {
        double h = 0.22;
        flatQuad(b, m, x - h, y, z - h, x + h, z + h, c, 0.18f);
        ribbon(b, m, cam, x - h, y, z - h, x + h, y, z - h, 0.7f, c, XRAY_PAD_A);
        ribbon(b, m, cam, x + h, y, z - h, x + h, y, z + h, 0.7f, c, XRAY_PAD_A);
        ribbon(b, m, cam, x + h, y, z + h, x - h, y, z + h, 0.7f, c, XRAY_PAD_A);
        ribbon(b, m, cam, x - h, y, z + h, x - h, y, z - h, 0.7f, c, XRAY_PAD_A);
    }

    private static void xv(VertexConsumer b, Matrix4f m, double x, double y, double z, float[] c, float a) {
        b.addVertex(m, (float) x, (float) y, (float) z).setColor(c[0], c[1], c[2], a).setUv(0.5f, 0.5f).setLight(XRAY_LIGHT);
    }

    /**
     * A camera-facing flat strip from (x0,y0,z0) to (x1,y1,z1). Its world width is proportional to distance,
     * so on screen it is always about LINE_WIDTH pixels (times widthScale) -- the same as the normal lines,
     * however close or far.
     */
    private static void ribbon(VertexConsumer b, Matrix4f m, Vec3 cam, double x0, double y0, double z0,
                               double x1, double y1, double z1, float widthScale, float[] c, float a) {
        double dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
        if (dx * dx + dy * dy + dz * dz < 1.0e-10) return;
        double mx = (x0 + x1) / 2 - cam.x, my = (y0 + y1) / 2 - cam.y, mz = (z0 + z1) / 2 - cam.z;
        double dist = Math.sqrt(mx * mx + my * my + mz * mz);
        // side = dir x toCamera (perpendicular to both the segment and the view ray)
        double sx = dy * -mz - dz * -my, sy = dz * -mx - dx * -mz, sz = dx * -my - dy * -mx;
        double len = Math.sqrt(sx * sx + sy * sy + sz * sz);
        if (len < 1.0e-9) { sx = 0; sy = 1; sz = 0; len = 1; } // looking straight down the segment
        double half = Math.max(0.0008, 0.5 * LINE_WIDTH * xrayPxScale * dist) * widthScale;
        sx = sx / len * half; sy = sy / len * half; sz = sz / len * half;
        xquad(b, m, x0 + sx, y0 + sy, z0 + sz, x0 - sx, y0 - sy, z0 - sz,
            x1 - sx, y1 - sy, z1 - sz, x1 + sx, y1 + sy, z1 + sz, c, a);
    }

    /**
     * The text pipelines cull back faces (the pipeline builder defaults cull to true and they never turn it
     * off), so every quad is emitted in BOTH windings: whichever faces the camera is drawn, the other is culled.
     */
    private static void xquad(VertexConsumer b, Matrix4f m,
                              double ax, double ay, double az, double bx, double by, double bz,
                              double cx, double cy, double cz, double dx, double dy, double dz,
                              float[] c, float a) {
        xv(b, m, ax, ay, az, c, a);
        xv(b, m, bx, by, bz, c, a);
        xv(b, m, cx, cy, cz, c, a);
        xv(b, m, dx, dy, dz, c, a);
        xv(b, m, dx, dy, dz, c, a);
        xv(b, m, cx, cy, cz, c, a);
        xv(b, m, bx, by, bz, c, a);
        xv(b, m, ax, ay, az, c, a);
    }

    private static void flatQuad(VertexConsumer b, Matrix4f m, double x0, double y, double z0, double x1, double z1,
                                 float[] c, float a) {
        xquad(b, m, x0, y, z0, x1, y, z0, x1, y, z1, x0, y, z1, c, a);
    }

    private static void xrayFaces(VertexConsumer b, Matrix4f m, float[] data, float[] c, float a) {
        for (int i = 0; i + 11 < data.length; i += 12) {
            xquad(b, m, data[i], data[i + 1], data[i + 2], data[i + 3], data[i + 4], data[i + 5],
                data[i + 6], data[i + 7], data[i + 8], data[i + 9], data[i + 10], data[i + 11], c, a);
        }
    }

    private static void xrayEdges(VertexConsumer b, Matrix4f m, Vec3 cam, float[] data, float[] c, float a) {
        for (int i = 0; i + 5 < data.length; i += 6) {
            ribbon(b, m, cam, data[i], data[i + 1], data[i + 2], data[i + 3], data[i + 4], data[i + 5], 0.7f, c, a);
        }
    }

    private static void xrayBox(VertexConsumer b, Matrix4f m, Vec3 cam,
                                double x0, double y0, double z0, double x1, double y1, double z1) {
        float[] white = {1f, 1f, 1f};
        double[][] corner = new double[8][];
        int i = 0;
        for (int dx = 0; dx <= 1; dx++) for (int dy = 0; dy <= 1; dy++) for (int dz = 0; dz <= 1; dz++)
            corner[i++] = new double[] {dx == 0 ? x0 : x1, dy == 0 ? y0 : y1, dz == 0 ? z0 : z1};
        int[][] edges = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
        for (int[] e : edges) {
            double[] p = corner[e[0]], q = corner[e[1]];
            ribbon(b, m, cam, p[0], p[1], p[2], q[0], q[1], q[2], 0.8f, white, 1f);
        }
    }

    private static void addQuads(VertexConsumer quads, Matrix4f pose, float[] data, float[] color, float alpha) {
        for (int i = 0; i + 11 < data.length; i += 12) {
            for (int v = 0; v < 4; v++) {
                quads.addVertex(pose, data[i + v * 3], data[i + v * 3 + 1], data[i + v * 3 + 2])
                    .setColor(color[0], color[1], color[2], alpha);
            }
        }
    }

    private static void addEdges(VertexConsumer lines, Matrix4f pose, float[] data, float[] color, float alpha) {
        for (int i = 0; i + 5 < data.length; i += 6) {
            float x0 = data[i], y0 = data[i + 1], z0 = data[i + 2], x1 = data[i + 3], y1 = data[i + 4], z1 = data[i + 5];
            float ex = x1 - x0, ey = y1 - y0, ez = z1 - z0;
            float len = (float) Math.sqrt(ex * ex + ey * ey + ez * ez);
            if (len < 1.0e-6f) continue;
            ex /= len; ey /= len; ez /= len;
            lines.addVertex(pose, x0, y0, z0).setColor(color[0], color[1], color[2], alpha).setNormal(ex, ey, ez).setLineWidth(LINE_WIDTH);
            lines.addVertex(pose, x1, y1, z1).setColor(color[0], color[1], color[2], alpha).setNormal(ex, ey, ez).setLineWidth(LINE_WIDTH);
        }
    }

    private static void addLine(VertexConsumer lines, Matrix4f pose,
                                 double[] from, double[] to, float r, float g, float b, float a) {
        float x1 = (float) from[0], y1 = (float) from[1], z1 = (float) from[2];
        float x2 = (float) to[0], y2 = (float) to[1], z2 = (float) to[2];
        float nx = x2 - x1, ny = y2 - y1, nz = z2 - z1;
        float len = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1.0e-4f) {
            nx /= len; ny /= len; nz /= len;
        } else {
            nx = 0; ny = 1; nz = 0;
        }
        lines.addVertex(pose, x1, y1, z1).setColor(r, g, b, a).setNormal(nx, ny, nz).setLineWidth(LINE_WIDTH);
        lines.addVertex(pose, x2, y2, z2).setColor(r, g, b, a).setNormal(nx, ny, nz).setLineWidth(LINE_WIDTH);
    }

    /** Axis-aligned wireframe box between two opposite corners; each edge carries its own direction normal. */
    private static void addBox(VertexConsumer lines, Matrix4f pose,
                               double x0, double y0, double z0, double x1, double y1, double z1,
                               float r, float g, float b, float a) {
        float[][] corners = new float[8][];
        int idx = 0;
        for (int dx = 0; dx <= 1; dx++) {
            for (int dy = 0; dy <= 1; dy++) {
                for (int dz = 0; dz <= 1; dz++) {
                    corners[idx++] = new float[] {
                        (float) (dx == 0 ? x0 : x1), (float) (dy == 0 ? y0 : y1), (float) (dz == 0 ? z0 : z1)};
                }
            }
        }
        // corner index bits: bit2=dx, bit1=dy, bit0=dz
        int[][] edges = {
            {0, 1}, {2, 3}, {4, 5}, {6, 7}, // along z
            {0, 2}, {1, 3}, {4, 6}, {5, 7}, // along y
            {0, 4}, {1, 5}, {2, 6}, {3, 7}, // along x
        };
        for (int[] edge : edges) {
            float[] p1 = corners[edge[0]];
            float[] p2 = corners[edge[1]];
            float ex = p2[0] - p1[0], ey = p2[1] - p1[1], ez = p2[2] - p1[2];
            float len = (float) Math.sqrt(ex * ex + ey * ey + ez * ez);
            if (len < 1.0e-6f) continue;
            ex /= len; ey /= len; ez /= len;
            lines.addVertex(pose, p1[0], p1[1], p1[2]).setColor(r, g, b, a).setNormal(ex, ey, ez).setLineWidth(LINE_WIDTH);
            lines.addVertex(pose, p2[0], p2[1], p2[2]).setColor(r, g, b, a).setNormal(ex, ey, ez).setLineWidth(LINE_WIDTH);
        }
    }
}
