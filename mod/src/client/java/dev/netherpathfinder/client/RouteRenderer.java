package dev.netherpathfinder.client;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
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

    private static final float LINE_R = 0.2f, LINE_G = 0.9f, LINE_B = 1.0f, LINE_A = 0.9f;
    private static final float BOX_R = 1.0f, BOX_G = 0.85f, BOX_B = 0.1f, BOX_A = 0.9f;
    private static final double BOX_HALF_SIZE = 0.15;

    public static void register() {
        LevelRenderEvents.COLLECT_SUBMITS.register(RouteRenderer::render);
    }

    private static void render(LevelRenderContext context) {
        RouteState.Route route = RouteState.INSTANCE.get();
        if (route.status != RouteState.Status.FOUND || route.points.length < 1) {
            return;
        }

        Vec3 camera = context.levelState().cameraRenderState.pos;
        PoseStack poseStack = context.poseStack();

        poseStack.pushPose();
        poseStack.translate(-camera.x, -camera.y, -camera.z);
        context.submitNodeCollector().submitCustomGeometry(poseStack, RenderTypes.LINES, (pose, lines) -> {
            Matrix4f matrix = pose.pose();
            for (int i = 0; i + 1 < route.points.length; i++) {
                addLine(lines, matrix, route.points[i], route.points[i + 1], LINE_R, LINE_G, LINE_B, LINE_A);
            }
            for (double[] point : route.points) {
                addWireBox(lines, matrix, point, BOX_HALF_SIZE, BOX_R, BOX_G, BOX_B, BOX_A);
            }
        });
        poseStack.popPose();
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
        lines.addVertex(pose, x1, y1, z1).setColor(r, g, b, a).setNormal(nx, ny, nz);
        lines.addVertex(pose, x2, y2, z2).setColor(r, g, b, a).setNormal(nx, ny, nz);
    }

    private static void addWireBox(VertexConsumer lines, Matrix4f pose,
                                    double[] center, double halfSize, float r, float g, float b, float a) {
        float cx = (float) center[0];
        float cy = (float) center[1];
        float cz = (float) center[2];
        float h = (float) halfSize;

        // 8 corners of the box, then the 12 edges connecting them.
        float[][] corners = new float[8][];
        int idx = 0;
        for (int dx = -1; dx <= 1; dx += 2) {
            for (int dy = -1; dy <= 1; dy += 2) {
                for (int dz = -1; dz <= 1; dz += 2) {
                    corners[idx++] = new float[] {cx + dx * h, cy + dy * h, cz + dz * h};
                }
            }
        }
        // corner index bit layout: bit0=dx, bit1=dy, bit2=dz (matches the loop order above)
        int[][] edges = {
            {0, 1}, {2, 3}, {4, 5}, {6, 7}, // along dz
            {0, 2}, {1, 3}, {4, 6}, {5, 7}, // along dy
            {0, 4}, {1, 5}, {2, 6}, {3, 7}, // along dx
        };
        for (int[] edge : edges) {
            float[] p1 = corners[edge[0]];
            float[] p2 = corners[edge[1]];
            lines.addVertex(pose, p1[0], p1[1], p1[2]).setColor(r, g, b, a).setNormal(0, 1, 0);
            lines.addVertex(pose, p2[0], p2[1], p2[2]).setColor(r, g, b, a).setNormal(0, 1, 0);
        }
    }
}
