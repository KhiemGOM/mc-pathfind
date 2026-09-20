package dev.netherpathfinder.client;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import dev.netherpathfinder.engine.Action;
import dev.netherpathfinder.engine.EdgeRules;
import net.fabricmc.fabric.api.client.rendering.v1.level.LevelRenderContext;
import net.minecraft.client.Minecraft;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.client.player.LocalPlayer;
import net.minecraft.client.renderer.rendertype.RenderType;
import net.minecraft.client.renderer.rendertype.RenderTypes;
import net.minecraft.core.BlockPos;
import net.minecraft.resources.Identifier;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.Vec3;
import org.joml.Matrix3f;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

/**
 * Floating, camera-facing icons that label what the route asks of you.
 *
 * Far away (between iconMinDistance and iconMaxDistance) an icon floats at eye height at the START of
 * its action, just before the first block still to dig / place / jump from. Once you are within
 * iconMinDistance it stops floating and goes onto the work itself:
 *   - digging: a pickaxe lying flat ON the face of each of the next blocks to mine (the next column's two);
 *   - bridging: a block icon lying flat on TOP of the next block to place (the nearest one you are not
 *     standing on), level with where you will stand;
 *   - jumping (plain or boat): the icon moves to the middle of the jump, low, so it never covers the
 *     landing.
 * Dig and bridge targets come from the real world (blocks already mined or placed drop out), so the
 * icon always points at what is left. Crawls and the goal keep the simple floating icon.
 *
 * Each icon is a plain flat sprite, drawn normally or through terrain (iconThroughWalls).
 * Purely a route overlay -- it only draws.
 */
final class RouteIcons {
    private RouteIcons() {}

    enum Kind { PICKAXE, BRIDGE_BLOCK, JUMP, BOAT, GOAL }

    record Marker(Kind kind, double x, double y, double z, int node) {}

    private static final double EYE_HEIGHT = 1.62;        // far icons float at player eye level over the route
    private static final double BASE_SCALE = 0.6;
    private static final double SCALE_PER_BLOCK = 0.03;   // grows a little with distance so far icons stay legible
    private static final double MAX_SCALE = 1.2;
    private static final float ON_BLOCK_SCALE = 0.85f;    // block-sized when sitting on a block
    private static final double MID_JUMP_DROP = 0.5;      // mid-jump icon sits this far below the lower end
    private static final double LANDED_HORIZONTAL = 1.6;  // within this of the landing (and level) = jump done
    private static final int FULL_BRIGHT = 0xF000F0;
    private static final float DECAL_GAP = 0.01f;         // how far a surface icon sits off the block face (no z-fighting)

    private static final Identifier PICKAXE_SPRITE = Identifier.withDefaultNamespace("textures/item/iron_pickaxe.png");
    private static final Identifier BRIDGE_SPRITE = Identifier.withDefaultNamespace("textures/block/cobblestone.png");
    private static final Identifier JUMP_SPRITE = Identifier.withDefaultNamespace("textures/mob_effect/jump_boost.png");
    private static final Identifier BOAT_SPRITE = Identifier.withDefaultNamespace("textures/item/oak_boat.png");
    private static final Identifier GOAL_SPRITE = Identifier.withDefaultNamespace("textures/block/gold_block.png");

    private static Identifier spriteFor(Kind kind) {
        return switch (kind) {
            case PICKAXE -> PICKAXE_SPRITE;
            case BRIDGE_BLOCK -> BRIDGE_SPRITE;
            case JUMP -> JUMP_SPRITE;
            case BOAT -> BOAT_SPRITE;
            case GOAL -> GOAL_SPRITE;
        };
    }

    // ---- analysis: a route -> runs of work ------------------------------------------------------

    enum Group { NONE, DIG, BRIDGE, CRAWL, PARKOUR }

    /** One stretch of the route the icons care about. */
    static final class Run {
        final Group group;
        final int startNode;      // node index where the action begins
        final int[][] cells;      // DIG / BRIDGE targets in route order: {x, y, z, step}; step = node stood on before it
        final double[] from, to;  // PARKOUR takeoff / landing
        final boolean boat;       // PARKOUR only: committed boat jump rather than a plain hop

        Run(Group group, int startNode, int[][] cells, double[] from, double[] to, boolean boat) {
            this.group = group;
            this.startNode = startNode;
            this.cells = cells;
            this.from = from;
            this.to = to;
            this.boat = boat;
        }
    }

    static final class Analysis {
        final List<Run> runs = new ArrayList<>();
        final List<Marker> simple = new ArrayList<>(); // crawls and the goal: floating icon only
    }

    private static RouteState.Route cachedRoute;
    private static Analysis cachedAnalysis = new Analysis();

    private static Group classOf(Action a) {
        if (a == null) return Group.NONE;
        return switch (a) {
            case MINE, MINE_DOWN -> Group.DIG;
            case BRIDGE, BRIDGE_UP -> Group.BRIDGE;
            case BOAT_CRAWL -> Group.CRAWL;
            case PARKOUR -> Group.PARKOUR;
            default -> Group.NONE;
        };
    }

    /** Blocks a step digs at its destination: a 2-tall tunnel for MINE, 1 for MINE_DOWN. */
    private static int digHeight(Action a) {
        return a == Action.MINE ? 2 : a == Action.MINE_DOWN ? 1 : 0;
    }

    /**
     * Mirrors EdgeRules' PARKOUR cost split: a short hop (flat under sqrt(13), one up under sqrt(5),
     * one down under sqrt(20) horizontal blocks) is a plain jump; anything longer is a committed
     * boat jump.
     */
    static boolean isBoatJump(double[] from, double[] to) {
        double dist = Math.hypot(to[0] - from[0], to[2] - from[2]);
        long dy = Math.round(to[1] - from[1]);
        boolean cheap = (dy == 0 && dist < EdgeRules.PARKOUR_CHEAP_MAX_DIST_FLAT)
            || (dy == 1 && dist < EdgeRules.PARKOUR_CHEAP_MAX_DIST_UP)
            || (dy == -1 && dist < EdgeRules.PARKOUR_CHEAP_MAX_DIST_DOWN);
        return !cheap;
    }

    static Analysis analyze(RouteState.Route route) {
        if (route == cachedRoute) return cachedAnalysis;
        Analysis out = new Analysis();
        double[][] p = route.points;
        Action[] actions = route.actions;
        int n = p.length;
        int steps = Math.min(n - 1, actions.length);
        int i = 0;
        while (i < steps) {
            Group g = classOf(actions[i]);
            if (g == Group.NONE) { i++; continue; }
            if (g == Group.PARKOUR) {
                out.runs.add(new Run(g, i, null, p[i], p[i + 1], isBoatJump(p[i], p[i + 1])));
                i++;
                continue;
            }
            int j = i;
            while (j + 1 < steps && classOf(actions[j + 1]) == g) j++;
            if (g == Group.CRAWL) {
                out.simple.add(new Marker(Kind.BOAT, p[i][0], p[i][1], p[i][2], i));
            } else {
                List<int[]> cells = new ArrayList<>();
                for (int s = i; s <= j; s++) {
                    int x = (int) Math.floor(p[s + 1][0]);
                    int y = (int) Math.floor(p[s + 1][1]);
                    int z = (int) Math.floor(p[s + 1][2]);
                    if (g == Group.DIG) {
                        for (int k = 0; k < digHeight(actions[s]); k++) cells.add(new int[] {x, y + k, z, s});
                    } else {
                        cells.add(new int[] {x, y - 1, z, s}); // the block placed under the destination
                    }
                }
                out.runs.add(new Run(g, i, cells.toArray(new int[0][]), null, null, false));
            }
            i = j + 1;
        }
        if (n > 0) out.simple.add(new Marker(Kind.GOAL, p[n - 1][0], p[n - 1][1], p[n - 1][2], n - 1));
        cachedRoute = route;
        cachedAnalysis = out;
        return out;
    }

    // ---- target selection (pure: the world is reached only through `pending`) ------------------

    /** What is left of a dig/bridge run, and which of it to mark when the player is close. */
    record Targets(List<int[]> remaining, List<int[]> nearest) {}

    /**
     * @param pending  true if the cell still needs work (still solid to mine / still empty to fill)
     * @param standing the cell directly under the player's feet; never chosen as a bridge target
     */
    static Targets select(Run run, Predicate<int[]> pending, double px, double py, double pz, int[] standing) {
        List<int[]> remaining = new ArrayList<>();
        for (int[] cell : run.cells) {
            if (run.group == Group.BRIDGE && standing != null
                && cell[0] == standing[0] && cell[1] == standing[1] && cell[2] == standing[2]) continue;
            if (pending.test(cell)) remaining.add(cell);
        }
        if (remaining.isEmpty()) return new Targets(remaining, List.of());

        // Nearest to the player's feet.
        int[] best = null;
        double bestD = Double.MAX_VALUE;
        for (int[] cell : remaining) {
            double d = sq(cell[0] + 0.5 - px) + sq(cell[1] + 0.5 - py) + sq(cell[2] + 0.5 - pz);
            if (d < bestD) { bestD = d; best = cell; }
        }
        List<int[]> nearest = new ArrayList<>();
        if (run.group == Group.BRIDGE) {
            nearest.add(best); // one block at a time
        } else {
            for (int[] cell : remaining) if (cell[3] == best[3]) nearest.add(cell); // the whole next column
        }
        return new Targets(remaining, nearest);
    }

    private static double sq(double v) { return v * v; }

    // ---- drawing -------------------------------------------------------------------------------

    /**
     * Draws the icons for every part of the route that has settled. Call from the collect-submits
     * phase with the pose stack in its plain (camera-origin) state; each icon is positioned relative
     * to the camera here.
     */
    static void render(LevelRenderContext context, RouteState.Route route, int settledNodes) {
        Minecraft mc = Minecraft.getInstance();
        ClientLevel level = mc.level;
        if (level == null) return;
        var settings = PathfindCommands.settings();
        double minDistance = settings == null ? 2.5 : settings.get("iconMinDistance");
        double maxDistance = settings == null ? 16.0 : settings.get("iconMaxDistance");
        boolean throughWalls = settings == null || settings.get("iconThroughWalls") >= 1;

        var camState = context.levelState().cameraRenderState;
        Vec3 cam = camState.pos;
        LocalPlayer player = mc.player;
        double px = player != null ? player.getX() : cam.x;
        double py = player != null ? player.getY() : cam.y - EYE_HEIGHT;
        double pz = player != null ? player.getZ() : cam.z;
        int[] standing = {(int) Math.floor(px), (int) Math.floor(py) - 1, (int) Math.floor(pz)};
        double seconds = System.nanoTime() / 1.0e9;
        BlockPos.MutableBlockPos probe = new BlockPos.MutableBlockPos();

        Analysis analysis = analyze(route);
        int index = 0;

        // Crawls and the goal: floating icon, hidden inside the minimum distance.
        for (Marker m : analysis.simple) {
            index++;
            if (m.node() >= settledNodes) continue;
            double d = dist(cam, m.x(), m.y() + EYE_HEIGHT, m.z());
            if (d < minDistance || d > maxDistance) continue;
            floating(context, camState, spriteFor(m.kind()), m.x(), m.y(), m.z(), d, seconds, index, throughWalls);
        }

        for (Run run : analysis.runs) {
            index++;
            if (run.startNode >= settledNodes) continue;

            if (run.group == Group.PARKOUR) {
                double[] a = run.from, b = run.to;
                boolean landed = Math.hypot(px - b[0], pz - b[2]) < LANDED_HORIZONTAL && Math.abs(py - b[1]) < 2.5;
                if (landed) continue;
                double midX = (a[0] + b[0]) / 2, midZ = (a[2] + b[2]) / 2;
                double midY = Math.min(a[1], b[1]) - MID_JUMP_DROP;
                double dFar = dist(cam, a[0], a[1] + EYE_HEIGHT, a[2]);
                double dMid = dist(cam, midX, midY, midZ);
                Kind kind = run.boat ? Kind.BOAT : Kind.JUMP;
                if (dFar < minDistance || dMid < minDistance) {
                    onSpot(context, camState, spriteFor(kind), midX, midY, midZ, ON_BLOCK_SCALE, throughWalls);
                } else if (dFar <= maxDistance) {
                    floating(context, camState, spriteFor(kind), a[0], a[1], a[2], dFar, seconds, index, throughWalls);
                }
                continue;
            }

            boolean dig = run.group == Group.DIG;
            Targets t = select(run, cell -> {
                if (!level.hasChunk(cell[0] >> 4, cell[2] >> 4)) return true; // unloaded: assume still to do
                BlockState s = level.getBlockState(probe.set(cell[0], cell[1], cell[2]));
                return dig ? !s.isAir() : s.canBeReplaced();
            }, px, py, pz, standing);
            if (t.remaining().isEmpty()) continue; // this run is finished

            int[] first = t.remaining().get(0);
            double[] stand = route.points[first[3]];
            double dFar = dist(cam, stand[0], stand[1] + EYE_HEIGHT, stand[2]);
            double dTarget = Double.MAX_VALUE;
            for (int[] cell : t.nearest()) {
                dTarget = Math.min(dTarget, dist(cam, cell[0] + 0.5, cell[1] + 0.5, cell[2] + 0.5));
            }
            Kind kind = dig ? Kind.PICKAXE : Kind.BRIDGE_BLOCK;
            if (dFar < minDistance || dTarget < minDistance) {
                for (int[] cell : t.nearest()) {
                    double cx = cell[0] + 0.5, cy = cell[1] + 0.5, cz = cell[2] + 0.5;
                    if (dig) {
                        // Lie flat on the face of the block that points toward the player.
                        double ax = cam.x - cx, ay = cam.y - cy, az = cam.z - cz;
                        double mag = Math.max(Math.abs(ax), Math.max(Math.abs(ay), Math.abs(az)));
                        int axis = mag == Math.abs(ax) ? 0 : mag == Math.abs(ay) ? 1 : 2;
                        double comp = axis == 0 ? ax : axis == 1 ? ay : az;
                        onFace(context, camState, spriteFor(kind), cx, cy, cz, axis, comp < 0 ? -1 : 1,
                            ON_BLOCK_SCALE, throughWalls);
                    } else {
                        // The block to place doesn't exist yet: mark its top surface, level with where you'll stand.
                        onFace(context, camState, spriteFor(kind), cx, cy, cz, 1, 1, ON_BLOCK_SCALE, throughWalls);
                    }
                }
            } else if (dFar <= maxDistance) {
                floating(context, camState, spriteFor(kind), stand[0], stand[1], stand[2], dFar, seconds, index, throughWalls);
            }
        }
    }

    private static double dist(Vec3 cam, double x, double y, double z) {
        return Math.sqrt(sq(x - cam.x) + sq(y - cam.y) + sq(z - cam.z));
    }

    /** A bobbing icon at eye height above a route node; grows slightly with distance. */
    private static void floating(LevelRenderContext ctx, net.minecraft.client.renderer.state.level.CameraRenderState cam,
                                 Identifier sprite, double x, double y, double z, double distance,
                                 double seconds, int index, boolean throughWalls) {
        double bob = Math.sin(seconds * 2.0 + index) * 0.08;
        float scale = (float) Math.min(MAX_SCALE, BASE_SCALE + distance * SCALE_PER_BLOCK);
        draw(ctx, cam, sprite, x, y + EYE_HEIGHT + bob, z, scale, throughWalls);
    }

    /** A steady, block-sized icon sitting exactly where the work is. */
    private static void onSpot(LevelRenderContext ctx, net.minecraft.client.renderer.state.level.CameraRenderState cam,
                               Identifier sprite, double x, double y, double z, float scale, boolean throughWalls) {
        draw(ctx, cam, sprite, x, y, z, scale, throughWalls);
    }

    private static void draw(LevelRenderContext ctx, net.minecraft.client.renderer.state.level.CameraRenderState camState,
                             Identifier sprite, double wx, double wy, double wz, float scale, boolean throughWalls) {
        Vec3 cam = camState.pos;
        PoseStack pose = ctx.poseStack();
        pose.pushPose();
        pose.translate(wx - cam.x, wy - cam.y, wz - cam.z);
        pose.mulPose(camState.orientation);
        pose.scale(scale, scale, scale);
        submitSprite(ctx, pose, sprite, 0f, throughWalls);
        pose.popPose();
    }

    /**
     * A sprite lying flat on one face of the block centered at (cx, cy, cz): axis 0/1/2 = x/y/z, sign =
     * which side. It does NOT turn to face the camera; it sits DECAL_GAP off the surface like a sticker.
     * On side faces "up" is world up; on top/bottom faces it points away from the camera so it reads upright.
     */
    private static void onFace(LevelRenderContext ctx, net.minecraft.client.renderer.state.level.CameraRenderState camState,
                               Identifier sprite, double cx, double cy, double cz, int axis, int sign,
                               float scale, boolean throughWalls) {
        Vec3 cam = camState.pos;
        Vector3f n = new Vector3f(axis == 0 ? sign : 0, axis == 1 ? sign : 0, axis == 2 ? sign : 0);
        Vector3f up = new Vector3f(0, 1, 0);
        if (axis == 1) {
            float hx = (float) (cx - cam.x), hz = (float) (cz - cam.z);
            float len = (float) Math.hypot(hx, hz);
            up = len < 1.0e-4f ? new Vector3f(0, 0, -1) : new Vector3f(hx / len, 0, hz / len);
        }
        Vector3f right = new Vector3f(up).cross(n); // local +x; with up=+y, n=+z this is +x (not mirrored)
        Quaternionf rotation = new Quaternionf().setFromNormalized(new Matrix3f(right, up, n));

        PoseStack pose = ctx.poseStack();
        pose.pushPose();
        pose.translate(cx + n.x * 0.5 - cam.x, cy + n.y * 0.5 - cam.y, cz + n.z * 0.5 - cam.z);
        pose.mulPose(rotation);
        pose.scale(scale, scale, scale);
        submitSprite(ctx, pose, sprite, DECAL_GAP / scale, throughWalls);
        pose.popPose();
    }

    /** Draws the sprite quad at the current pose; spriteZ is a local offset along the quad normal. */
    private static void submitSprite(LevelRenderContext ctx, PoseStack pose, Identifier sprite,
                                     float spriteZ, boolean throughWalls) {
        RenderType type = throughWalls ? RenderTypes.textSeeThrough(sprite) : RenderTypes.text(sprite);
        ctx.submitNodeCollector().submitCustomGeometry(pose, type, (p, buffer) ->
            quad(buffer, p.pose(), 0f, 0f, spriteZ, 1f, 1f, 1f));
    }

    /** A unit quad centered at (cx, cy) in the camera-facing plane. Supplies every element the text formats use. */
    private static void quad(VertexConsumer buffer, Matrix4f m, float cx, float cy, float z, float r, float g, float b) {
        float h = 0.5f;
        buffer.addVertex(m, cx - h, cy - h, z).setColor(r, g, b, 1f).setUv(0f, 1f).setLight(FULL_BRIGHT);
        buffer.addVertex(m, cx + h, cy - h, z).setColor(r, g, b, 1f).setUv(1f, 1f).setLight(FULL_BRIGHT);
        buffer.addVertex(m, cx + h, cy + h, z).setColor(r, g, b, 1f).setUv(1f, 0f).setLight(FULL_BRIGHT);
        buffer.addVertex(m, cx - h, cy + h, z).setColor(r, g, b, 1f).setUv(0f, 0f).setLight(FULL_BRIGHT);
        // Back face too: the text pipelines cull back faces, so a sprite seen from behind would vanish.
        buffer.addVertex(m, cx - h, cy + h, z).setColor(r, g, b, 1f).setUv(0f, 0f).setLight(FULL_BRIGHT);
        buffer.addVertex(m, cx + h, cy + h, z).setColor(r, g, b, 1f).setUv(1f, 0f).setLight(FULL_BRIGHT);
        buffer.addVertex(m, cx + h, cy - h, z).setColor(r, g, b, 1f).setUv(1f, 1f).setLight(FULL_BRIGHT);
        buffer.addVertex(m, cx - h, cy - h, z).setColor(r, g, b, 1f).setUv(0f, 1f).setLight(FULL_BRIGHT);
    }
}
