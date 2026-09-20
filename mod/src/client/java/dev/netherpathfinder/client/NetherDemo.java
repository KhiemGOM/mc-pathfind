package dev.netherpathfinder.client;

import dev.netherpathfinder.NetherPathfinderMod;
import dev.netherpathfinder.config.PathfinderSettings;
import dev.netherpathfinder.engine.Action;
import dev.netherpathfinder.engine.GoalPoints;
import dev.netherpathfinder.engine.MineCostModel;
import dev.netherpathfinder.engine.SearchResult;
import dev.netherpathfinder.engine.StateCodec;
import dev.netherpathfinder.terrain.TerrainCapture;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.event.lifecycle.v1.ServerTickEvents;
import net.minecraft.client.Minecraft;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.client.player.LocalPlayer;
import net.minecraft.core.BlockPos;
import net.minecraft.network.chat.Component;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.ChunkPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.chunk.LevelChunk;
import net.minecraft.world.level.storage.LevelResource;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Scripted single-player demo. Arriving in the Nether (through a portal, or by loading into it)
 * automatically finds the nearest bastion and plans a high-quality route to it:
 *
 *  - first arrival at a portal: generate the chunks between portal and bastion, capture the real
 *    terrain, run the retry ladder (see SearchLadder), cache the route, and play the reveal animation;
 *  - later arrivals at the same portal: skip the search and replay the cached route with the same
 *    reveal animation.
 *
 * Like the rest of the mod this only computes and draws a route; it never moves the player.
 * Terrain comes from the integrated server's chunks, so the bastion doesn't have to be within
 * render distance.
 */
final class NetherDemo {
    private NetherDemo() {}

    private static final int ARRIVAL_SETTLE_TICKS = 30;
    // Staged planning: a long route is searched in legs, each ending near a target point ahead of the
    // last leg, so no single search has to cross the whole distance (which is intractable past ~300 blocks).
    private static final int STAGE_LENGTH = 150;                // longest a leg is allowed to be; legs are an even split of what's left
    private static final double MIN_LEG_SCALE = 0.25;           // a blocked leg is halved down to this fraction, then we give up
    private static final int STAGE_MARGIN = 48;                 // terrain captured beyond the leg's endpoints
    private static final int MAX_ROUTE_DISTANCE = 1500;
    private static final int[] DETOUR_SHIFTS = {0, 45, -45, 90, -90}; // sideways retargets when a leg is walled off
    private static final int[] STAGE_GOAL_RADII = {16, 32, 64};
    private static final int GOAL_RADIUS = 10;
    // The heuristic scans every goal point per generated state, so keep this small (600 goals cost ~2.6x throughput).
    private static final int GOAL_MAX_POINTS = 32;
    private static final int GOAL_MAX_DY = 48;
    private static final long WARM_BUDGET_NANOS = 30_000_000L; // per server tick

    private static final ExecutorService EXECUTOR = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "netherpathfinder-demo");
        t.setDaemon(true);
        return t;
    });
    private static final AtomicLong RUN = new AtomicLong();
    private static final ConcurrentLinkedQueue<ServerJob> JOBS = new ConcurrentLinkedQueue<>();

    private static ClientLevel lastLevel;
    private static boolean arrivalPending;
    private static int arrivalTicks;

    interface ServerJob {
        /** Runs on the server thread once per tick; returns true when finished. */
        boolean step(MinecraftServer server);
    }

    static void register() {
        ClientTickEvents.END_CLIENT_TICK.register(client -> tick());
        ServerTickEvents.END_SERVER_TICK.register(server -> {
            ServerJob job = JOBS.peek();
            if (job != null && job.step(server)) JOBS.poll();
        });
    }

    // ---- arrival detection --------------------------------------------------------------------

    private static void tick() {
        Minecraft mc = Minecraft.getInstance();
        ClientLevel level = mc.level;
        LocalPlayer player = mc.player;

        if (level != lastLevel) {
            lastLevel = level;
            RUN.incrementAndGet(); // supersede any in-flight plan from the previous level
            JOBS.clear();
            RouteState.INSTANCE.clear();
            arrivalPending = level != null && level.dimension() == Level.NETHER
                && PathfindCommands.settings().snapshot().demoAutoRun();
            arrivalTicks = 0;
        }
        if (!arrivalPending || level == null || player == null) return;
        if (++arrivalTicks < ARRIVAL_SETTLE_TICKS) return;
        BlockPos pos = player.blockPosition();
        if (!level.hasChunk(pos.getX() >> 4, pos.getZ() >> 4)) return;

        arrivalPending = false;
        begin(mc, false);
    }

    // ---- entry points (also used by /pathfind demo) -------------------------------------------

    /** Plans (or replays from cache) the route for the player's current position. */
    static void begin(Minecraft mc, boolean forceReplan) {
        LocalPlayer player = mc.player;
        ClientLevel level = mc.level;
        MinecraftServer server = mc.getSingleplayerServer();
        if (player == null || level == null) return;
        if (level.dimension() != Level.NETHER) {
            say("enter the Nether first.");
            return;
        }
        if (server == null) {
            say("the demo needs a single-player world (it reads terrain from the integrated server).");
            return;
        }

        BlockPos portal = player.blockPosition();
        String worldKey = worldKey(server);
        long run = RUN.incrementAndGet();

        if (!forceReplan) {
            RouteCache.Entry hit = RouteCache.find(worldKey, portal.getX(), portal.getY(), portal.getZ());
            if (hit != null) {
                replay(hit, "replaying the cached route for this portal");
                return;
            }
        }

        say(forceReplan ? "re-planning the bastion route (this can take a while)..."
                        : "new portal -- planning the bastion route (this can take a while)...");
        PathfinderSettings.Snapshot cfg = PathfindCommands.settings().snapshot();
        InventoryResources.Snapshot inventory = InventoryResources.capture(player);
        int blocks = Math.max(inventory.bridgeBlocks(), cfg.demoBridgeBlocks());
        MineCostModel.PickaxeTier demoTier = MineCostModel.PickaxeTier.values()[
            Math.min(cfg.demoPickaxeTier(), MineCostModel.PickaxeTier.values().length - 1)];
        MineCostModel.PickaxeTier tier = demoTier.preferredOver(inventory.pickaxeTier(), false)
            ? demoTier : inventory.pickaxeTier();

        RouteState.INSTANCE.set(new RouteState.Route(new double[0][], new Action[0],
            RouteState.Status.SEARCHING, null));

        // Structure lookup and chunk generation must happen on the server thread.
        server.execute(() -> {
            if (RUN.get() != run) return;
            ServerLevel nether = server.getLevel(Level.NETHER);
            BlockPos bastion = nether == null ? null : PathfindCommands.locateNearestBastion(nether, portal);
            if (bastion == null) {
                fail(run, "no bastion found near this portal.");
                return;
            }
            double distance = Math.hypot(bastion.getX() - portal.getX(), bastion.getZ() - portal.getZ());
            if (distance > MAX_ROUTE_DISTANCE) {
                fail(run, "the nearest bastion is too far for the demo (" + (int) distance + " blocks).");
                return;
            }
            Plan plan = new Plan(run, server, nether, portal, bastion, blocks, tier, cfg, worldKey, distance);
            say(String.format(java.util.Locale.ROOT,
                "bastion at %d, %d (%d blocks away); planning in %d leg(s) of about %d blocks...",
                bastion.getX(), bastion.getZ(), (int) distance, plan.stageEstimate, STAGE_LENGTH));
            startStage(plan);
        });
    }

    static void replayCurrent(Minecraft mc) {
        RouteState.Route route = RouteState.INSTANCE.get();
        if (route.status != RouteState.Status.FOUND || route.points.length < 2) {
            say("no route to replay yet.");
            return;
        }
        RouteState.INSTANCE.set(new RouteState.Route(route.points, route.actions, route.status,
            route.message, System.nanoTime()));
    }

    static void clearCache(Minecraft mc) {
        MinecraftServer server = mc.getSingleplayerServer();
        if (server == null) {
            say("open a single-player world first.");
            return;
        }
        int removed = RouteCache.clear(worldKey(server));
        RouteState.INSTANCE.clear();
        say("cleared " + removed + " cached route(s) for this world.");
    }

    // ---- planning ----------------------------------------------------------------------------

    /** State of one staged planning run. Mutated only by the leg that is currently running. */
    private static final class Plan {
        final long run;
        final MinecraftServer server;
        final ServerLevel level;
        final BlockPos portal, bastion;
        final MineCostModel.PickaxeTier tier;
        final PathfinderSettings.Snapshot cfg;
        final String worldKey;
        final int stageEstimate;
        final long deadlineNanos;
        final long startedNanos = System.nanoTime();
        final List<double[]> points = new ArrayList<>();
        final List<Action> actions = new ArrayList<>();
        int curX, curY, curZ;
        int blocksLeft;
        int stage = 0;        // completed legs
        int retry = 0;        // sideways-detour attempts on the current leg
        double legScale = 1.0; // halved when a leg can't be crossed even with detours
        long totalExpansions = 0;
        double totalCost = 0;
        double maxWeight = 0;

        Plan(long run, MinecraftServer server, ServerLevel level, BlockPos portal, BlockPos bastion, int blocks,
             MineCostModel.PickaxeTier tier, PathfinderSettings.Snapshot cfg, String worldKey, double distance) {
            this.run = run;
            this.server = server;
            this.level = level;
            this.portal = portal;
            this.bastion = bastion;
            this.tier = tier;
            this.cfg = cfg;
            this.worldKey = worldKey;
            this.blocksLeft = blocks;
            this.curX = portal.getX();
            this.curY = portal.getY();
            this.curZ = portal.getZ();
            this.stageEstimate = Math.max(1, (int) Math.ceil(distance / STAGE_LENGTH));
            this.deadlineNanos = System.nanoTime() + Math.round(cfg.demoTimeoutSeconds() * 1.0e9);
        }

        String label() { return "Leg " + (stage + 1) + "/" + Math.max(stageEstimate, stage + 1); }
    }

    /** Picks this leg's target, then has the server generate the terrain it needs. Any thread. */
    private static void startStage(Plan p) {
        double dx = p.bastion.getX() - p.curX, dz = p.bastion.getZ() - p.curZ;
        double dist = Math.hypot(dx, dz);
        // Even split of the remaining distance into legs no longer than STAGE_LENGTH, shortened if a leg
        // has already been blocked. A leg that reaches the bastion is the final one.
        int legsLeft = Math.max(1, (int) Math.ceil(dist / STAGE_LENGTH));
        double legLength = dist / legsLeft * p.legScale;
        boolean finalLeg = legLength >= dist - 1;
        int tx, tz;
        if (finalLeg) {
            tx = p.bastion.getX();
            tz = p.bastion.getZ();
        } else {
            double ux = dx / dist, uz = dz / dist;          // toward the bastion
            double shift = DETOUR_SHIFTS[p.retry];           // sideways, when a straight leg was blocked
            tx = (int) Math.round(p.curX + ux * legLength - uz * shift);
            tz = (int) Math.round(p.curZ + uz * legLength + ux * shift);
        }
        int radius = (int) Math.min(220, Math.hypot(tx - p.curX, tz - p.curZ) / 2 + STAGE_MARGIN);
        BlockPos center = new BlockPos((p.curX + tx) / 2, p.curY, (p.curZ + tz) / 2);
        final int fx = tx, fz = tz;
        p.server.execute(() -> {
            if (RUN.get() != p.run) return;
            JOBS.add(new WarmJob(p.level, center, radius, p.run, chunks ->
                EXECUTOR.submit(() -> searchStage(p, chunks, center, radius, fx, fz, finalLeg))));
        });
    }

    /** Searches one leg on the demo executor thread. */
    private static void searchStage(Plan p, Map<Long, LevelChunk> chunks, BlockPos center, int radius,
                                    int tx, int tz, boolean finalLeg) {
        try {
            if (RUN.get() != p.run) return;
            status(p.label() + " | capturing terrain...");
            TerrainCapture.Result capture = TerrainCapture.capture(
                TerrainCapture.of(chunks, p.level.getMinY(), p.level.getMaxY()), center, radius);
            if (RUN.get() != p.run) return;

            int sx = capture.toLocalX(p.curX), sy = capture.toLocalY(p.curY), sz = capture.toLocalZ(p.curZ);
            if (!inBounds(capture, sx, sy, sz)) {
                fail(p.run, "leg start is outside the captured terrain.");
                return;
            }
            GoalPoints goal = finalLeg
                ? goalNear(capture, tx, tz, p.curY, new int[] {GOAL_RADIUS})
                : goalNear(capture, tx, tz, p.curY, STAGE_GOAL_RADII);
            if (goal == null) {
                LegDump.write(p.worldKey, p.stage + 1, p.retry, capture, sx, sy, sz, null, p.blocksLeft, "no standable ground near the target");
                retryOrFail(p, finalLeg, "no standable ground near this leg's target");
                return;
            }

            double remaining = (p.deadlineNanos - System.nanoTime()) / 1.0e9;
            if (remaining < 5) {
                fail(p.run, String.format(java.util.Locale.ROOT, "no route within %.0f seconds.", p.cfg.demoTimeoutSeconds()));
                return;
            }
            PathfinderSettings.Snapshot legCfg = withTimeout(p.cfg, remaining);
            PathfindCommands.applySettings(p.cfg);
            String prefix = p.label() + " | ";
            SearchLadder.Outcome outcome = SearchLadder.run(capture.world,
                new StateCodec.State(sx, sy, sz, p.blocksLeft, false), goal, p.blocksLeft,
                MineCostModel.forTier(p.tier), legCfg, m -> {},
                text -> status(prefix + text.replaceFirst("^Bastion route \\| ", "")));
            if (RUN.get() != p.run) return;
            p.totalExpansions += outcome.totalExpansions();

            SearchResult result = outcome.result();
            if (result == null || !result.found()) {
                LegDump.write(p.worldKey, p.stage + 1, p.retry, capture, sx, sy, sz, goal, p.blocksLeft,
                    outcome.timedOut() ? "timed out" : String.format(java.util.Locale.ROOT, "no path (%,d expansions)", outcome.totalExpansions()));
                if (outcome.timedOut()) {
                    fail(p.run, String.format(java.util.Locale.ROOT, "no route within %.0f seconds (%,d expansions, %d leg(s) done).",
                        p.cfg.demoTimeoutSeconds(), p.totalExpansions, p.stage));
                } else {
                    retryOrFail(p, finalLeg, String.format(java.util.Locale.ROOT,
                        "no path for this leg (%,d expansions)", outcome.totalExpansions()));
                }
                return;
            }

            StateCodec.State[] path = result.path();
            int from = p.stage == 0 ? 0 : 1; // later legs start on the node the last one ended on
            for (int i = from; i < path.length; i++) {
                p.points.add(new double[] {
                    capture.toWorldX(path[i].x()) + 0.5,
                    capture.toWorldY(path[i].y()),
                    capture.toWorldZ(path[i].z()) + 0.5});
            }
            for (Action a : result.actions()) p.actions.add(a);
            StateCodec.State end = path[path.length - 1];
            p.curX = capture.toWorldX(end.x());
            p.curY = capture.toWorldY(end.y());
            p.curZ = capture.toWorldZ(end.z());
            p.blocksLeft = end.blocksRemaining();
            p.totalCost += result.totalCost();
            p.maxWeight = Math.max(p.maxWeight, outcome.weight());
            p.stage++;
            p.retry = 0;
            p.legScale = 1.0; // the next leg starts at full length again

            if (finalLeg) finish(p);
            else startStage(p);
        } catch (CancellationException e) {
            // superseded by a newer run or a dimension change
        } catch (Throwable t) {
            NetherPathfinderMod.LOGGER.error("Nether demo planning failed", t);
            fail(p.run, "planning failed: " + t);
        }
    }

    /**
     * A blocked leg is re-aimed sideways first; if every detour fails it is halved (coarse-to-fine) and
     * tried again, and only when even a quarter-length leg is blocked does the whole plan give up.
     */
    private static void retryOrFail(Plan p, boolean finalLeg, String reason) {
        if (!finalLeg && p.retry + 1 < DETOUR_SHIFTS.length) {
            p.retry++;
            say(reason + "; trying a detour (" + p.retry + "/" + (DETOUR_SHIFTS.length - 1) + ")...");
            startStage(p);
        } else if (p.legScale * 0.5 >= MIN_LEG_SCALE) {
            p.legScale *= 0.5;
            p.retry = 0;
            say(reason + "; splitting the leg and trying a shorter one...");
            startStage(p);
        } else {
            fail(p.run, reason + "; giving up after " + p.stage + " leg(s).");
        }
    }

    private static void finish(Plan p) {
        double seconds = (System.nanoTime() - p.startedNanos) / 1.0e9;
        double[][] points = p.points.toArray(new double[0][]);
        Action[] actions = p.actions.toArray(new Action[0]);
        double[] last = points[points.length - 1];
        RouteCache.Entry entry = RouteCache.newEntry(p.portal.getX(), p.portal.getY(), p.portal.getZ(),
            (int) Math.floor(last[0]), (int) Math.floor(last[1]), (int) Math.floor(last[2]),
            points, actions, p.maxWeight, p.totalCost, (int) Math.min(Integer.MAX_VALUE, p.totalExpansions));
        RouteCache.put(p.worldKey, entry);

        String message = String.format(java.util.Locale.ROOT,
            "route found: %d waypoints in %d leg(s), cost %.1f, weight up to %.2f, %,d expansions, %.1fs -- cached for this portal",
            points.length, p.stage, p.totalCost, p.maxWeight, p.totalExpansions, seconds);
        RouteState.INSTANCE.set(new RouteState.Route(points, actions, RouteState.Status.FOUND, message, System.nanoTime()));
        say(message);
        status(String.format(java.util.Locale.ROOT, "Bastion route ready | %d waypoints | %d leg(s) | %,d expansions | %.0fs",
            points.length, p.stage, p.totalExpansions, seconds));
    }

    private static PathfinderSettings.Snapshot withTimeout(PathfinderSettings.Snapshot c, double seconds) {
        return new PathfinderSettings.Snapshot(c.weight(), c.timeoutSeconds(), c.maxExpansions(), c.radius(),
            c.sprintSpeed(), c.bridgeSpeed(), c.placeTime(), c.demoAutoRun(), c.ladderStartWeight(),
            c.demoMaxExpansions(), seconds, c.demoBridgeBlocks(), c.demoPickaxeTier());
    }

    /**
     * Up to GOAL_MAX_POINTS standable cells nearest the target (x, z), widening through `radii` until
     * some exist. Few points on purpose: the heuristic scans every goal per generated state.
     */
    private static GoalPoints goalNear(TerrainCapture.Result capture, int tx, int tz, int nearY, int[] radii) {
        int cx = capture.toLocalX(tx), cz = capture.toLocalZ(tz);
        int centerY = capture.toLocalY(nearY);
        for (int radius : radii) {
            List<int[]> cells = new ArrayList<>();
            for (int dx = -radius; dx <= radius; dx++) {
                for (int dz = -radius; dz <= radius; dz++) {
                    int x = cx + dx, z = cz + dz;
                    if (x < 0 || x >= capture.world.sizeX || z < 0 || z >= capture.world.sizeZ) continue;
                    for (int y : capture.world.standableColumnYs(x, z)) {
                        if (Math.abs(y - centerY) <= GOAL_MAX_DY) cells.add(new int[] {x, y, z, dx * dx + dz * dz});
                    }
                }
            }
            if (cells.isEmpty()) continue;
            cells.sort((a, b) -> Integer.compare(a[3], b[3]));
            int n = Math.min(cells.size(), GOAL_MAX_POINTS);
            int[] xs = new int[n], ys = new int[n], zs = new int[n];
            for (int i = 0; i < n; i++) {
                xs[i] = cells.get(i)[0];
                ys[i] = cells.get(i)[1];
                zs[i] = cells.get(i)[2];
            }
            return new GoalPoints(xs, ys, zs);
        }
        return null;
    }

    private static void replay(RouteCache.Entry entry, String note) {
        RouteState.INSTANCE.set(new RouteState.Route(entry.points, RouteCache.actionsOf(entry),
            RouteState.Status.FOUND,
            String.format(java.util.Locale.ROOT, "cached route: %d waypoints, cost %.1f, weight %.2f",
                entry.points.length, entry.cost, entry.weight),
            System.nanoTime()));
        say(note + " (" + entry.points.length + " waypoints).");
    }

    // ---- server-thread chunk generation --------------------------------------------------------

    /** Generates (or loads) every chunk the search needs, a few per server tick, then hands them over. */
    private static final class WarmJob implements ServerJob {
        private final ServerLevel level;
        private final int minCx, minCz, width, total;
        private final long run;
        private final java.util.function.Consumer<Map<Long, LevelChunk>> onDone;
        private final Map<Long, LevelChunk> chunks = new ConcurrentHashMap<>();
        private int next;

        WarmJob(ServerLevel level, BlockPos center, int radius, long run,
                java.util.function.Consumer<Map<Long, LevelChunk>> onDone) {
            this.level = level;
            this.run = run;
            this.onDone = onDone;
            this.minCx = (center.getX() - radius) >> 4;
            this.minCz = (center.getZ() - radius) >> 4;
            int maxCx = (center.getX() + radius) >> 4;
            int maxCz = (center.getZ() + radius) >> 4;
            this.width = maxCx - minCx + 1;
            this.total = width * (maxCz - minCz + 1);
        }

        @Override
        public boolean step(MinecraftServer server) {
            if (RUN.get() != run) return true;
            long deadline = System.nanoTime() + WARM_BUDGET_NANOS;
            do {
                int cx = minCx + next % width;
                int cz = minCz + next / width;
                chunks.put(ChunkPos.pack(cx, cz), level.getChunk(cx, cz));
                next++;
            } while (next < total && System.nanoTime() < deadline);

            if (next % 8 == 0 || next >= total) {
                status(String.format(java.util.Locale.ROOT, "Bastion route | generating terrain: %d / %d chunks", next, total));
            }
            if (next >= total) {
                onDone.accept(chunks);
                return true;
            }
            return false;
        }
    }

    // ---- helpers -------------------------------------------------------------------------------

    private static boolean inBounds(TerrainCapture.Result capture, int x, int y, int z) {
        return x >= 0 && x < capture.world.sizeX && y >= 0 && y < capture.world.sizeY
            && z >= 0 && z < capture.world.sizeZ;
    }

    private static String worldKey(MinecraftServer server) {
        var name = server.getWorldPath(LevelResource.ROOT).toAbsolutePath().normalize().getFileName();
        return name == null ? "world" : name.toString();
    }

    private static void fail(long run, String message) {
        if (RUN.get() != run) return;
        RouteState.INSTANCE.set(new RouteState.Route(new double[0][], new Action[0],
            RouteState.Status.NOT_FOUND, message));
        say(message);
    }

    /** Live progress line on the action bar, from any thread. */
    static void status(String message) {
        Minecraft mc = Minecraft.getInstance();
        mc.execute(() -> {
            LocalPlayer player = mc.player;
            if (player != null) player.sendOverlayMessage(Component.literal(message));
        });
    }

    /** Chat feedback from any thread. */
    static void say(String message) {
        Minecraft mc = Minecraft.getInstance();
        mc.execute(() -> {
            LocalPlayer player = mc.player;
            if (player != null) player.sendSystemMessage(Component.literal("[nether-pathfinder] " + message));
        });
    }
}
