package dev.netherpathfinder.client;

import com.mojang.brigadier.CommandDispatcher;
import com.mojang.brigadier.arguments.IntegerArgumentType;
import com.mojang.brigadier.arguments.StringArgumentType;
import dev.netherpathfinder.NetherPathfinderMod;
import dev.netherpathfinder.config.PathfinderSettings;
import dev.netherpathfinder.engine.EdgeRules;
import dev.netherpathfinder.engine.GoalPoints;
import dev.netherpathfinder.engine.SearchResult;
import dev.netherpathfinder.engine.StateCodec;
import dev.netherpathfinder.engine.WeightedAStar;
import dev.netherpathfinder.terrain.TerrainCapture;
import net.fabricmc.fabric.api.client.command.v2.ClientCommandRegistrationCallback;
import net.fabricmc.fabric.api.client.command.v2.FabricClientCommandSource;
import net.fabricmc.loader.api.FabricLoader;
import net.minecraft.client.Minecraft;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.client.player.LocalPlayer;
import net.minecraft.commands.CommandBuildContext;
import net.minecraft.core.BlockPos;
import net.minecraft.core.HolderSet;
import net.minecraft.core.registries.Registries;
import net.minecraft.network.chat.Component;
import net.minecraft.resources.Identifier;
import net.minecraft.resources.ResourceKey;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.levelgen.structure.Structure;

import java.io.IOException;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import com.mojang.datafixers.util.Pair;

import static net.fabricmc.fabric.api.client.command.v2.ClientCommands.argument;
import static net.fabricmc.fabric.api.client.command.v2.ClientCommands.literal;

/**
 * The /pathfind command tree. Baritone-style -- every route
 * starts from an explicit command, nothing runs automatically. A search
 * only computes and stores a route (see RouteState); it never moves,
 * mines, or places anything for the player.
 */
public final class PathfindCommands {
    private PathfindCommands() {}

    // Search terrain is captured out to this many blocks horizontally from
    // the midpoint of player and target, full level height vertically.
    // Capped well below StateCodec's 16-bit coordinate range so a distant
    // target can't silently request a multi-hundred-MB capture.
    private static final int MAX_HORIZONTAL_RADIUS = 256;
    private static final int CAPTURE_MARGIN = 24;
    private static final int BASTION_PROBE_SPACING = 432;
    private static final int BASTION_PROBE_RADIUS_CHUNKS = 27;
    private static final int BASTION_FALLBACK_RADIUS_CHUNKS = 128;

    private static final ExecutorService SEARCH_EXECUTOR = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "netherpathfinder-search");
        t.setDaemon(true);
        return t;
    });
    private static final ScheduledExecutorService SEARCH_TIMEOUTS = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread t = new Thread(r, "netherpathfinder-timeout");
        t.setDaemon(true);
        return t;
    });
    private static final AtomicLong SEARCH_GENERATION = new AtomicLong();
    private static final AtomicReference<Future<?>> ACTIVE_SEARCH = new AtomicReference<>();
    private static PathfinderSettings settings;

    static PathfinderSettings settings() { return settings; }

    public static void register() {
        try {
            settings = new PathfinderSettings(FabricLoader.getInstance().getConfigDir()
                .resolve("nether-pathfinder.properties"));
        } catch (IOException e) {
            throw new IllegalStateException("Could not load Nether Pathfinder settings", e);
        }
        ClientCommandRegistrationCallback.EVENT.register(PathfindCommands::registerAll);
    }

    private static void registerAll(CommandDispatcher<FabricClientCommandSource> dispatcher, CommandBuildContext buildContext) {
        dispatcher.register(literal("pathfind")
            .then(literal("goto")
                .then(argument("x", IntegerArgumentType.integer())
                    .then(argument("y", IntegerArgumentType.integer())
                        .then(argument("z", IntegerArgumentType.integer())
                            .executes(ctx -> {
                                int x = IntegerArgumentType.getInteger(ctx, "x");
                                int y = IntegerArgumentType.getInteger(ctx, "y");
                                int z = IntegerArgumentType.getInteger(ctx, "z");
                                return runGoto(ctx.getSource(), new BlockPos(x, y, z));
                            })))))
            .then(literal("stop")
                .executes(ctx -> {
                    stopActiveSearch();
                    RouteState.INSTANCE.clear();
                    ctx.getSource().sendFeedback(Component.literal("[nether-pathfinder] route cleared."));
                    return 1;
                }))
            .then(literal("bastion")
                .executes(ctx -> runBastion(ctx.getSource())))
            .then(literal("xray")
                // /pathfind xray          -- cycle: off -> falling -> full
                // /pathfind xray off      -- normal depth-tested route
                // /pathfind xray falling  -- pieces show through terrain while they fall in, then settle normally
                // /pathfind xray full     -- the whole route always shows through terrain
                .executes(ctx -> setXray(ctx.getSource(), null))
                .then(literal("off").executes(ctx -> setXray(ctx.getSource(), 0)))
                .then(literal("falling").executes(ctx -> setXray(ctx.getSource(), 1)))
                .then(literal("on").executes(ctx -> setXray(ctx.getSource(), 1)))
                .then(literal("full").executes(ctx -> setXray(ctx.getSource(), 2))))
            .then(literal("demo")
                // /pathfind demo          -- plan (or replay the cached route) for the current position
                // /pathfind demo replan   -- ignore the cache and search again
                // /pathfind demo replay   -- replay the reveal animation of the current route
                // /pathfind demo clear    -- forget this world's cached routes
                .executes(ctx -> { NetherDemo.begin(Minecraft.getInstance(), false); return 1; })
                .then(literal("replan")
                    .executes(ctx -> { NetherDemo.begin(Minecraft.getInstance(), true); return 1; }))
                .then(literal("replay")
                    .executes(ctx -> { NetherDemo.replayCurrent(Minecraft.getInstance()); return 1; }))
                .then(literal("clear")
                    .executes(ctx -> { NetherDemo.clearCache(Minecraft.getInstance()); return 1; })))
            .then(literal("config")
                .executes(ctx -> {
                    ctx.getSource().sendFeedback(Component.literal("[nether-pathfinder] " + settings.describe()));
                    return 1;
                })
                .then(argument("key", StringArgumentType.word())
                    .then(argument("value", StringArgumentType.word())
                        .executes(ctx -> setConfig(ctx.getSource(),
                            StringArgumentType.getString(ctx, "key"),
                            StringArgumentType.getString(ctx, "value")))))));
    }

    private static int runGoto(FabricClientCommandSource source, BlockPos targetPos) {
        ClientLevel level = source.getLevel();
        LocalPlayer player = source.getPlayer();
        BlockPos startPos = player.blockPosition();
        InventoryResources.Snapshot resources = InventoryResources.capture(player);

        PathfinderSettings.Snapshot snapshot = settings.snapshot();
        int dx = Math.abs(targetPos.getX() - startPos.getX());
        int dz = Math.abs(targetPos.getZ() - startPos.getZ());
        int horizontalRadius = Math.min(MAX_HORIZONTAL_RADIUS, Math.max(snapshot.radius(),
            Math.max(dx, dz) / 2 + CAPTURE_MARGIN));
        BlockPos captureCenter = startPos.offset(
            (targetPos.getX() - startPos.getX()) / 2,
            0,
            (targetPos.getZ() - startPos.getZ()) / 2);

        source.sendFeedback(Component.literal(
            "[nether-pathfinder] searching for a route (radius " + horizontalRadius
                + ", " + resources.pickaxeTier().name().toLowerCase(java.util.Locale.ROOT)
                + " pickaxe, " + resources.bridgeBlocks() + " bridge blocks)..."));
        long generation = SEARCH_GENERATION.incrementAndGet();
        cancelActiveSearch();
        RouteState.INSTANCE.set(new RouteState.Route(new double[0][], new dev.netherpathfinder.engine.Action[0],
            RouteState.Status.SEARCHING, null));

        Future<?> task = SEARCH_EXECUTOR.submit(() -> {
            try {
                runSearch(level, startPos, targetPos, captureCenter, horizontalRadius, source, generation, snapshot, resources);
            } catch (CancellationException ignored) {
                // /pathfind stop or the configured deadline superseded this search.
            } catch (Exception e) {
                NetherPathfinderMod.LOGGER.error("Pathfind search failed", e);
                reportAndSet(source, generation, RouteState.Status.ERROR, "search failed: " + e.getMessage());
            }
        });
        ACTIVE_SEARCH.set(task);
        return 1;
    }

    /**
     * Structure location needs the world generator, which only exists locally
     * in integrated single-player. On a multiplayer connection the client has
     * no authoritative data for ungenerated structure placement, so reporting
     * that boundary is safer than drawing a guessed route to a false target.
     */
    private static int runBastion(FabricClientCommandSource source) {
        if (source.getLevel().dimension() != Level.NETHER) {
            source.sendFeedback(Component.literal("[nether-pathfinder] enter the Nether before locating a bastion."));
            return 0;
        }
        var server = Minecraft.getInstance().getSingleplayerServer();
        if (server == null) {
            source.sendFeedback(Component.literal(
                "[nether-pathfinder] bastion locate needs an integrated single-player world; use /pathfind goto on servers."));
            return 0;
        }

        BlockPos playerPos = source.getPlayer().blockPosition();
        source.sendFeedback(Component.literal("[nether-pathfinder] locating the nearest bastion..."));
        server.execute(() -> {
            BlockPos bastion = locateNearestBastion(server.getLevel(Level.NETHER), playerPos);
            Minecraft.getInstance().execute(() -> {
                if (bastion == null) {
                    source.sendFeedback(Component.literal("[nether-pathfinder] no bastion found in the search area."));
                } else {
                    runGoto(source, bastion);
                }
            });
        });
        return 1;
    }

    /** Probes overlapping bastion cells, then selects the genuinely nearest result by X/Z distance. */
    static BlockPos locateNearestBastion(ServerLevel level, BlockPos playerPos) {
        var bastionKey = ResourceKey.create(Registries.STRUCTURE, Identifier.withDefaultNamespace("bastion_remnant"));
        var bastion = level.registryAccess().lookupOrThrow(Registries.STRUCTURE).get(bastionKey).orElse(null);
        if (bastion == null) return null;
        HolderSet<Structure> target = HolderSet.direct(bastion);
        BlockPos nearest = null;
        long nearestDistance = Long.MAX_VALUE;
        java.util.HashSet<Long> seen = new java.util.HashSet<>();
        for (int gridX = -1; gridX <= 1; gridX++) {
            for (int gridZ = -1; gridZ <= 1; gridZ++) {
                BlockPos origin = playerPos.offset(gridX * BASTION_PROBE_SPACING, 0, gridZ * BASTION_PROBE_SPACING);
                Pair<BlockPos, net.minecraft.core.Holder<Structure>> result = level.getChunkSource().getGenerator()
                    .findNearestMapStructure(level, target, origin, BASTION_PROBE_RADIUS_CHUNKS, false);
                if (result == null || !seen.add(result.getFirst().asLong())) continue;
                long dx = (long) result.getFirst().getX() - playerPos.getX();
                long dz = (long) result.getFirst().getZ() - playerPos.getZ();
                long distance = dx * dx + dz * dz;
                if (distance < nearestDistance) {
                    nearest = result.getFirst();
                    nearestDistance = distance;
                }
            }
        }
        if (nearest != null) return atPlayerHeight(level, nearest, playerPos);
        Pair<BlockPos, net.minecraft.core.Holder<Structure>> fallback = level.getChunkSource().getGenerator()
            .findNearestMapStructure(level, target, playerPos, BASTION_FALLBACK_RADIUS_CHUNKS, false);
        return fallback == null ? null : atPlayerHeight(level, fallback.getFirst(), playerPos);
    }

    /** Structure locating is fundamentally X/Z-based; its returned Y is not a navigable entrance height. */
    private static BlockPos atPlayerHeight(ServerLevel level, BlockPos structurePos, BlockPos playerPos) {
        int y = Math.clamp(playerPos.getY(), level.getMinY() + 1, level.getMaxY() - 2);
        return new BlockPos(structurePos.getX(), y, structurePos.getZ());
    }

    private static void runSearch(ClientLevel level, BlockPos startPos, BlockPos targetPos,
                                   BlockPos captureCenter, int horizontalRadius, FabricClientCommandSource source,
                                   long generation, PathfinderSettings.Snapshot settings,
                                   InventoryResources.Snapshot resources) {
        if (!isCurrent(generation)) throw new CancellationException();
        TerrainCapture.Result capture = TerrainCapture.capture(level, captureCenter, horizontalRadius);
        if (!isCurrent(generation)) throw new CancellationException();

        int startX = capture.toLocalX(startPos.getX());
        int startY = capture.toLocalY(startPos.getY());
        int startZ = capture.toLocalZ(startPos.getZ());
        int goalX = capture.toLocalX(targetPos.getX());
        int goalY = capture.toLocalY(targetPos.getY());
        int goalZ = capture.toLocalZ(targetPos.getZ());

        if (!inBounds(capture, startX, startY, startZ) || !inBounds(capture, goalX, goalY, goalZ)) {
            reportAndSet(source, generation, RouteState.Status.ERROR,
                "target is outside the captured terrain range, try a closer target");
            return;
        }

        StateCodec.State start = new StateCodec.State(startX, startY, startZ, resources.bridgeBlocks(), false);
        GoalPoints goal = GoalPoints.point(goalX, goalY, goalZ);
        var tools = dev.netherpathfinder.engine.MineCostModel.forTier(resources.pickaxeTier());

        applySettings(settings);
        WeightedAStar search = new WeightedAStar();
        Thread searchThread = Thread.currentThread();
        Future<?> timeout = SEARCH_TIMEOUTS.schedule(searchThread::interrupt,
            Math.round(settings.timeoutSeconds() * 1000), TimeUnit.MILLISECONDS);
        SearchResult result;
        try {
            result = search.search(capture.world, start, goal,
                resources.bridgeBlocks(), settings.weight(), tools, settings.maxExpansions());
        } finally {
            timeout.cancel(false);
            if (Thread.interrupted()) {
                if (isCurrent(generation)) {
                    reportAndSet(source, generation, RouteState.Status.NOT_FOUND,
                        "search timed out after " + settings.timeoutSeconds() + " seconds");
                }
                return;
            }
        }

        if (!result.found()) {
            reportAndSet(source, generation, RouteState.Status.NOT_FOUND,
                "no route found (" + result.expansions() + " expansions)");
            return;
        }

        StateCodec.State[] path = result.path();
        double[][] points = new double[path.length][];
        for (int i = 0; i < path.length; i++) {
            points[i] = new double[] {
                capture.toWorldX(path[i].x()) + 0.5,
                capture.toWorldY(path[i].y()),
                capture.toWorldZ(path[i].z()) + 0.5
            };
        }

        String message = String.format(java.util.Locale.ROOT,
            "route found: %d waypoints, cost %.1f, %d expansions",
            points.length, result.totalCost(), result.expansions());
        if (isCurrent(generation)) {
            RouteState.INSTANCE.set(new RouteState.Route(points, result.actions(), RouteState.Status.FOUND, message));
            sendFeedback(source, message);
        }
    }

    private static boolean inBounds(TerrainCapture.Result capture, int x, int y, int z) {
        return x >= 0 && x < capture.world.sizeX
            && y >= 0 && y < capture.world.sizeY
            && z >= 0 && z < capture.world.sizeZ;
    }

    private static void reportAndSet(FabricClientCommandSource source, long generation,
                                     RouteState.Status status, String message) {
        if (!isCurrent(generation)) return;
        RouteState.INSTANCE.set(new RouteState.Route(new double[0][], new dev.netherpathfinder.engine.Action[0], status, message));
        sendFeedback(source, message);
    }

    private static int setXray(FabricClientCommandSource source, Integer requested) {
        int mode = requested != null ? requested : ((int) settings.get("routeThroughWalls") + 1) % 3;
        try {
            settings.set("routeThroughWalls", Integer.toString(mode));
            settings.set("iconThroughWalls", mode == 0 ? "0" : "1");
        } catch (IOException e) {
            source.sendFeedback(Component.literal("[nether-pathfinder] " + e.getMessage()));
            return 0;
        }
        String what = switch (mode) {
            case 0 -> "OFF (normal depth-tested route)";
            case 1 -> "FALLING (pieces show through terrain while they drop in, then settle normally)";
            default -> "FULL (the whole route always shows through terrain)";
        };
        source.sendFeedback(Component.literal("[nether-pathfinder] see-through mode: " + what));
        return 1;
    }

    private static int setConfig(FabricClientCommandSource source, String key, String value) {
        try {
            settings.set(key, value);
            source.sendFeedback(Component.literal("[nether-pathfinder] " + key + "=" + settings.get(key)));
            return 1;
        } catch (IOException | IllegalArgumentException e) {
            source.sendFeedback(Component.literal("[nether-pathfinder] " + e.getMessage()));
            return 0;
        }
    }

    static void applySettings(PathfinderSettings.Snapshot settings) {
        EdgeRules.SPRINT_SPEED = settings.sprintSpeed();
        EdgeRules.SPEED_BRIDGE_SPEED = settings.bridgeSpeed();
        EdgeRules.PLACE_TIME = settings.placeTime();
    }

    private static boolean isCurrent(long generation) {
        return SEARCH_GENERATION.get() == generation && !Thread.currentThread().isInterrupted();
    }

    private static void stopActiveSearch() {
        SEARCH_GENERATION.incrementAndGet();
        cancelActiveSearch();
    }

    private static void cancelActiveSearch() {
        Future<?> previous = ACTIVE_SEARCH.getAndSet(null);
        if (previous != null) previous.cancel(true);
    }

    // sendFeedback touches chat/GUI state, so it must run on the client
    // thread -- runSearch (and its error paths) run on SEARCH_EXECUTOR.
    private static void sendFeedback(FabricClientCommandSource source, String message) {
        Minecraft.getInstance().execute(() ->
            source.sendFeedback(Component.literal("[nether-pathfinder] " + message)));
    }
}
