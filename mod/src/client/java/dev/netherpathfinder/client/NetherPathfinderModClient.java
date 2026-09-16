package dev.netherpathfinder.client;

import net.fabricmc.api.ClientModInitializer;

/** Client entrypoint: registers the /pathfind command tree and the world-render hook that
 * draws the currently active route. */
public class NetherPathfinderModClient implements ClientModInitializer {
    @Override
    public void onInitializeClient() {
        PathfindCommands.register();
        RouteRenderer.register();
    }
}
