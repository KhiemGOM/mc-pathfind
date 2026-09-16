package dev.netherpathfinder;

import net.fabricmc.api.ModInitializer;

import net.minecraft.resources.Identifier;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/** Common (server+client) entrypoint. Command registration and the actual pathfinding
 * logic live in the client-only entrypoint, since this mod only ever runs client-side. */
public class NetherPathfinderMod implements ModInitializer {
    public static final String MOD_ID = "netherpathfinder";

    public static final Logger LOGGER = LoggerFactory.getLogger(MOD_ID);

    @Override
    public void onInitialize() {
        LOGGER.info("Nether Pathfinder initializing");
    }

    public static Identifier id(String path) {
        return Identifier.fromNamespaceAndPath(MOD_ID, path);
    }
}
