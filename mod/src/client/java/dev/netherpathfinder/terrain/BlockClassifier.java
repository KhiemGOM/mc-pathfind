package dev.netherpathfinder.terrain;

import dev.netherpathfinder.engine.BlockType;
import net.minecraft.core.registries.BuiltInRegistries;
import net.minecraft.world.level.block.Block;
import net.minecraft.world.level.block.state.BlockState;

import java.util.Locale;

/**
 * Maps a real BlockState to the engine's BlockType codes, by substring-
 * matching the block's registry path (e.g. "netherrack", "soul_sand",
 * "warped_wart_block") -- functionally the same classification vanilla
 * translation keys would give (registry path and translation key agree for
 * every block below), but reads directly off the real Block class instead
 * of a display-string lookup.
 */
public final class BlockClassifier {
    private BlockClassifier() {}

    public static int classify(BlockState state) {
        if (state == null || state.isAir()) {
            return BlockType.AIR;
        }
        Block block = state.getBlock();
        String path = BuiltInRegistries.BLOCK.getKey(block).getPath().toLowerCase(Locale.ROOT);

        if (path.contains("magma_block")) {
            return BlockType.MAGMA;
        }
        if (path.contains("soul_sand")) {
            return BlockType.SOUL_SAND;
        }
        if (path.contains("fire")) {
            return BlockType.FIRE;
        }
        if (path.contains("netherrack") || path.contains("nylium")) {
            return BlockType.NETHERRACK;
        }
        // Nether Wart Block / Warped Wart Block / Shroomlight -- solid, full
        // blocks -- "wart_block" MUST be checked before the generic "wart"
        // check below (aimed at the non-solid nether_wart crop), which would
        // otherwise also swallow these two solid blocks and misclassify them
        // as passable. Hardness 1.0, best tool is a HOE (not pickaxe) in
        // vanilla -- SOFT_ORGANIC reflects that (no pickaxe speed bonus),
        // unlike STONE which assumes pickaxe-class.
        if (path.contains("wart_block") || path.contains("shroomlight")) {
            return BlockType.SOFT_ORGANIC;
        }
        if (path.contains("portal") || path.contains("vine") || path.contains("roots")
                || path.contains("sprouts") || path.contains("fungus")
                || path.contains("mushroom") || path.contains("wart")
                || path.contains("torch") || path.contains("button") || path.contains("pressure_plate")) {
            return BlockType.AIR;
        }
        if (path.contains("lava")) {
            return BlockType.LAVA;
        }
        if (path.contains("bedrock") || path.contains("barrier")) {
            return BlockType.BEDROCK;
        }
        // Respawn Anchor shares Obsidian's exact hardness (50) and harvest
        // level (diamond/netherite pickaxe only), so it's classified as
        // OBSIDIAN directly rather than getting its own code.
        if (path.contains("obsidian") || path.contains("ancient_debris") || path.contains("respawn_anchor")) {
            return BlockType.OBSIDIAN;
        }
        // Crimson/Warped Stem & Hyphae (+stripped variants) -- hardness 2.0,
        // best tool is an AXE (not pickaxe) in vanilla, so no pickaxe speed
        // bonus applies.
        if (path.contains("stem") || path.contains("hyphae")) {
            return BlockType.WOOD_TIER;
        }
        if (path.contains("chain")) {
            return BlockType.CHAIN;
        }
        // Must be checked before the generic "stone" check below --
        // "lodestone" contains "stone" and would otherwise land there,
        // losing its distinct (higher) hardness.
        if (path.contains("lodestone")) {
            return BlockType.LODESTONE;
        }
        if (path.contains("bone_block")) {
            return BlockType.BONE_BLOCK;
        }
        if (path.contains("stone") || path.contains("basalt") || path.contains("ore") || path.contains("brick")) {
            return BlockType.STONE;
        }
        if (state.isAir()) {
            return BlockType.AIR;
        }
        return BlockType.DIRT;
    }
}
