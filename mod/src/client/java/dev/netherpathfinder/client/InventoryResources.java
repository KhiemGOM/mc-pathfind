package dev.netherpathfinder.client;

import dev.netherpathfinder.engine.BlockType;
import dev.netherpathfinder.engine.MineCostModel;
import dev.netherpathfinder.engine.StateCodec;
import dev.netherpathfinder.terrain.BlockClassifier;
import net.minecraft.client.player.LocalPlayer;
import net.minecraft.core.BlockPos;
import net.minecraft.world.item.BlockItem;
import net.minecraft.world.item.Item;
import net.minecraft.world.item.ItemStack;
import net.minecraft.world.item.Items;
import net.minecraft.world.level.block.Block;
import net.minecraft.world.level.block.state.BlockState;

/**
 * A client-thread snapshot of the resources that affect a route. The search
 * receives only this immutable value, so it never reads player inventory from
 * its worker thread.
 */
final class InventoryResources {
    record Snapshot(MineCostModel.PickaxeTier pickaxeTier, int bridgeBlocks) {}

    private InventoryResources() {}

    static Snapshot capture(LocalPlayer player) {
        MineCostModel.PickaxeTier bestPickaxe = MineCostModel.PickaxeTier.NONE;
        int bridgeBlocks = 0;

        for (int slot = 0; slot < player.getInventory().getContainerSize(); slot++) {
            ItemStack stack = player.getInventory().getItem(slot);
            if (stack.isEmpty()) continue;

            MineCostModel.PickaxeTier tier = pickaxeTier(stack.getItem());
            if (tier.preferredOver(bestPickaxe, false)) {
                bestPickaxe = tier;
            }
            if (isUsableBridgeBlock(player, stack)) {
                bridgeBlocks = Math.min(StateCodec.MAX_BLOCKS, bridgeBlocks + stack.getCount());
            }
        }
        return new Snapshot(bestPickaxe, bridgeBlocks);
    }

    private static MineCostModel.PickaxeTier pickaxeTier(Item item) {
        if (item == Items.NETHERITE_PICKAXE) return MineCostModel.PickaxeTier.NETHERITE;
        if (item == Items.DIAMOND_PICKAXE) return MineCostModel.PickaxeTier.DIAMOND;
        if (item == Items.IRON_PICKAXE) return MineCostModel.PickaxeTier.IRON;
        if (item == Items.STONE_PICKAXE) return MineCostModel.PickaxeTier.STONE;
        if (item == Items.GOLDEN_PICKAXE) return MineCostModel.PickaxeTier.GOLDEN;
        if (item == Items.WOODEN_PICKAXE) return MineCostModel.PickaxeTier.WOODEN;
        return MineCostModel.PickaxeTier.NONE;
    }

    /** Counts only full, non-hazardous blocks that the engine can stand on. */
    private static boolean isUsableBridgeBlock(LocalPlayer player, ItemStack stack) {
        if (!(stack.getItem() instanceof BlockItem blockItem)) return false;
        BlockState state = blockItem.getBlock().defaultBlockState();
        if (!Block.isShapeFullBlock(state.getCollisionShape(player.level(), BlockPos.ZERO))) return false;
        int type = BlockClassifier.classify(state);
        return type != BlockType.AIR && type != BlockType.LAVA && type != BlockType.MAGMA
            && type != BlockType.FIRE && type != BlockType.BEDROCK;
    }
}
