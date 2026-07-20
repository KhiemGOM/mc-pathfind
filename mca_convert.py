"""
Converts a real Minecraft Anvil region file (.mca) into the pathfinder's
world[x][y][z] int8 array format (AIR/DIRT/STONE/OBSIDIAN/BEDROCK).

Mapping rationale:
  - AIR, cave_air, void_air        -> AIR (0)          walkable space
  - bedrock                        -> BEDROCK (4)       unbreakable (matches real MC)
  - obsidian, crying_obsidian      -> OBSIDIAN (3)      very slow to mine (matches MINE_TIME)
  - ancient_debris                 -> OBSIDIAN (3)      similarly slow-mining, blast-resistant
  - lava                           -> LAVA (5)          impassable fluid: cannot stand in it,
                                                          cannot be mined, but CAN be bridged
                                                          over (place a block on top) same as
                                                          any void gap; falling into it is
                                                          lethal (handled by the pathfinder,
                                                          not this converter)
  - netherrack, soul_sand, soul_soil, magma_block,
    gravel, basalt, glowstone     -> DIRT (1)           per your instruction: these "don't matter"
                                                          for now, treat as fast/normal terrain
  - blackstone, polished_blackstone_bricks,
    cracked_polished_blackstone_bricks,
    nether_quartz_ore, nether_gold_ore -> STONE (2)     stone-tier mining time, structurally
                                                          "wall-like" (bastion remnant material)
  - fire                           -> AIR (0)           not a solid obstruction, walkable-through
                                                          is wrong physically (damages player) but
                                                          out of scope for this pathfinder's cost
                                                          model right now; treated as passable air
  - anything else unrecognized      -> STONE (2)         safe default: assume it's a mineable
                                                          obstruction rather than silently AIR
                                                          (which could let the planner walk through
                                                          something solid) or BEDROCK (which could
                                                          make a real gap look unfairly impassable)
"""

import numpy as np
import anvil
from pathlib import Path

AIR, DIRT, STONE, OBSIDIAN, BEDROCK, LAVA = 0, 1, 2, 3, 4, 5

BLOCK_MAP = {
    "air": AIR, "cave_air": AIR, "void_air": AIR, "fire": AIR,
    "bedrock": BEDROCK,
    "lava": LAVA,
    "obsidian": OBSIDIAN, "crying_obsidian": OBSIDIAN, "ancient_debris": OBSIDIAN,
    "netherrack": DIRT, "soul_sand": DIRT, "soul_soil": DIRT,
    "magma_block": DIRT, "gravel": DIRT, "basalt": DIRT, "glowstone": DIRT,
    "polished_basalt": DIRT,
    "blackstone": STONE, "polished_blackstone_bricks": STONE,
    "cracked_polished_blackstone_bricks": STONE,
    "nether_quartz_ore": STONE, "nether_gold_ore": STONE,
    "blackstone_slab": STONE, "blackstone_stairs": STONE, "blackstone_wall": STONE,
    "chiseled_polished_blackstone": STONE, "gilded_blackstone": STONE,
    "polished_blackstone": STONE, "polished_blackstone_brick_slab": STONE,
    "polished_blackstone_brick_stairs": STONE, "polished_blackstone_slab": STONE,
    "gold_block": STONE,
    # small decorations/furniture: treat as passable-ish obstruction (STONE default
    # already covers these via DEFAULT_UNKNOWN, but list explicitly for clarity)
    "chain": STONE, "lantern": STONE, "chest": STONE, "spawner": STONE,
    "brown_mushroom": DIRT, "red_mushroom": DIRT,  # small non-blocking plants -> treat as trivial/DIRT-tier
    "crimson_nylium": DIRT, "warped_nylium": DIRT,  # walkable ground, same tier as netherrack
    "bone_block": STONE,
    "crimson_roots": AIR, "warped_roots": AIR,  # decorative ground cover, non-blocking
    "soul_fire": AIR,
    "crimson_stem": STONE, "warped_stem": STONE,  # fungus "wood", structurally wall-like at trunk scale
    "nether_wart_block": DIRT, "warped_wart_block": DIRT, "shroomlight": DIRT,
    "twisting_vines": AIR, "twisting_vines_plant": AIR,
    "weeping_vines": AIR, "weeping_vines_plant": AIR,
}

DEFAULT_UNKNOWN = STONE


# anvil-parser's own section decoder (Chunk.stream_blocks) constructs a
# fresh Block object -- NBT dict lookups, a str.split, an isinstance check --
# for every single one of a section's 4096 voxels, even though a section
# typically only has a handful of *distinct* block types (its palette).
# Profiling confirmed this Block-object construction is ~90% of total
# convert_region time. _fast_section_codes replaces it with: look up each
# palette ENTRY's code once (typically single digits, not 4096), then
# vectorize the packed-bitfield -> palette-index unpacking with numpy and
# use a single fancy-index gather (palette code table[indices]) instead of
# 4096 per-voxel Python-level Block constructions.
#
# Only handles the "non-stretched" packed format (MC 1.16+, DataVersion >=
# _VERSION_20w17a) where each 64-bit long holds a whole number of
# fixed-width palette indices and no index straddles a long boundary --
# this matches every region file this project has been given (DataVersion
# 2567). Anything else (older worlds, or a section actually missing
# BlockStates/Palette) raises so the caller can fall back to the slow but
# always-correct stream_blocks() path; this function must never silently
# produce wrong voxel data.
_VERSION_20w17a = 2529


def _fast_section_codes(section, version):
    if version < _VERSION_20w17a:
        raise NotImplementedError("pre-1.16 stretched block-state packing not supported by the fast path")
    if section is None or "BlockStates" not in section or "Palette" not in section:
        raise NotImplementedError("section missing BlockStates/Palette")

    palette = section["Palette"]
    n = len(palette)
    bits = max((n - 1).bit_length(), 4)
    per_long = 64 // bits
    bits_mask = np.uint64((1 << bits) - 1)

    states = section["BlockStates"].value
    # The `nbt` package's TAG_Long_Array values aren't consistently signed
    # two's-complement -- some come back already in the unsigned range
    # [2**63, 2**64), which overflows a direct int64 view. Masking to 64
    # bits in pure Python (arbitrary precision, no overflow) normalizes
    # both representations to the same canonical unsigned value before
    # handing off to numpy. Confirmed necessary by direct testing: several
    # longs in real section data raised OverflowError on the naive
    # dtype=np.int64 -> view(np.uint64) conversion.
    arr = np.array([s & 0xFFFFFFFFFFFFFFFF for s in states], dtype=np.uint64)
    expected_longs = -(-4096 // per_long)  # ceil
    if len(arr) < expected_longs:
        raise NotImplementedError("BlockStates shorter than expected for non-stretched packing")

    idx = np.arange(4096)
    which_long = idx // per_long
    offset = (idx % per_long) * bits
    raw = arr[which_long]
    palette_ids = ((raw >> offset.astype(np.uint64)) & bits_mask).astype(np.int64)
    if palette_ids.max(initial=0) >= n:
        raise NotImplementedError("decoded palette index out of range -- packing assumption doesn't match this section")

    code_table = np.empty(n, dtype=np.int8)
    unknown_ids = []
    for i, entry in enumerate(palette):
        name = entry["Name"].value
        block_id = name.split(":")[-1]
        code = BLOCK_MAP.get(block_id)
        if code is None:
            code = DEFAULT_UNKNOWN
            unknown_ids.append(block_id)

        code_table[i] = code

    codes = code_table[palette_ids]  # (4096,) in (local_y, z, x) flat order
    return codes.reshape(16, 16, 16), unknown_ids


def convert_region(mca_path, chunk_x_range, chunk_z_range, y_range=(0, 128), verbose=True):
    """
    chunk_x_range, chunk_z_range: (start, end) exclusive, in CHUNK coords (0-31 within a region).
    y_range: (min_y, max_y) exclusive upper bound, in world y coordinates.
    Returns (world_array, origin) where origin = (world_x0, world_y0, world_z0) so callers
    can map array indices back to real MC world coordinates.

    Uses Chunk.stream_chunk() (bulk section decode) rather than per-voxel get_block(),
    which is roughly 5-6x faster since it avoids repeated palette/section lookups.
    The installed anvil-parser streams its 16 stored sections (y = 0..255) in
    flat index order `local_y*256 + z*16 + x`.  These region files use that
    legacy-height layout, so array y maps directly to stream y; applying a
    modern-world -64 offset would read the wrong vertical band (including the
    Nether roof instead of the terrain below it).
    """
    region = anvil.Region.from_file(mca_path)

    cx0, cx1 = chunk_x_range
    cz0, cz1 = chunk_z_range
    y0, y1 = y_range
    WORLD_Y_MIN = 0

    size_x = (cx1 - cx0) * 16
    size_y = y1 - y0
    size_z = (cz1 - cz0) * 16

    world = np.zeros((size_x, size_y, size_z), dtype=np.int8)
    unknown_seen = set()
    unreadable_chunks = 0

    # anvil-parser's stream_chunk() unconditionally decodes all 16 fixed
    # 16-tall sections (each section building 4096 Block objects via NBT
    # palette lookups + bit-unpacking), even though a sub-height y_range --
    # e.g. the Nether's 0..128 -- only ever needs a subset. Precompute which
    # sections actually overlap [y0, y1) and the local slice needed within
    # each, so per-chunk work scales with the requested height, not always
    # the full 256-block legacy section stack.
    section_plan = []  # (section_index, slice_start_in_section, slice_end_in_section, world_y_start, world_y_end)
    for s in range(16):
        s_lo, s_hi = s * 16, s * 16 + 16
        lo, hi = max(s_lo, y0), min(s_hi, y1)
        if lo < hi:
            section_plan.append((s, lo - s_lo, hi - s_lo, lo - y0, hi - y0))

    for cx in range(cx0, cx1):
        for cz in range(cz0, cz1):
            try:
                chunk = anvil.Chunk.from_region(region, cx, cz)
            except Exception:
                unreadable_chunks += 1
                continue

            wx0 = (cx - cx0) * 16
            wz0 = (cz - cz0) * 16

            try:
                for section, s_lo, s_hi, wy_lo, wy_hi in section_plan:
                    try:
                        # Vectorized path: unpack the section's raw palette
                        # indices with numpy and gather codes through a
                        # tiny per-palette-entry lookup table, instead of
                        # anvil-parser constructing a Block object per voxel
                        # (profiled at ~90% of total conversion time).
                        section_nbt = chunk.get_section(section)
                        codes, unknown_ids = _fast_section_codes(section_nbt, chunk.version)
                        unknown_seen.update(unknown_ids)
                    except NotImplementedError:
                        # Format this function doesn't handle (older world,
                        # or a section actually missing BlockStates/Palette,
                        # e.g. an all-air section) -- fall back to the
                        # slower but always-correct per-block decode rather
                        # than guessing.
                        ids = [b.id for b in chunk.stream_blocks(section=section)]
                        codes = np.empty(len(ids), dtype=np.int8)
                        for i, block_id in enumerate(ids):
                            code = BLOCK_MAP.get(block_id)
                            if code is None:
                                code = DEFAULT_UNKNOWN
                                unknown_seen.add(block_id)
                            codes[i] = code
                        codes = codes.reshape(16, 16, 16)

                    codes = codes[s_lo:s_hi]  # (used_y, z, x)
                    world[wx0:wx0 + 16, wy_lo:wy_hi, wz0:wz0 + 16] = np.transpose(codes, (2, 0, 1))
            except Exception:
                # Same anvil-parser decode-bug handling as chunk lookup above
                # (see comment there): treat an unreadable section as AIR
                # rather than aborting the whole region's conversion.
                unreadable_chunks += 1
                continue

    if verbose and unknown_seen:
        print(f"[convert_region] {len(unknown_seen)} unmapped block type(s) defaulted to STONE: {sorted(unknown_seen)}")
    if verbose and unreadable_chunks:
        print(f"[convert_region] {unreadable_chunks} chunk(s) failed to decode and were left as AIR")

    # Region filenames carry their signed region coordinates.  Anvil's chunk
    # coordinates above are local (0..31), so include this offset when handing
    # callers coordinates they can relate back to their Minecraft world.
    stem_parts = Path(mca_path).stem.split(".")
    try:
        region_x, region_z = int(stem_parts[1]), int(stem_parts[2])
    except (IndexError, ValueError):
        region_x = region_z = 0
    origin = (region_x * 512 + cx0 * 16, y0, region_z * 512 + cz0 * 16)
    return world, origin


if __name__ == "__main__":
    import sys
    world, origin = convert_region(
        "/mnt/user-data/uploads/r_0_0.mca",
        chunk_x_range=(0, 8), chunk_z_range=(0, 8), y_range=(0, 128)
    )
    print("world shape:", world.shape, "origin (world coords):", origin)
    unique, counts = np.unique(world, return_counts=True)
    for u, c in zip(unique, counts):
        print(f"  code {u}: {c} voxels")
