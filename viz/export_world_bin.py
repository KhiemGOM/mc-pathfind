"""
One-off dev tool: dumps an already-converted region world array to the flat
binary format the Java port's WorldBinFormat loader reads (see
java/core/src/main/java/dev/mcpathfind/core/io/WorldBinFormat.java for the
exact layout this must match byte-for-byte).

This is NOT production code and does not belong to the Java port's scope --
in the real integration, block data comes from the live game client, not
from a region file. This exists purely so the Java search can be benchmarked
against the same real terrain already used to measure the Python baseline.

Usage (from the repo root; region path is relative to wherever you run this):
    python3 viz/export_world_bin.py regions/k1/r.0.0.mca out/r_0_0.wbin
    python3 viz/export_world_bin.py regions/k1/r.0.0.mca out/r_0_0.wbin \
        --start 30,43,90 --goal 159,39,7 --blocks 32
"""

import argparse
import struct
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from build_visualization import adaptive_chunk_range, _find_region_route
from mca_convert import convert_region

MAGIC = b"MCPW"
VERSION = 1
NO_COORD = -1


def export_world_bin(mca_path, output_path, y_range=(0, 128), start=None, goal=None, blocks_available=32):
    chunk_x_range, chunk_z_range = adaptive_chunk_range(mca_path)
    world, origin = convert_region(mca_path, chunk_x_range=chunk_x_range, chunk_z_range=chunk_z_range, y_range=y_range)

    if start is None or goal is None:
        auto_start, auto_goal = _find_region_route(world)
        start = start or auto_start
        goal = goal or auto_goal
        if start is None or goal is None:
            raise RuntimeError(f"{mca_path}: no walkable start/goal found and none given explicitly")

    size_x, size_y, size_z = world.shape
    header = struct.pack(
        "<4s14i",  # magic + 14 int32 fields; verify against WorldBinFormat.HEADER_SIZE=60 if this ever changes
        MAGIC, VERSION,
        size_x, size_y, size_z,
        origin[0], origin[1], origin[2],
        start[0], start[1], start[2],
        goal[0], goal[1], goal[2],
        blocks_available,
    )
    payload = world.tobytes()  # numpy C-order matches WorldBinFormat's (x*sizeY+y)*sizeZ+z exactly

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(header + payload)
    print(f"wrote {output_path}: shape={world.shape} start={start} goal={goal} "
          f"blocks={blocks_available} size={(len(header) + len(payload)) / 1024:.1f} KB")


def _parse_coord(s):
    if s is None:
        return None
    x, y, z = (int(v) for v in s.split(","))
    return (x, y, z)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mca_path")
    parser.add_argument("output_path")
    parser.add_argument("--start", default=None, help="x,y,z -- default: auto-selected via _find_region_route")
    parser.add_argument("--goal", default=None, help="x,y,z -- default: auto-selected via _find_region_route")
    parser.add_argument("--blocks", type=int, default=32)
    args = parser.parse_args()

    export_world_bin(
        args.mca_path, args.output_path,
        start=_parse_coord(args.start), goal=_parse_coord(args.goal),
        blocks_available=args.blocks,
    )
