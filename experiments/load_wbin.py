"""Minimal .wbin reader -- mirrors viz/export_world_bin.py's header format
byte-for-byte, so real-region worlds already exported for the Java
benchmarks can be loaded back into numpy without touching mca_convert.py
or the original .mca files again."""
import struct
import numpy as np

HEADER_FMT = "<4s14i"
HEADER_SIZE = struct.calcsize(HEADER_FMT)


def load_wbin(path):
    data = open(path, "rb").read()
    magic, version, sx, sy, sz, ox, oy, oz, startx, starty, startz, goalx, goaly, goalz, blocks = \
        struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
    assert magic == b"MCPW", f"bad magic: {magic}"
    world = np.frombuffer(data[HEADER_SIZE:], dtype=np.int8).reshape(sx, sy, sz)
    return world, (startx, starty, startz), (goalx, goaly, goalz), blocks
