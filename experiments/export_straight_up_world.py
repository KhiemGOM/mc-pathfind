"""One-off: a pure vertical shaft, goal directly overhead, no lateral
displacement at all -- isolates BRIDGE_UP with the goal-relative prune
staying maximally favorable (horizGoal=0) throughout the whole climb, to
confirm the action + reverse-probing work mechanically before reasoning
about the harder "goal on the far side of a ledge" case.
"""
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from world import AIR, DIRT
import numpy as np

MAGIC = b"MCPW"
VERSION = 1

size_x, size_y, size_z = 15, 20, 15
world = np.full((size_x, size_y, size_z), AIR, dtype=np.int8)
ground_h = 5
world[:, 0:ground_h, :] = DIRT  # flat floor, nothing above it -- pure open air shaft

start = (7, ground_h, 7)
goal = (7, ground_h + 4, 7)  # straight up, 4 blocks, same (x,z) the whole way

header = struct.pack(
    "<4s14i", MAGIC, VERSION,
    size_x, size_y, size_z,
    0, 0, 0,
    start[0], start[1], start[2],
    goal[0], goal[1], goal[2],
    32,
)
out = Path(sys.argv[1] if len(sys.argv) > 1 else "java/benchmark/data/lab/straight_up_world.wbin")
out.parent.mkdir(parents=True, exist_ok=True)
out.write_bytes(header + world.tobytes())
print(f"wrote {out}: shape={world.shape} start={start} goal={goal}")
