"""One-off: build a standalone HTML viz for floating_islands_world using the
Java-computed path (dumped by DumpPathJson.java as JSON), since Python's
pathfind.py has no BRIDGE_UP -- can't solve this world itself, only render it.
"""
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_REPO_ROOT / "viz"))
from world import generate_floating_islands_world
from build_visualization import export_scenario, build_html

path_json = json.loads(Path(__file__).with_name("floating_islands_path.json").read_text())
world, islands = generate_floating_islands_world()
start_x0, start_x1, start_y = islands[0]
goal_x0, goal_x1, goal_y = islands[-1]
mid_z = world.shape[2] // 2
start = ((start_x0 + start_x1) // 2, start_y, mid_z)
goal = ((goal_x0 + goal_x1) // 2, goal_y, mid_z)

scenario = export_scenario(
    world, path_json["path"], path_json["actions"], start, goal,
    path_json["cost"], path_json["expansions"],
)

out_path = _REPO_ROOT / "viz" / "viz_floating_islands.html"
build_html({"floating_islands": scenario}, output_path=str(out_path))
print(f"wrote {out_path}")
