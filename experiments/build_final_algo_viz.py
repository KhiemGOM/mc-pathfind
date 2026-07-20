"""Builds one combined, tabbed visualization covering the full final
algorithm state (BRIDGE_UP, ratio=0.0 default, y-tiebreak, dense-id
bidirectional search) across every lab scenario plus a real-terrain region --
the complete action repertoire (SPRINT/MINE/MINE_DOWN/BOAT_CRAWL/BRIDGE/
BRIDGE_UP/FALL/CLIMB) demonstrated in one place, all computed by the actual
Java solver (not Python's pathfind.py, which has no BRIDGE_UP).
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from build_visualization import export_scenario, crop_and_export
from load_wbin import load_wbin

HERE = Path(__file__).resolve().parent
PATHS_DIR = HERE / "paths"
LAB_DIR = HERE.parent / "java" / "benchmark" / "data" / "lab"
REGION_DIR = HERE.parent / "java" / "benchmark" / "data"

LAB_SCENARIOS = [
    ("wall_world", "Find the gap (bedrock wall)"),
    ("bridge_world", "Bridge across chasm"),
    ("cave_shortcut_world", "Mine shortcut vs detour"),
    ("ledge_bedrock_world", "Sheer ledge -- BRIDGE_UP only way up"),
    ("ledge_lava_world", "Sheer ledge -- rise above lava"),
    ("straight_up_world", "Pure vertical shaft (BRIDGE_UP)"),
    ("floating_islands_world", "Floating islands (bridge across + up)"),
    ("rolling_terrain_world", "Mixed terrain (ravine/wall/overhang)"),
]

scenarios = {}

for key, label in LAB_SCENARIOS:
    world, start, goal, blocks = load_wbin(LAB_DIR / f"{key}.wbin")
    path_data = json.loads((PATHS_DIR / f"{key}.json").read_text())
    if not path_data.get("found"):
        print(f"skip {key}: not found")
        continue
    scenarios[key] = export_scenario(
        world, path_data["path"], path_data["actions"], start, goal,
        path_data["cost"], path_data["expansions"],
    )

# Real region: k1_r_neg1_0, the region that went from cost=74.57 to 43.22
# once BRIDGE_UP + ratio=0.0 landed -- crop around the path since the full
# world is 176x128x192.
world, start, goal, blocks = load_wbin(REGION_DIR / "k1_r_neg1_0.wbin")
path_data = json.loads((PATHS_DIR / "k1_r_neg1_0.json").read_text())
scenarios["real_k1_neg1_0"] = crop_and_export(
    world, path_data["path"], path_data["actions"], start, goal,
    path_data["cost"], path_data["expansions"],
)

from build_visualization import build_html

out_path = Path(__file__).resolve().parent.parent / "viz_final_algo.html"
build_html(scenarios, output_path=str(out_path))

# build_html auto-labels unknown scenario keys with the key itself (lab_*)
# or "Real Nether: <rest>" (real_*) -- patch those exact auto-generated
# strings to friendlier labels now that we know what they actually are.
html = out_path.read_text(encoding="utf-8")
label_map = {key: label for key, label in LAB_SCENARIOS}
for key, label in label_map.items():
    html = html.replace(f'"{key}": "{key}"', f'"{key}": "{label}"')
html = html.replace(
    '"real_k1_neg1_0": "Real Nether: k1_neg1_0"',
    '"real_k1_neg1_0": "REAL TERRAIN: k1/r.-1.0 (cost 74.57->43.22 w/ BRIDGE_UP)"',
)
out_path.write_text(html, encoding="utf-8")

print(f"wrote {out_path} with {len(scenarios)} scenarios: {list(scenarios)}")
