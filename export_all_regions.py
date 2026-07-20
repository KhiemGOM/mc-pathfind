"""
Batch-export every .mca region file under a set of folders to .wbin files,
for testing the Java port against the same real terrain used throughout this
session. Skips regions with no walkable terrain (matches run_region_viz.py's
per-region try/except -- e.g. k2's generation-stub-only files) rather than
aborting the whole batch.

Usage:
    python3 export_all_regions.py regions/k1 regions/long1 regions/k2 --out-dir java/benchmark/data
"""

import argparse
from pathlib import Path

from export_world_bin import export_world_bin


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("region_dirs", nargs="+")
    parser.add_argument("--out-dir", default="java/benchmark/data")
    parser.add_argument("--blocks", type=int, default=32)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    exported, skipped = [], []

    for region_dir in args.region_dirs:
        region_dir = Path(region_dir)
        for mca_file in sorted(region_dir.glob("*.mca")):
            # matches run_region_viz.py's _region_key: folder + stem, so files
            # with the same r.X.Z name across k1/long1/k2 don't collide
            key = f"{region_dir.name}_{mca_file.stem.replace('.', '_').replace('-', 'neg')}"
            out_path = out_dir / f"{key}.wbin"
            try:
                export_world_bin(str(mca_file), str(out_path), blocks_available=args.blocks)
                exported.append(str(mca_file))
            except Exception as exc:
                print(f"SKIP {mca_file}: {exc}")
                skipped.append(str(mca_file))

    print(f"\nexported {len(exported)}, skipped {len(skipped)}")


if __name__ == "__main__":
    main()
