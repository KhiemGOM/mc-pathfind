# experiments/

Scratch/investigation scripts, not maintained library code. Kept for
provenance rather than pruned, since they document *why* the solver looks the
way it does, not just what it does.

- **`SESSION_LOG.md`** -- a dev log from the Java solver's optimization pass:
  what was profiled, what turned out to be the real bottleneck (heap
  `siftUp`/`siftDown`, not the search algorithm itself), and the reasoning
  behind the "air potential" mining heuristic. Written so a later session
  (human or agent) can pick up without re-deriving settled conclusions.
- **`setup.sh`** -- one-shot fastutil download + clean `javac` build of
  `core`/`benchmark` + a sanity-check benchmark run. Run from `java/` via
  `bash ../experiments/setup.sh`.
- **`*.java`** -- one-off debug/sweep/profiling drivers used while tuning the
  Java solver (bridge-depth sweeps, prune-mode comparisons, threshold
  sweeps, parallel-bridge/mine-vs-crawl debugging, etc). Not part of the
  `java/` Gradle build; compiled standalone against `core`/`pearl` when
  needed, per the commands in `SESSION_LOG.md`.
- **`*.py`** -- helper scripts for building the two non-standard
  visualizations (`viz/viz_final_algo.html`, `viz/viz_floating_islands.html`)
  and for exporting/loading `.wbin` world dumps and lab-world fixtures
  outside the main `viz/build_visualization.py` pipeline.
- **`paths/`, `floating_islands_path.json`** -- small JSON path dumps produced
  by the scripts above, used as inputs to the two non-standard visualizations.
