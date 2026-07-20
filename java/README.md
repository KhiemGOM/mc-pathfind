# Java port (search engine only)

Java port of `../pathfind.py`'s weighted A* search, for raw search-speed
comparison against the Python baseline. Does NOT port `.mca`/NBT parsing,
replanning, or visualization -- those stay Python-side. See
`../export_world_bin.py` for the bridge: it dumps an already-converted world
array to a flat binary file (`.wbin`) this project loads.

## Modules

- **`core`** -- `World`, state encoding, the weighted A* solver (fastutil for
  primitive-keyed maps/sets). Forward, bidirectional, and parallel-bidirectional
  variants.
- **`benchmark`** -- CLI (`BenchmarkCli`) that loads a `.wbin` file and times
  `WeightedAStar`/`BidirectionalWeightedAStar`/`ParallelBidirectionalWeightedAStar`
  search only (no world-load time), mirroring the Python epsilon-retry ladder.
- **`pearl`** -- **experimental**, not depended on by `core` or `benchmark`.
  A parallel fork of the solver that folds `EdgeRules.pearlEdges` (an Ender
  Pearl throw/movement action) into ordinary walking moves, plus the
  dense-integer-state-ID + decrease-key-heap optimizations described in
  `../experiments/SESSION_LOG.md`. Self-contained (only depends on fastutil +
  the JDK, not on `core`). Has its own CLI, `PearlBenchmarkCli`, for
  benchmarking against the same `.wbin` regions `BenchmarkCli` uses.

## Building

The wrapper jar (`gradle/wrapper/gradle-wrapper.jar`) is not committed -- it
was generated in a network-sandboxed environment that couldn't download it or
run Gradle at all (even loopback networking was blocked, which the Gradle
daemon needs). Run once, on a machine with normal network access:

```
gradle wrapper --gradle-version 8.14.3
```

(or just use a locally installed `gradle` directly instead of `./gradlew` --
both work identically once the wrapper jar exists). After that:

```
./gradlew :core:test
./gradlew :benchmark:run --args="<path-to-world.wbin>"
./gradlew :pearl:run --args="<path-to-world.wbin>"
```

## Compiling without Gradle

No jar is vendored in the repo -- fetch `fastutil` from Maven Central once:

```
curl -sL -o fastutil.jar \
  https://repo1.maven.org/maven2/it/unimi/dsi/fastutil/8.5.18/fastutil-8.5.18.jar
```

Then, from `java/`:

```
javac -cp fastutil.jar -d out $(find core/src/main/java -name '*.java')
java  -cp "out;fastutil.jar" dev.mcpathfind.bench.BenchmarkCli <world.wbin>
```

(swap `;` for `:` on macOS/Linux). `experiments/setup.sh` automates exactly
this for a quick sanity check.
