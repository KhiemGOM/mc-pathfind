# Java pathfinder optimization session -- full log

This document exists so a fresh Claude Code instance (or the person) can pick
up exactly where this session left off, without re-deriving any of the
reasoning or re-running experiments that are already settled.

**Target metric, as defined by the person early in this session:** only the
SUCCESS-CASE solve time matters (the epsilon rung in the retry ladder that
actually finds a path). Failed/lower-epsilon rungs are expected test overhead
and are NOT something to optimize away. All benchmarking in this log follows
that rule -- numbers quoted are for the succeeding rung only, unless stated
otherwise.

**Primary test region:** `k2_r_0_0.wbin` (368x128x368, the largest region in
`benchmark/data/`, converted from the largest raw `.mca` file in
`regions/k2/`). This was identified mid-session as "the real usecase" after
the person described it from memory ("~3M expansions, 1.x time" -- actually
2,773,428 total expansions across the full ladder, ~12.5s total, with the
succeeding eps=1.5 rung alone at 773,427 expansions / ~4s in the original
code). All tuning/threshold decisions in this log were made against this
region; other regions (`k1_*`, `long1_*`) were used to check for regressions,
not to tune.

**Environment note:** all profiling and timing in this log was done on a
single-CPU-core sandbox (`nproc` = 1). Wall-clock numbers should be
considered directionally correct but re-measured on a real multi-core
machine before trusting exact ratios, especially anything involving
`ParallelBidirectionalWeightedAStar` (see part 2).

---

## Part 1: DSA-level fixes (dense state IDs, decrease-key heap)

### Finding: heap sift operations dominated wall-clock, not the algorithm

JFR profiling (`jdk.ExecutionSample`, `settings=profile`) on the real region
showed **~70-73% of total search wall-clock was `siftUp`/`siftDown`** inside
the binary heap (`LongDoubleOpenHeap`), across every solver variant
(`WeightedAStar`, `BidirectionalWeightedAStar`). This was the single biggest
surprise of the session: it means the SLOWNESS WAS SUBSTRATE COST (hashing,
heap bookkeeping), not algorithm quality. Bidirectional search's expansion
count was already good (2.5x-8x fewer expansions than forward-only on real
regions) -- the algorithm was never the bottleneck, the bookkeeping was.

Root cause: `WeightedAStar`/`BidirectionalWeightedAStar` originally used
`Long2DoubleOpenHashMap`/`Long2LongOpenHashMap`/`Long2ObjectOpenHashMap`/
`LongOpenHashSet` (fastutil), all keyed by the packed-`long` state, PLUS a
heap with NO decrease-key: every `relax()` that improved a state's gScore
pushed a brand-new heap entry rather than updating one in place, leaving the
old (stale) entry to be found and discarded later. Measured on the real
region: **2.59x push-to-expansion ratio, ~31% of all heap pops wasted on
already-closed (stale) entries.**

### Fix: dense integer state IDs + array-backed everything

New files: `StateIdMap.java`, `IntIdOpenHeap.java`.

- `StateIdMap`: hands out small dense ints (0, 1, 2, ...) for packed-long
  states, ONE hashmap lookup per DISTINCT state ever seen (not per touch).
- `IntIdOpenHeap`: binary heap storing dense ids instead of packed longs,
  with TRUE decrease-key via a flat `int[] positionOf` array indexed by id
  (not a hashmap -- see the "first attempt was a regression" note below).
- `gScore`, `cameFromId`, `actionUsed`, `closed` all became plain arrays
  indexed by dense id, replacing the four fastutil hashmaps.

**Important failed intermediate attempt, kept in code comments as a
warning:** the first decrease-key attempt kept packed-long keying and added
a `Long2IntOpenHashMap` to track each state's heap position. This correctly
eliminated duplicate entries but was measurably SLOWER overall (~4.1s ->
~5.0-5.3s on the real region), because every heap-internal `swap()` -- which
fires far more often than pushes, on every step of every siftUp/siftDown --
now paid two hashmap `put()`s to keep the index in sync. The dense-id/array
version fixes this: array reads/writes during sifting cost the same
regardless of frequency, unlike hashmap ops.

### Results (real region, success-case rung only)

| | `WeightedAStar` | `BidirectionalWeightedAStar` |
|---|---|---|
| Before (hashmap-based) | ~4.0s | ~8.3s |
| After (dense-id) | ~3.0s | ~3.0s |
| Speedup | ~25% | **~2.7x** |

Correctness verified: identical path (node count, sim cost, action counts)
before/after in both solvers, confirmed by re-running the ORIGINAL
hashmap-based code side-by-side after the rewrite (not just trusting the
new code's own output). `PathValidator` clean in both.

**Scope note:** `ParallelBidirectionalWeightedAStar` was deliberately LEFT ON
THE OLD HASHMAP SCHEME, by explicit agreement with the person, to keep the
refactor's blast radius small. It still uses the original `LongDoubleOpenHeap`
(preserved unchanged, alongside the new `IntIdOpenHeap` -- both files coexist
in `core/src/main/java/dev/mcpathfind/core/`). **This is a real, known,
not-yet-captured optimization opportunity** if someone wants to extend the
dense-id treatment there too -- expect a similar or larger win given
`ParallelBidirectionalWeightedAStar` has the most hashmap surface area of any
solver (two full sets of structures across two worker threads, plus the
`fwdPosBestG`/`fwdPosBestState` cross-side projection maps).

**A pre-existing, NOT newly introduced bug was found and reproduced (not
fixed) during this work:** on `k2_r_0_0` at eps=1.5, both the original and
rewritten `BidirectionalWeightedAStar` produce a path with
`PathValidator` reporting `step 402: FALL cuts a blocked diagonal corner`.
Confirmed byte-identical between old and new code (same path, same cost,
same action counts, same violation) by rebuilding the original solver
side-by-side. This is the diagonal-corner-cutting issue flagged (but never
fixed) in an earlier session on the Python prototype. Still open.

---

## Part 2: Per-action candidate waste profiling

Separately from the heap/DSA work, instrumented `relax()` to count, per
`Action` enum value: how many candidates were OFFERED (generated by
`EdgeRules`), how many actually IMPROVED a gScore, and how many were
rejected because the target was already closed. On the real region
(pre-AirPotential, dense-id baseline):

```
action            offered     improved   improve%
SPRINT             514717        96759      18.8%
MINE              2388257       271039      11.3%   <-- worst offender by volume
MINE_DOWN          620081       438061      70.6%   <-- best-behaved action
BOAT_CRAWL        2033620       702621      34.6%
BRIDGE            1089093       382329      35.1%
FALL              1081921        89013       8.2%
CLIMB              344159        23773       6.9%
TOTAL             8071848      2003595      24.8%
```

MINE was the single largest source of wasted candidate generation by
absolute volume (2.39M candidates, ~89% wasted). This motivated Part 3.

Separately, path-shape analysis (walking the final `path[]`/`actions[]` of
a successful search) found: 80% of the final path's actions are SPRINT, but
the straight-line SPRINT runs are SHORT (average 2.01 blocks, longest single
run only 16 blocks, 59% of runs are exactly 1 block). **Conclusion: an
exponential/ray-marching jump-ahead scheme (1,2,4,8,16 blocks) was
considered and NOT pursued** -- this region's terrain doesn't have the long
straight corridors that technique needs to pay off. This may be worth
revisiting on a genuinely flat biome (plains/desert/frozen ocean) if one
becomes available, but not on the regions in this repo.

---

## Part 3: "Air potential" MINE pruning -- BFS-based

### Design (see `AirPotentialField.java`, `AirPotential.java` for full docs)

One-time precompute per search: BFS over PURE AIR connectivity (only
`!BlockType.isSolid()` checked, no walkability/legality rules at all --
deliberately the cheapest possible connectivity signal), seeded at the
search's START position, coarsened to 4x4x4 chunks for cheapness. Produces
`chunkBfsDist[]`, a chunk-hop distance field, with a sentinel (`UNREACHED`)
for chunks the flood-fill never touched.

For a MINE candidate at `(nx,y,nz)` from current `(x,y,z)`:

```
delta = bfsDist[chunk(nx,y,nz)] - bfsDist[chunk(x,y,z)]
f(delta) = 0                          if delta <= 0  ("old ground", already-open space)
         = 1 - exp(-delta/scaleBfs)   if delta > 0   (asymptotic ramp toward 1)
g(dist)  = 1 / euclideanDistSquared   (classic inverse-square: thin wall = high, long tunnel = low)
potential = f(delta) * g(dist)        (bounded in [0,1), high only when BOTH new-pocket AND thin-wall)
```

An `UNREACHED` chunk on either endpoint yields `potential = 0` -- "no
signal" is treated as "assume not worth it", not given a free pass.

### Design iteration history (all three variants exist in git history / can
be reconstructed from this log; only the FINAL variant, hard prune, is live
in the current code)

1. **Soft DISCOUNT** (first attempt, wrong polarity): `cost *= (1 -
   potential)`, i.e. reward good candidates, leave everything else at
   baseline. Measured effect: essentially NONE (MINE improve rate 11.3% ->
   11.5%) and total wall-clock got WORSE (~3.0s -> ~4.0s on the real
   region), because the common case (most MINE candidates, which are
   wasteful) was left completely untouched -- only the rare good candidate
   got (mildly) cheaper. **Root lesson: this never reduces the OFFERED
   count, only reshuffles priority among candidates that were always going
   to be generated anyway.**

2. **Soft PENALTY** (corrected polarity, still soft): `cost *= (1 +
   penaltyStrength * (1 - potential))`, i.e. expensive by default, relieved
   only when potential is high. Measured: MINE offered 2.39M -> 2.15M
   (-10%), total offered -7.5%, expansions -8.2%, improve rate 24.8% ->
   26.5%. Real but modest improvement in CANDIDATE QUALITY, but wall-clock
   got WORSE (~3.0s -> ~3.6s) because the extra per-candidate overhead of
   computing `potential()` for every single MINE candidate (even ones that
   still get pushed) wasn't paid back by the modest expansion reduction.

3. **Hard PRUNE** (final, live in current code): `shouldPruneMine()`
   returns true if `potential < MINE_PRUNE_THRESHOLD` (currently 0.02 in
   `EdgeRules.MINE_PRUNE_THRESHOLD`) -- SKIP generating the candidate
   entirely (no heap push, no cost computation at all) rather than just
   penalizing cost. This is a genuine prune with a real completeness/
   optimality risk (see below), not just a soft deterrent.

   Per explicit instruction from the person ("fuck the penalty btw, only
   pruning"), the soft-penalty code (`adjustedMineCost`) was REMOVED
   entirely from `AirPotential` once the hard prune existed and was shown
   to make the penalty redundant (anything surviving the prune already
   passed the potential bar; further cost-shaping on survivors bought
   nothing, just paid for extra arithmetic).

### Results, hard prune only (current live code), real region k2_r_0_0

```
MINE offered:        2,388,257 -> 121,094   (-94.9%)
MINE improve rate:    11.3%    -> 19.3%     (nearly doubled -- survivors are much higher quality)
Total offered:        8,071,848 -> 5,397,132 (-33.1%)
Total improve rate:   24.8%    -> 32.5%
Total expansions:      773,427 -> 673,005   (-13.0%)
Wall-clock (warmed):  ~3,000ms -> ~2,784ms  (best result of any variant tried)
Path cost:             98.26s  -> 97.93s    (marginally BETTER, not just comparable)
```

### KNOWN, ACCEPTED TRADEOFF -- read before changing MINE_PRUNE_THRESHOLD

On `k1_r_0_0` (a DIFFERENT, smaller region used only for regression
checking, NOT the tuning target), the hard prune makes path quality WORSE:

```
NONE (true optimum):     cost=21.68, expansions=19,016
AIR_POTENTIAL (pruned):  cost=26.27 (+21.2% worse), expansions=61,696 (MORE, not fewer!)
```

This is real and reproducible: the prune occasionally blocks a MINE step
that was genuinely part of the optimal (or a much cheaper) route, forcing a
costlier detour, AND can require MORE total expansions on regions where
baseline search already converges fast (small/easy regions don't benefit
from candidate-volume reduction the way the large real-target region does).
`PathValidator` still reports `OK` in all cases -- the found path is always
VALID, just not always as cheap as it could be. **This tradeoff was
explicitly accepted by the person given the target region is k2_r_0_0
specifically; it was not "fixed" or tuned away.** If someone wants to
revisit this, the threshold (currently 0.02) is the first thing to try
loosening, trading back some of k2's speedup for better k1 path quality.

---

## Part 4: Manhattan-distance MINE prune (alternative, NOT a replacement)

Per explicit instruction ("lets try a simpler prune... dont replace air
potential method now"), a second, independent, much simpler prune was
added as an ALTERNATIVE to A/B against AirPotential -- both exist side by
side in the code, toggled via `WeightedAStar.minePruneMode` (a public
static field, enum `MinePruneMode { NONE, AIR_POTENTIAL, MANHATTAN }`,
currently defaulted to `AIR_POTENTIAL`).

**Rule:** `EdgeRules.mineMovesCloserToGoal()` -- prune a MINE candidate
unless it STRICTLY decreases Manhattan distance to the goal
`(goalX, goalY, goalZ)`. No BFS field, no chunk lookup -- just 4
subtractions and a comparison per candidate. Much cheaper per-call than
AirPotential, and conceptually much blunter (no "new pocket" concept at
all, purely goal-directed).

### A/B/C comparison (NONE vs AIR_POTENTIAL vs MANHATTAN), both test regions

**k2_r_0_0 (the real target):**
```
NONE            expansions=773,427   avg_ms=3,491.8   cost=98.26
AIR_POTENTIAL   expansions=673,005   avg_ms=2,036.9   cost=97.93   <- best here
MANHATTAN       expansions=746,901   avg_ms=2,205.9   cost=98.26   (found the SAME optimal path as NONE)
```

**k1_r_0_0 (regression-check region):**
```
NONE            expansions=19,016    avg_ms=308.3     cost=21.68   (true optimum)
AIR_POTENTIAL   expansions=61,696    avg_ms=371.8      cost=26.27   (+21.2% worse, MORE expansions, SLOWER than NONE)
MANHATTAN       expansions=30,569    avg_ms=267.7      cost=23.15   (+6.8% worse, but FASTER than NONE)
```

**Conclusion, not yet acted on further:** neither prune dominates
universally. AIR_POTENTIAL is the stronger performer specifically on the
real target region (k2_r_0_0) on every axis (speed, expansions, even path
cost). MANHATTAN is gentler and safer on the smaller region (much smaller
quality hit, and actually faster than doing nothing there), but a smaller
win on the actual target region. Given the target region is explicitly
k2_r_0_0, AIR_POTENTIAL was left as the active default
(`minePruneMode = AIR_POTENTIAL`), but this was NOT a final decision --
the person asked to package everything up for further experimentation on
their own machine rather than commit to one mode yet.

**Untried next step, explicitly flagged as worth trying:** stacking both
prunes (a MINE candidate must survive BOTH the AirPotential threshold AND
the Manhattan check) to see if that recovers k1's path quality while
keeping k2's speed. The `horizontalEdges` overload already supports
passing both `airPotential` (non-null) and `manhattanPruneEnabled=true`
simultaneously (see its javadoc: "If both airPotential and
manhattanPruneEnabled are supplied, a MINE candidate must survive BOTH
checks") -- `WeightedAStar` just doesn't currently exercise that combined
path, since `minePruneMode` is a mutually-exclusive enum. Wiring this up
would need either a 4th enum value (`BOTH`) or decoupling the two booleans.

---

## How to reproduce anything in this log

All commands assume you're in `mc_pathfind/java/` with a JDK (21+) on PATH
and `fastutil-8.5.18.jar` downloaded (see `experiments/setup.sh`).

```bash
# compile everything
javac -cp <fastutil.jar> -d out $(find core/src/main/java benchmark/src/main/java -name '*.java')

# single-region benchmark, default epsilon ladder, forward solver
java -Xmx3g -cp "out:<fastutil.jar>" dev.mcpathfind.bench.BenchmarkCli benchmark/data/k2_r_0_0.wbin

# force the success-case rung directly (skip the failing eps=1.0 rung to save time)
java -Xmx3g -cp "out:<fastutil.jar>" dev.mcpathfind.bench.BenchmarkCli benchmark/data/k2_r_0_0.wbin --epsilons 1.5 --max-expansions 3000000

# bidirectional / parallel solver modes
java -Xmx3g -cp "out:<fastutil.jar>" dev.mcpathfind.bench.BenchmarkCli benchmark/data/k2_r_0_0.wbin --bidirectional
java -Xmx3g -cp "out:<fastutil.jar>" dev.mcpathfind.bench.BenchmarkCli benchmark/data/k2_r_0_0.wbin --parallel

# batch mode across every region in a directory
java -Xmx3g -cp "out:<fastutil.jar>" dev.mcpathfind.bench.BenchmarkCli benchmark/data

# JFR profiling (built into the JDK, no extra tool needed)
java -Xmx3g -XX:StartFlightRecording=filename=profile.jfr,settings=profile -cp "out:<fastutil.jar>" dev.mcpathfind.bench.BenchmarkCli benchmark/data/k2_r_0_0.wbin --epsilons 1.5 --max-expansions 3000000
jfr print --events jdk.ExecutionSample profile.jfr | less   # then aggregate leaf frames (see experiments/profile_analyze.py)
```

See `experiments/profile_analyze.py` for the script used to aggregate JFR
leaf-frame samples into a hot-method ranking, and
`experiments/prune_mode_compare.java` for the A/B/C harness used in Part 4
(also droppable straight into the repo root and compiled the same way as
`BenchmarkCli`).

To switch which MINE prune is active, set the static field before calling
`search()`:
```java
WeightedAStar.minePruneMode = WeightedAStar.MinePruneMode.MANHATTAN; // or NONE, AIR_POTENTIAL
```
