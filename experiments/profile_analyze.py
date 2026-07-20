#!/usr/bin/env python3
"""
Aggregates jdk.ExecutionSample events from a JFR recording into a ranked
list of hot LEAF methods (i.e. where the CPU was actually sampled executing,
not the full call stack). This is what was used throughout the session to
find that ~70-73% of search wall-clock was in LongDoubleOpenHeap's
siftUp/siftDown, and later to confirm that number dropped once the
dense-id/IntIdOpenHeap rewrite landed.

Usage:
    python3 profile_analyze.py path/to/recording.jfr

Requires the `jfr` CLI tool, which ships with any JDK 11+ (no separate
install needed -- it's at $JAVA_HOME/bin/jfr).

To generate a recording in the first place:
    java -Xmx3g -XX:StartFlightRecording=filename=out.jfr,settings=profile \\
        -cp "out:<fastutil.jar>" dev.mcpathfind.bench.BenchmarkCli \\
        benchmark/data/k2_r_0_0.wbin --epsilons 1.5 --max-expansions 3000000

For denser sampling (more reliable percentages), loop the search several
times in a small throwaway harness class rather than relying on a single
multi-second run -- see e.g. ProfileK2.java pattern described in
SESSION_LOG.md Part 1.
"""
import subprocess
import re
import sys
from collections import Counter


def analyze(jfr_path: str, top_n: int = 25):
    result = subprocess.run(
        ["jfr", "print", "--events", "jdk.ExecutionSample", jfr_path],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print("jfr print failed:", result.stderr, file=sys.stderr)
        sys.exit(1)

    out = result.stdout
    samples = out.split("jdk.ExecutionSample {")[1:]
    leaf_counter = Counter()

    for s in samples:
        if "stackTrace = [" not in s:
            continue
        lines = s.split("stackTrace = [")[1].split("]")[0].strip().split("\n")
        frames = [l.strip() for l in lines if l.strip() and l.strip() != "..."]
        if not frames:
            continue
        leaf = frames[0]
        m = re.match(r"([\w.$]+\.[\w<>]+)\(", leaf)
        if m:
            leaf_counter[m.group(1)] += 1

    total = sum(leaf_counter.values())
    if total == 0:
        print("No execution samples found -- recording may be too short. "
              "Try looping the search several times in one JVM invocation "
              "for denser sampling.")
        return

    print(f"Total samples parsed: {total}\n")
    print(f"=== TOP {top_n} LEAF METHODS ===")
    for method, count in leaf_counter.most_common(top_n):
        pct = 100 * count / total
        print(f"{count:6d} ({pct:5.1f}%)  {method}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <recording.jfr> [top_n]")
        sys.exit(1)
    top_n = int(sys.argv[2]) if len(sys.argv) > 2 else 25
    analyze(sys.argv[1], top_n)
