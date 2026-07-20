#!/usr/bin/env bash
# Downloads fastutil (needed by core/) and does a clean full-project build.
# Run this from mc_pathfind/java/ like so:
#     bash ../experiments/setup.sh
set -euo pipefail

FASTUTIL_VERSION="8.5.18"
FASTUTIL_JAR="fastutil-${FASTUTIL_VERSION}.jar"
FASTUTIL_URL="https://repo1.maven.org/maven2/it/unimi/dsi/fastutil/${FASTUTIL_VERSION}/${FASTUTIL_JAR}"

if [ ! -f "${FASTUTIL_JAR}" ]; then
    echo "Downloading fastutil ${FASTUTIL_VERSION}..."
    curl -sL -o "${FASTUTIL_JAR}" "${FASTUTIL_URL}"
fi

echo "Compiling core + benchmark..."
rm -rf out
mkdir -p out
javac -cp "${FASTUTIL_JAR}" -d out $(find core/src/main/java benchmark/src/main/java -name '*.java')

echo ""
echo "Build OK. Quick sanity check on the real target region:"
java -Xmx3g -cp "out:${FASTUTIL_JAR}" dev.mcpathfind.bench.BenchmarkCli benchmark/data/k2_r_0_0.wbin --epsilons 1.5 --max-expansions 3000000

echo ""
echo "Setup complete. See experiments/SESSION_LOG.md for the full write-up"
echo "of what's been tried, what worked, and what's still open."
