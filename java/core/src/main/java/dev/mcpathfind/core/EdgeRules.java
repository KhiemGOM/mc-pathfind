package dev.mcpathfind.core;

/**
 * Per-action edge legality/cost computation, extracted from WeightedAStar so
 * forward search and the backward half of bidirectional search share the
 * EXACT same rules -- hand-writing a separate "reverse" version of each
 * action would risk it silently diverging from forward semantics, which is
 * exactly the class of bug this project has been hunting all session
 * (the SPRINT head-clearance bug, the diagonal corner-cut bug). Any behavior
 * change here must be re-verified against the full region batch before
 * being trusted (see WeightedAStarTest / the batch benchmark).
 */
public final class EdgeRules {

    // 8 horizontal directions (cardinal + diagonal), matches DIRS_H in
    // pathfind.py. Declared first since PARKOUR_KERNEL's static
    // initializer (below) reads DIR_DX/DIR_DZ, and Java runs field
    // initializers in textual declaration order.
    public static final int[] DIR_DX = {1, -1, 0, 0, 1, 1, -1, -1};
    public static final int[] DIR_DZ = {0, 0, 1, -1, 1, -1, 1, -1};

    public static final double BOAT_CRAWL_TAX = 5.0;
    public static final double BOAT_CRAWL_SPEED = 3.0;
    public static final double SPRINT_SPEED = 5.6;
    public static final double WALK_SPEED = 4.3;
    public static final double JUMP_PENALTY = 0.15;
    public static final double PLACE_TIME = 0.20;
    public static final double BRIDGE_RISK_PENALTY = 0.5;
    public static final double BLOCK_TAX_BASE = 0.6;
    public static final double BLOCK_TAX_SCARCITY_SCALE = 3.0;
    public static final int CLUTCH_THRESHOLD = 3;
    public static final double CLUTCH_SETUP_TIME = 0.4;
    public static final double LAVA_DEATH_PENALTY = 1e6;
    public static double MINE_PRUNE_THRESHOLD = 0.02;
    // Not a modeled game mechanic -- a deliberately small artificial
    // tie-breaker against MINE/BOAT_CRAWL so insta-mine (see max() below)
    // doesn't make digging tie exactly with an equal-time SPRINT/BOAT_CRAWL
    // alternative. Must stay >= JUMP_PENALTY: CLIMB and MINE compete for the
    // exact same target cell on a single-block bump (step over it vs. dig
    // through it) -- once mining is fast, MINE's cost collapses to
    // dist/SPRINT_SPEED + this tax while CLIMB's is dist/SPRINT_SPEED +
    // JUMP_PENALTY, so a smaller tax here let MINE systematically undercut
    // CLIMB on every bump. Still small enough to never meaningfully
    // discourage a real dig CLIMB can't cross at all.
    public static double MINE_DURABILITY_TAX = 0.2;

    // --- PARKOUR (long jump; ported from pathfind.py's PARKOUR, which
    // explicitly covers "boat-jump and equivalent long-jump techniques" as
    // one action -- see parkourEdges' javadoc for the split into a cheap
    // short-jump tier and that formula as the fallback tier). ---
    public static final double MIN_PARKOUR_DIST = 2.0;
    public static final double MAX_PARKOUR_DIST = 7.0;
    public static double PARKOUR_SPEED = 6.5;
    public static double PARKOUR_RISK_BASE = 0.8;
    public static double PARKOUR_RISK_PER_BLOCK = 0.15;
    // Cheap-tier horizontal-distance ceilings, one per landing dy -- a jump
    // landing level or lower reaches farther than one that also has to gain
    // height, matching real jump-arc physics. Inside these, PARKOUR costs
    // about as much as a single CLIMB-style hop; outside, it falls back to
    // the full risk-scaled formula below.
    public static final double PARKOUR_CHEAP_MAX_DIST_FLAT = Math.sqrt(13.0);
    public static final double PARKOUR_CHEAP_MAX_DIST_UP = Math.sqrt(5.0);
    public static final double PARKOUR_CHEAP_MAX_DIST_DOWN = Math.sqrt(20.0);
    // Not a modeled game mechanic -- same tie-breaker role as
    // MINE_DURABILITY_TAX, just for the cheap PARKOUR tier.
    public static double PARKOUR_CHEAP_RISK_TAX = 0.05;

    // Cardinal-only (N/S/E/W, the same DIR_DX/DIR_DZ[0..3] axes SPRINT/
    // BRIDGE already use) instead of angle-sampled directions -- diagonal
    // PARKOUR candidates were the majority of the old kernel for barely any
    // extra reachability, since SPRINT/BRIDGE/CLIMB already cover diagonal
    // movement cheaply right up to the gap's edge; a real gap almost always
    // has a cardinal-facing crossing point too. Exact integer distances (no
    // angle/cos/sin/round approximation, no dedup needed -- a cardinal
    // direction at a given integer distance is already a single, unique
    // offset) further shrinks and simplifies the kernel: 3,4 short-tier
    // distances (the actual useful cheap-tier range) and a single
    // mid-range long-tier distance (6) for the rare boat-jump fallback,
    // whose entries are also the priciest ones to evaluate (trajectory
    // sampling scales with distance), so sampling it sparsely cuts the
    // most total work per node. Went 16-direction/8-step angled kernel
    // (~180 entries, ~66k-120k expansions/sec on real terrain, k2_r_0_0)
    // -> two-band 8-or-6-direction angled kernel (57 entries, ~120k-210k
    // exp/s) -> this cardinal-only kernel (24 entries).
    private static final int[] PARKOUR_SHORT_DISTS = {3, 4};
    private static final int[] PARKOUR_LONG_DISTS = {6};

    private record ParkourOffset(int dx, int dy, int dz, double dist) {}

    private static final ParkourOffset[] PARKOUR_KERNEL = buildParkourKernel();

    /**
     * Sparse sample of (dx,dy,dz,dist) offsets approximating valid PARKOUR
     * jumps, rather than every integer lattice point in the outer shell
     * (~1000 cells at MAX_PARKOUR_DIST=7 -- intractable per-expansion), and
     * rather than pathfind.py's original single uniform 16-direction/
     * 8-distance-step angled kernel across the whole range -- see the
     * cardinal-only rationale in the constants above. Built once at
     * class-init; parkourEdges just iterates this static array per call.
     */
    private static ParkourOffset[] buildParkourKernel() {
        java.util.List<ParkourOffset> kernel = new java.util.ArrayList<>();
        for (int d = 0; d < 4; d++) {
            for (int dist : PARKOUR_SHORT_DISTS) {
                addCardinalParkourOffsets(kernel, DIR_DX[d], DIR_DZ[d], dist);
            }
            for (int dist : PARKOUR_LONG_DISTS) {
                addCardinalParkourOffsets(kernel, DIR_DX[d], DIR_DZ[d], dist);
            }
        }
        return kernel.toArray(new ParkourOffset[0]);
    }

    private static void addCardinalParkourOffsets(java.util.List<ParkourOffset> kernel,
                                                   int dirX, int dirZ, int dist) {
        if (dist <= MIN_PARKOUR_DIST || dist > MAX_PARKOUR_DIST) {
            return;
        }
        for (int dy = -1; dy <= 1; dy++) {
            kernel.add(new ParkourOffset(dirX * dist, dy, dirZ * dist, (double) dist));
        }
    }

    /**
     * PARKOUR: a long jump across a gap. Gated behind a cheap "is there
     * actually a gap worth jumping?" check first -- scanning the kernel
     * from every node (including deep-interior flat terrain, or ordinary
     * 1-block ledges/steps that FALL/CLIMB already cross for a fraction of
     * the cost) would multiply every expansion's cost for no benefit; see
     * hasJumpableGap's javadoc for why a single non-solid-floored neighbor
     * isn't a strong enough signal on its own.
     *
     * Two cost tiers based on the landing's (dy, horizontal dist) against
     * PARKOUR_CHEAP_MAX_DIST_{FLAT,UP,DOWN}: a short jump that doesn't need
     * much of a running leap costs about as much as a single CLIMB-style
     * hop (dist/SPRINT_SPEED + JUMP_PENALTY) plus a small tie-breaker tax --
     * genuinely cheap, matching how trivial a short hop actually is.
     * Anything past that threshold falls back to pathfind.py's own
     * risk-scaled long-jump formula (PARKOUR_RISK_BASE +
     * PARKOUR_RISK_PER_BLOCK*dist + dist/PARKOUR_SPEED), which is explicitly
     * documented there as covering "boat-jump and equivalent long-jump
     * techniques" -- a real setup/commit cost for a genuinely risky leap,
     * not a trivial hop. The two tiers deliberately do NOT blend smoothly
     * at the boundary: a short hop being flatly, sharply cheaper than
     * "committing to a boat jump" is the whole point of the split.
     *
     * Trajectory and landing legality are unconditional (same for both cost
     * tiers): sampled once per block of distance along the straight-line
     * flight path for any solid/lava obstruction (a small fixed sample
     * count could skip over a single-block wall), the flight path must
     * cross at least one non-solid-floored cell (otherwise it's just
     * sailing over normal ground, not actually jumping a gap), and the
     * landing cell needs solid support with clear, lava-free feet/head.
     */
    public static void parkourEdges(World world, int x, int y, int z, int blocks, EdgeConsumer out) {
        if (!hasJumpableGap(world, x, y, z)) {
            return;
        }
        for (ParkourOffset offset : PARKOUR_KERNEL) {
            tryParkourEdge(world, x, y, z, offset.dx(), offset.dy(), offset.dz(), offset.dist(), blocks, out);
        }
    }

    /**
     * True iff there's a genuine gap to jump, not just an ordinary
     * single-block dip or step. The original version of this check ("does
     * any of the 8 adjacent columns lack solid footing?") fired just as
     * readily on a single missing block or a one-block stair-step as on a
     * real chasm -- both are common on natural terrain, and both are
     * already crossed by FALL/CLIMB for a fraction of PARKOUR's cost, so
     * running the full kernel scan for them was pure wasted work. Requires
     * the SECOND cell out in the same direction to also lack solid footing
     * (a real 2+-block-wide gap), not just the first -- this is the "1
     * block dist dont consider" filter: it doesn't change what PARKOUR can
     * ultimately produce (tryParkourEdge's own trajectory/landing checks
     * are unchanged and still the actual source of truth), only how often
     * the expensive kernel scan is attempted at all.
     */
    private static boolean hasJumpableGap(World world, int x, int y, int z) {
        for (int d = 0; d < 8; d++) {
            int adjX = x + DIR_DX[d], adjZ = z + DIR_DZ[d];
            int adj2X = x + DIR_DX[d] * 2, adj2Z = z + DIR_DZ[d] * 2;
            if (adjX < 0 || adjX >= world.sizeX || adjZ < 0 || adjZ >= world.sizeZ) {
                continue;
            }
            if (adj2X < 0 || adj2X >= world.sizeX || adj2Z < 0 || adj2Z >= world.sizeZ) {
                continue;
            }
            if (!BlockType.isSolid(world.voxelAt(adjX, y - 1, adjZ))
                    && !BlockType.isSolid(world.voxelAt(adj2X, y - 1, adj2Z))) {
                return true;
            }
        }
        return false;
    }

    /**
     * Checks one candidate (source, kernel offset) PARKOUR jump and emits it
     * if legal -- factored out of parkourEdges so reverseParkourSources can
     * call the EXACT same trajectory/landing logic per candidate source
     * instead of a hand-written "reverse" duplicate that could silently
     * diverge (same reuse-forward-logic philosophy as every other reverse
     * probe in this file). Does NOT check hasJumpableGap -- callers are
     * responsible for that, since forward checks it once for its fixed
     * source before the kernel loop (cheap short-circuit for the common
     * no-gap case), while the reverse probe must check it per candidate
     * source since only the TARGET is fixed there.
     */
    private static void tryParkourEdge(World world, int x, int y, int z,
                                        int kdx, int kdy, int kdz, double dist,
                                        int blocks, EdgeConsumer out) {
        int nx = x + kdx, ny = y + kdy, nz = z + kdz;
        if (nx < 0 || nx >= world.sizeX || ny < 0 || ny >= world.sizeY || nz < 0 || nz >= world.sizeZ) {
            return;
        }

        int nSamples = Math.max(2, (int) Math.round(dist));
        boolean trajectoryBlocked = false;
        boolean crossesGap = false;
        for (int s = 1; s < nSamples; s++) {
            double t = (double) s / nSamples;
            int sx = (int) Math.round(x + kdx * t);
            int sz = (int) Math.round(z + kdz * t);
            int sy = (int) Math.round(y + kdy * t);
            int sampleFoot = world.voxelAt(sx, sy, sz);
            int sampleHead = world.voxelAt(sx, sy + 1, sz);
            if (BlockType.isSolid(sampleFoot) || BlockType.isLava(sampleFoot)
                    || BlockType.isSolid(sampleHead) || BlockType.isLava(sampleHead)) {
                trajectoryBlocked = true;
                break;
            }
            if (!BlockType.isSolid(world.voxelAt(sx, sy - 1, sz))) {
                crossesGap = true;
            }
        }
        if (trajectoryBlocked || !crossesGap) {
            return;
        }

        int landTarget = world.voxelAt(nx, ny, nz);
        int landHead = world.voxelAt(nx, ny + 1, nz);
        int landSupport = world.voxelAt(nx, ny - 1, nz);
        if (!BlockType.isSolid(landSupport)) {
            return;
        }
        if (BlockType.isSolid(landTarget) || BlockType.isLava(landTarget)) {
            return;
        }
        if (BlockType.isSolid(landHead) || BlockType.isLava(landHead)) {
            return;
        }

        boolean cheap = (kdy == 0 && dist < PARKOUR_CHEAP_MAX_DIST_FLAT)
            || (kdy == 1 && dist < PARKOUR_CHEAP_MAX_DIST_UP)
            || (kdy == -1 && dist < PARKOUR_CHEAP_MAX_DIST_DOWN);
        double cost = cheap
            ? dist / SPRINT_SPEED + JUMP_PENALTY + PARKOUR_CHEAP_RISK_TAX
            : PARKOUR_RISK_BASE + PARKOUR_RISK_PER_BLOCK * dist + dist / PARKOUR_SPEED;
        out.accept(StateCodec.pack(nx, ny, nz, blocks, false), cost, Action.PARKOUR);
    }

    /**
     * Predecessors of a PARKOUR landing at (tx,ty,tz): for each kernel
     * offset, the candidate source is (tx-kdx,ty-kdy,tz-kdz) -- inverted
     * per entry rather than called once from a fixed source, since the
     * ledge-gate and trajectory legality both depend on the SOURCE, which
     * varies per kernel entry here (the target is what's fixed instead).
     * O(kernel size) per call, same complexity class as parkourEdges
     * itself -- NOT O(kernel size squared), since this calls
     * tryParkourEdge directly rather than re-invoking parkourEdges (which
     * would redundantly re-scan its own full kernel) per candidate source.
     *
     * This exists because omitting it is not just "no bidirectional
     * speedup" the way BRIDGE_UP's backward-chain limitation is: confirmed
     * via LabFidelityCheck on bridge_world.wbin that without it,
     * bidirectional/parallel settle for a real WORSE-cost route (20.27 vs
     * forward's 10.75) whenever the true optimum requires a PARKOUR edge
     * backward's graph doesn't know about -- the shared meet-in-the-middle
     * termination check can fire on a worse backward-reachable mu before
     * forward's own search reaches the cheaper PARKOUR-including goal.
     *
     * Like SPRINT/MINE/CLIMB (see reverseNonCrawlEdges' javadoc), PARKOUR's
     * legality and cost don't reference the source's crawling flag at all,
     * so both source-crawling states are valid predecessors -- emitted
     * together per matched kernel entry rather than re-running the geometry
     * check twice for an outcome that can't differ between them.
     */
    public static void reverseParkourSources(World world, int tx, int ty, int tz, int blocks, EdgeConsumer out) {
        for (ParkourOffset offset : PARKOUR_KERNEL) {
            int sx = tx - offset.dx();
            int sy = ty - offset.dy();
            int sz = tz - offset.dz();
            if (sx < 0 || sx >= world.sizeX || sy < 0 || sy >= world.sizeY || sz < 0 || sz >= world.sizeZ) {
                continue;
            }
            if (!hasJumpableGap(world, sx, sy, sz)) {
                continue;
            }
            tryParkourEdge(world, sx, sy, sz, offset.dx(), offset.dy(), offset.dz(), offset.dist(), blocks,
                (toState, cost, action) -> {
                    if (action != Action.PARKOUR) {
                        return;
                    }
                    if (StateCodec.unpackX(toState) != tx || StateCodec.unpackY(toState) != ty
                            || StateCodec.unpackZ(toState) != tz) {
                        return;
                    }
                    out.accept(StateCodec.pack(sx, sy, sz, blocks, false), cost, Action.PARKOUR);
                    out.accept(StateCodec.pack(sx, sy, sz, blocks, true), cost, Action.PARKOUR);
                });
        }
    }

    /**
     * True iff mining from (x,y,z) to (nx,y,nz) strictly decreases Manhattan
     * distance to the NEAREST goal point. Used by the alternate
     * (non-AirPotential) MINE prune: "don't mine backward/sideways relative
     * to the goal". Note this is intentionally a STRICT decrease (not <=)
     * -- a MINE that leaves Manhattan distance unchanged (e.g. purely
     * lateral relative to the goal in one axis while irrelevant in another)
     * is treated the same as one that increases it: not making progress, so
     * prune it.
     */
    public static boolean mineMovesCloserToGoal(int x, int y, int z, int nx, int nz, GoalPoints goal) {
        int distBefore = Integer.MAX_VALUE, distAfter = Integer.MAX_VALUE;
        for (int i = 0; i < goal.size(); i++) {
            int gx = goal.x(i), gy = goal.y(i), gz = goal.z(i);
            distBefore = Math.min(distBefore, Math.abs(x - gx) + Math.abs(y - gy) + Math.abs(z - gz));
            distAfter = Math.min(distAfter, Math.abs(nx - gx) + Math.abs(y - gy) + Math.abs(nz - gz));
        }
        return distAfter < distBefore;
    }

    public static final double SQRT2 = Math.sqrt(2.0);

    private static double diagDist(int dx, int dz) {
        return (dx != 0 && dz != 0) ? SQRT2 : 1.0;
    }

    // Bound on how far reverse-FALL scans upward from a landing spot looking
    // for valid step-off heights. Real nether rooms/caverns are rarely much
    // taller than this; bidirectional search falls back to forward-only if
    // backward search can't connect, so an occasional missed tall-drop
    // predecessor costs a speedup opportunity, never correctness.
    public static final int MAX_REVERSE_FALL_SCAN = 24;

    @FunctionalInterface
    public interface EdgeConsumer {
        void accept(long toState, double cost, Action action);
    }

    public record LandingResult(boolean found, int landingY, int drop, boolean passedLava) {}

    private static final double[] BLOCK_TAX_TABLE = buildBlockTaxTable();

    private static double[] buildBlockTaxTable() {
        double[] t = new double[StateCodec.MAX_BLOCKS + 1];
        t[0] = Double.POSITIVE_INFINITY;
        for (int i = 1; i < t.length; i++) {
            t[i] = BLOCK_TAX_BASE * (1.0 + BLOCK_TAX_SCARCITY_SCALE / i);
        }
        return t;
    }

    public static double blockTax(int blocksRemaining) {
        if (blocksRemaining < 0) {
            return Double.POSITIVE_INFINITY;
        }
        if (blocksRemaining < BLOCK_TAX_TABLE.length) {
            return BLOCK_TAX_TABLE[blocksRemaining];
        }
        return BLOCK_TAX_BASE * (1.0 + BLOCK_TAX_SCARCITY_SCALE / blocksRemaining);
    }

    /**
     * MC never lets you cut a diagonal corner when BOTH flanking columns are
     * solid. If only one is solid you can still squeeze past it.
     */
    public static boolean diagonalCornerBlocked(World world, int x, int y, int z, int dx, int dz) {
        if (dx == 0 || dz == 0) {
            return false;
        }
        boolean corner1 = BlockType.isSolid(world.voxelAt(x + dx, y, z))
                || BlockType.isSolid(world.voxelAt(x + dx, y + 1, z));
        boolean corner2 = BlockType.isSolid(world.voxelAt(x, y, z + dz))
                || BlockType.isSolid(world.voxelAt(x, y + 1, z + dz));
        return corner1 && corner2;
    }

    /** Matches find_landing_y(max_drop=None) in pathfind.py exactly. */
    public static LandingResult findLandingY(World world, int x, int y, int z) {
        int drop = 0;
        int yy = y;
        boolean passedLava = false;
        while (yy >= 0) {
            int v = world.voxelAt(x, yy, z);
            if (BlockType.isLava(v)) {
                passedLava = true;
            }
            if (BlockType.isSolid(v)) {
                int landingY = yy + 1;
                int landingCell = world.voxelAt(x, landingY, z);
                int landingHead = world.voxelAt(x, landingY + 1, z);
                if (BlockType.isLava(landingCell) || BlockType.isLava(landingHead)) {
                    passedLava = true;
                }
                return new LandingResult(true, landingY, drop, passedLava);
            }
            yy -= 1;
            drop += 1;
        }
        return new LandingResult(false, -1, drop, passedLava);
    }

    /**
     * Horizontal moves for one direction from (x,y,z): SPRINT, MINE,
     * BOAT_CRAWL, BRIDGE (flat and rise-above-lava), FALL. Direct port of
     * pathfind.py's get_neighbors horizontal-move phase, including the
     * head-clearance fix (target_or_head_blocked).
     */
    public static void horizontalEdges(World world, int x, int y, int z, int blocks, boolean crawling,
                                        int dx, int dz, double toolMultiplier, EdgeConsumer out) {
        horizontalEdges(world, x, y, z, blocks, crawling, dx, dz, toolMultiplier, null, out);
    }

    /**
     * Same as the 9-arg horizontalEdges, but with an optional AirPotential
     * (null = no adjustment, exactly the original behavior) applied to
     * discount MINE's cost when the dig target is both chunk-BFS-distant
     * (probably a different, unopened air pocket) and Euclidean-close (a
     * thin wall rather than a long tunnel). See AirPotential's javadoc for
     * the full rationale and formula. Only MINE is adjusted -- BOAT_CRAWL,
     * SPRINT, etc. are untouched; this is deliberately scoped narrow since
     * MINE was profiled as the single biggest source of wasted candidate
     * generation (2.39M candidates offered, only 11.3% ever useful, on a
     * real region).
     */
    public static void horizontalEdges(World world, int x, int y, int z, int blocks, boolean crawling,
                                        int dx, int dz, double toolMultiplier, AirPotential airPotential, EdgeConsumer out) {
        horizontalEdges(world, x, y, z, blocks, crawling, dx, dz, toolMultiplier, airPotential, false, null, out);
    }

    /**
     * Same as the AirPotential overload, but adds an ALTERNATE, independent
     * MINE prune: manhattanPruneEnabled=true skips generating a MINE
     * candidate if it doesn't strictly decrease Manhattan distance to the
     * nearest goal point -- i.e. "don't mine backward/sideways relative to
     * the goal". This is a much cheaper check than AirPotential (a
     * handful of subtractions, no BFS field lookup) and a much blunter one
     * -- it has no notion of "new pocket vs old ground", it only asks
     * "did this get me closer to the goal in raw Manhattan terms". Tried
     * as an alternative to AirPotential, not a replacement -- both exist
     * side by side so they can be A/B'd. If both airPotential and
     * manhattanPruneEnabled are supplied, a MINE candidate must survive
     * BOTH checks (this method applies airPotential's prune first, then
     * the Manhattan check, short-circuiting on whichever fails first).
     * goal may be null iff manhattanPruneEnabled is false (never
     * dereferenced in that case, since && short-circuits).
     */
    public static void horizontalEdges(World world, int x, int y, int z, int blocks, boolean crawling,
                                        int dx, int dz, double toolMultiplier, AirPotential airPotential,
                                        boolean manhattanPruneEnabled, GoalPoints goal, EdgeConsumer out) {
        horizontalEdges(world, x, y, z, blocks, crawling, dx, dz, toolMultiplier, airPotential,
                manhattanPruneEnabled, goal, false, out);
    }

    /**
     * EXPERIMENTAL tunables for the NearestAirDistance A/B arm -- see its
     * class javadoc. Not deeply tuned, just checked against an obviously bad
     * default: a light sweep (radius 1/2/3/4/6) on 2 regions found
     * radius<=2 reproduces MANHATTAN's quality loss (e.g. long1_r_0_0 cost
     * 72.93 vs the true 69.66), while radius>=3 recovers optimal cost on
     * both swept regions with expansions still modestly below NONE's. 3
     * chosen as the smallest radius that doesn't cost path quality on
     * either swept region -- not swept further than that.
     */
    public static int NEAREST_AIR_MAX_RADIUS = 6;
    public static int NEAREST_AIR_PRUNE_RADIUS = 3;

    /**
     * Same as the MANHATTAN overload, but adds a THIRD, independent,
     * EXPERIMENTAL MINE prune: nearestAirPruneEnabled=true skips a MINE
     * candidate unless NearestAirDistance says open space is within
     * NEAREST_AIR_PRUNE_RADIUS blocks (straight-line, source cell
     * excluded -- see that class's javadoc for why the exclusion is
     * required). This exists purely to be A/B'd against AirPotential
     * (chunk-BFS connectivity) and MANHATTAN (distance to goal) for a
     * blog comparison -- not a production option, never combined with the
     * other two in practice (WeightedAStar.minePruneMode is one arm at a
     * time), but the prune checks still short-circuit in the same
     * left-to-right order as the other two if a caller ever did combine
     * them.
     */
    public static void horizontalEdges(World world, int x, int y, int z, int blocks, boolean crawling,
                                        int dx, int dz, double toolMultiplier, AirPotential airPotential,
                                        boolean manhattanPruneEnabled, GoalPoints goal,
                                        boolean nearestAirPruneEnabled, EdgeConsumer out) {
        horizontalEdges(world, x, y, z, blocks, crawling, dx, dz, toolMultiplier, airPotential,
                manhattanPruneEnabled, goal, nearestAirPruneEnabled, null, null, out);
    }

    /**
     * EXPERIMENTAL tunables for the HYBRID A/B arm -- see its branch's
     * comment below and StoneDensityField's javadoc. Not deeply tuned.
     */
    public static double HYBRID_STONE_DENSITY_THRESHOLD = 0.35;

    /**
     * Same as the NEAREST_AIR overload, but adds a FOURTH, independent,
     * EXPERIMENTAL MINE prune: hybridDensityField != null (both it and
     * hybridAirPotential are non-null together, or both null -- same
     * "non-null means enabled" convention as airPotential's own field, both
     * built once in WeightedAStar.search()'s setup) switches between
     * AirPotential (cheap, O(1) chunk-BFS lookup -- wins on average across
     * long open-terrain routes) and NearestAirDistance (never measured
     * worse than no pruning at all, but O(radius^3) per candidate) based on
     * StoneDensityField's O(1) local density lookup: below
     * HYBRID_STONE_DENSITY_THRESHOLD, this is ordinary open terrain, so use
     * the cheap AirPotential check; at or above it, this is real bastion
     * wall material (not just generically solid terrain -- see
     * StoneDensityField's javadoc for why STONE-density and not isSolid()-
     * density), so pay NearestAirDistance's extra cost only here, where
     * AirPotential's chunk-BFS novelty signal was empirically found to lose
     * its discriminating power (measured on a real bastion-interior route:
     * AirPotential alone was worse than no pruning on cost, expansions, AND
     * wall-clock all at once).
     */
    public static void horizontalEdges(World world, int x, int y, int z, int blocks, boolean crawling,
                                        int dx, int dz, double toolMultiplier, AirPotential airPotential,
                                        boolean manhattanPruneEnabled, GoalPoints goal,
                                        boolean nearestAirPruneEnabled,
                                        AirPotential hybridAirPotential, StoneDensityField hybridDensityField,
                                        EdgeConsumer out) {
        int nx = x + dx, nz = z + dz;
        if (nx < 0 || nx >= world.sizeX || nz < 0 || nz >= world.sizeZ) {
            return;
        }
        if (diagonalCornerBlocked(world, x, y, z, dx, dz)) {
            return;
        }
        double dist = diagDist(dx, dz);

        int belowTarget = world.voxelAt(nx, y - 1, nz);
        int target = world.voxelAt(nx, y, nz);
        int head = world.voxelAt(nx, y + 1, nz);

        if (BlockType.isSolid(target)) {
            if (BlockType.isUnbreakable(target) || BlockType.isUnbreakable(head) || BlockType.isLava(head)) {
                // normal 2-tall mining blocked; boat-crawl may still work below
            } else if (airPotential != null && airPotential.shouldPruneMine(MINE_PRUNE_THRESHOLD, x, y, z, nx, y, nz)) {
                // hard prune: potential says this dig is very unlikely to lead
                // anywhere useful (old ground and/or a thick wall) -- skip
                // generating the candidate entirely. See
                // AirPotential.shouldPruneMine's javadoc for the
                // completeness tradeoff this accepts. NOTE: the soft cost
                // penalty (adjustedMineCost) was tried alongside this and
                // removed -- once pruning exists, the penalty on SURVIVING
                // candidates added no measurable benefit (they already
                // passed the potential bar) and only cost extra per-call
                // overhead, so MINE cost is left untouched (baseMineCost)
                // for anything that survives the prune.
            } else if (manhattanPruneEnabled && !mineMovesCloserToGoal(x, y, z, nx, nz, goal)) {
                // alternate, independent prune: this dig doesn't strictly
                // decrease Manhattan distance to the goal -- skip it. Much
                // cheaper than AirPotential (no BFS lookup) but blunter: it
                // has no "new pocket" concept, only "closer to goal or not".
            } else if (nearestAirPruneEnabled && NearestAirDistance.shouldPrune(world, nx, y, nz, x, y, z,
                    NEAREST_AIR_MAX_RADIUS, NEAREST_AIR_PRUNE_RADIUS)) {
                // EXPERIMENTAL third arm -- see NearestAirDistance's javadoc.
                // No open space within the prune radius (as the crow flies,
                // not maze-aware) -- skip it.
            } else if (hybridDensityField != null && (hybridDensityField.densityAt(nx, y, nz) >= HYBRID_STONE_DENSITY_THRESHOLD
                    ? NearestAirDistance.shouldPrune(world, nx, y, nz, x, y, z, NEAREST_AIR_MAX_RADIUS, NEAREST_AIR_PRUNE_RADIUS)
                    : (hybridAirPotential != null && hybridAirPotential.shouldPruneMine(MINE_PRUNE_THRESHOLD, x, y, z, nx, y, nz)))) {
                // EXPERIMENTAL fourth arm -- see the horizontalEdges overload
                // javadoc above. Density-gated: NearestAirDistance's more
                // expensive check only near real bastion wall material,
                // AirPotential's cheap O(1) check everywhere else.
            } else {
                // Mining and moving happen CONCURRENTLY (you can sprint
                // while mining), not sequentially -- the movement time
                // is max()'d with mine time (whichever is the actual
                // bottleneck), against SPRINT_SPEED specifically since
                // sprinting while mining is allowed, not WALK_SPEED.
                // Earlier this max() was simplified away to just mineCost,
                // on the assumption dist/SPRINT_SPEED (<=0.2525s) is always
                // smaller than mineTime -- true for a bare hand, false once
                // tool speed/Efficiency/Haste push mineTime toward zero:
                // insta-mining (breaking in 0 ticks) is a real, common
                // outcome in this mod's actual speedrun context (verified
                // against minecraft.wiki/w/Breaking: e.g. a Golden Pickaxe,
                // speed 12, already breaks Netherrack, hardness 0.4, in
                // exactly 1 tick with ZERO enchantments -- Efficiency I
                // alone crosses the instant-break threshold of speed >
                // 30*hardness=12). Dropping the max() let a 0-cost MINE
                // edge undercut real movement, so it's restored. A small
                // MINE_DURABILITY_TAX is added on top -- not a modeled game
                // mechanic, just a deliberate tie-breaker so an insta-mined
                // MINE doesn't cost EXACTLY the same as an equal-distance
                // SPRINT and get chosen arbitrarily by search order.
                double mineCost = BlockType.mineTime(target) * toolMultiplier;
                if (BlockType.isSolid(head)) {
                    mineCost += BlockType.mineTime(head) * toolMultiplier;
                }
                double cost = Math.max(mineCost, dist / SPRINT_SPEED) + MINE_DURABILITY_TAX;
                out.accept(StateCodec.pack(nx, y, nz, blocks, false), cost, Action.MINE);
            }
            if (BlockType.isSolid(head) && !BlockType.isUnbreakable(target)) {
                // BOAT_CRAWL is also mining -- same concurrent-mining-and-
                // moving, same insta-mine vulnerability, same fix: max()
                // against BOAT_CRAWL_SPEED (not SPRINT_SPEED -- crawling is
                // slower) plus the same small durability tax.
                double crawlMineCost = BlockType.mineTime(target) * toolMultiplier;
                double startupTax = crawling ? 0.0 : BOAT_CRAWL_TAX;
                double cost = Math.max(crawlMineCost, dist / BOAT_CRAWL_SPEED)
                    + startupTax + MINE_DURABILITY_TAX;
                out.accept(StateCodec.pack(nx, y, nz, blocks, true), cost, Action.BOAT_CRAWL);
            }
            return;
        }

        boolean targetOrHeadIsLava = BlockType.isLava(target) || BlockType.isLava(head);
        boolean targetOrHeadBlocked = targetOrHeadIsLava || BlockType.isSolid(head);

        if (BlockType.isSolid(belowTarget) && !targetOrHeadBlocked) {
            double cost = dist / SPRINT_SPEED;
            out.accept(StateCodec.pack(nx, y, nz, blocks, false), cost, Action.SPRINT);
            return;
        }

        if (!BlockType.isSolid(belowTarget) && !targetOrHeadBlocked) {
            if (blocks > 0) {
                double tax = blockTax(blocks);
                double cost = PLACE_TIME + BRIDGE_RISK_PENALTY + tax + dist / SPRINT_SPEED;
                out.accept(StateCodec.pack(nx, y, nz, blocks - 1, false), cost, Action.BRIDGE);
            }
            LandingResult landing = findLandingY(world, nx, y - 1, nz);
            if (landing.found()) {
                double fallCost = dist / SPRINT_SPEED + landing.drop() * 0.05;
                if (landing.drop() > CLUTCH_THRESHOLD) {
                    fallCost += CLUTCH_SETUP_TIME;
                }
                if (landing.passedLava()) {
                    fallCost += LAVA_DEATH_PENALTY;
                }
                out.accept(StateCodec.pack(nx, landing.landingY(), nz, blocks, false), fallCost, Action.FALL);
            }
            return;
        }

        if (targetOrHeadIsLava) {
            int maxLavaClimb = 6;
            for (int climb = 1; climb <= maxLavaClimb; climb++) {
                int ty = y + climb;
                int clearTarget = world.voxelAt(nx, ty, nz);
                int clearHead = world.voxelAt(nx, ty + 1, nz);
                if (BlockType.isLava(clearTarget) || BlockType.isLava(clearHead)) {
                    continue;
                }
                if (BlockType.isSolid(clearTarget) || BlockType.isSolid(clearHead)) {
                    break;
                }
                if (blocks > 0) {
                    double tax = blockTax(blocks);
                    double climbCost = climb * JUMP_PENALTY;
                    double cost = PLACE_TIME + BRIDGE_RISK_PENALTY + tax + dist / SPRINT_SPEED + climbCost;
                    out.accept(StateCodec.pack(nx, ty, nz, blocks - 1, false), cost, Action.BRIDGE);
                }
                break;
            }
        }
    }

    /** CLIMB (step-up) for one direction from (x,y,z). */
    public static void climbEdge(World world, int x, int y, int z, int blocks, int dx, int dz, EdgeConsumer out) {
        int nx = x + dx, nz = z + dz;
        if (nx < 0 || nx >= world.sizeX || nz < 0 || nz >= world.sizeZ) {
            return;
        }
        if (diagonalCornerBlocked(world, x, y, z, dx, dz)) {
            return;
        }
        double dist = diagDist(dx, dz);
        int targetUp = world.voxelAt(nx, y + 1, nz);
        int belowUp = world.voxelAt(nx, y, nz);
        int headClear = world.voxelAt(nx, y + 2, nz);
        if (!BlockType.isSolid(targetUp) && !BlockType.isLava(targetUp)
                && BlockType.isSolid(belowUp)
                && !BlockType.isSolid(headClear) && !BlockType.isLava(headClear)) {
            double cost = dist / SPRINT_SPEED + JUMP_PENALTY;
            out.accept(StateCodec.pack(nx, y + 1, nz, blocks, false), cost, Action.CLIMB);
        }
    }

    /** Vertical mining straight down from (x,y,z). */
    public static void mineDownEdge(World world, int x, int y, int z, int blocks, double toolMultiplier, EdgeConsumer out) {
        int belowHere = world.voxelAt(x, y - 1, z);
        if (BlockType.isSolid(belowHere) && !BlockType.isUnbreakable(belowHere)) {
            double cost = BlockType.mineTime(belowHere) * toolMultiplier + 0.2;
            out.accept(StateCodec.pack(x, y - 1, z, blocks, false), cost, Action.MINE_DOWN);
        }
    }

    /**
     * How strongly the vertical delta must dominate the horizontal delta
     * for bridgeUpEdge to consider pillaring worthwhile -- fires when, for
     * AT LEAST ONE goal point, (goalY - originY) > BRIDGE_UP_PRUNE_RATIO *
     * horizontalDistanceToThatPoint (see bridgeUpEdge's own javadoc for why
     * "any point" rather than "the nearest point" is the right check here).
     * Larger values are STRICTER (require an even more lopsided case before
     * pillaring is offered, so fewer opportunities survive); smaller values
     * are more permissive, down to 0.0 which reduces the check to just
     * "goal is above me at all".
     *
     * Default 0.0 (not 1.0): a real-terrain sweep across all 12 tested
     * regions found 1.0 was too strict and silently missing better paths.
     * On the hardest region (k1_r_neg1_0), 0.0 found a path 42% cheaper
     * (74.57 -> 43.22) using 8x fewer expansions and running 4x faster
     * (2547ms -> 595ms) than 1.0 -- confirmed via PathValidator, not a
     * fluke. The effect is NOT monotonic with the ratio, though: 0.5 was
     * measurably WORSE than both 1.0 and 0.0 on that same region (78.31),
     * so "somewhere in between" isn't a safe intermediate choice -- only
     * the fully-loose end reliably paid off in this sweep. One region
     * (k2_r_0_0) got ~1.6x slower at 0.0 with no quality gain (more
     * BRIDGE_UP candidates explored that never pay off there) -- a real
     * but bounded cost (still well under 3s), not a blowup. Mutable/tunable
     * like MINE_PRUNE_THRESHOLD and AirPotential.OLD_GROUND_CUTOFF
     * elsewhere in this file if a specific region ever needs revisiting.
     */
    public static double BRIDGE_UP_PRUNE_RATIO = 0.0;

    /**
     * Straight-up pillar move: place a block beneath yourself and rise one
     * level in place (dx=dz=0), the real-Minecraft "pillaring" technique.
     * Not in the original Python reference -- pathfind.py's BRIDGE only
     * ever fires for a flat void crossing or rising above LAVA
     * specifically, so a plain solid cliff face had NO way to ascend at
     * all (confirmed via experiments/LabFidelityCheck.java's
     * ledge_bedrock_world: genuinely unsolvable before this).
     *
     * GOAL-RELATIVE PRUNE, not toggleable on/off (the ratio is tunable,
     * but the check itself always applies): unlike flat BRIDGE (gated by
     * "is there actually a void here") or lava-rise BRIDGE (gated by "is
     * there actually lava here"), a plain in-place vertical move has no
     * such natural geometric gate -- every single expansion would offer
     * it, everywhere, multiplying branching factor for a move that's
     * almost never useful except right beneath the thing you're actually
     * trying to reach. So this only fires when the goal needs
     * substantially more vertical progress than horizontal -- "you're
     * already close to the thing, you just need height."
     *
     * originY is the y-coordinate to measure that vertical delta FROM --
     * deliberately NOT always the current y. If this call is continuing an
     * already-committed bridge-up chain, originY is the y where the chain
     * STARTED, not the current (already-partway-up) position: measuring
     * from the current y instead would erode the vertical margin by 1 on
     * every single step while the horizontal distance (fixed the whole
     * time, since bridging up never moves x/z) never shrinks to match --
     * so a chain that correctly passed the check at step 1 could get
     * pruned at step 2 or 3, getting stuck partway up a climb it could
     * actually have completed. Confirmed directly: a 4-block climb with
     * goal 3 blocks away horizontally passed at step 1 (4>3) and failed at
     * step 2 (3<=3) under the naive "always use current y" version. The
     * caller is responsible for tracking originY as auxiliary per-state
     * metadata (see WeightedAStar/BidirectionalWeightedAStar's
     * bridgeUpOriginY) -- it inherits from the predecessor's originY if
     * the edge that reached THIS state was itself BRIDGE_UP, or resets to
     * this state's own y otherwise (a fresh potential chain start).
     *
     * MULTI-TARGET NOTE: the gate below fires if ANY goal point justifies
     * pillaring, not just the nearest one -- deliberately an OR, not a
     * "pick the single closest target" check. This prune only controls
     * branching factor (a speed heuristic, not a correctness gate: being
     * too permissive costs a little search time, being too strict can
     * silently make a real route look unreachable), so it should never
     * reject a chain that's genuinely progressing toward SOME acceptable
     * target just because a different, closer-but-unhelpful target
     * dominates a "nearest point" computation.
     */
    public static void bridgeUpEdge(World world, int x, int y, int z, int blocks, double toolMultiplier,
                                     int originY, GoalPoints goal, EdgeConsumer out) {
        if (blocks <= 0) {
            return;
        }
        boolean anyTargetJustifiesPillaring = false;
        for (int i = 0; i < goal.size(); i++) {
            int dyGoal = goal.y(i) - originY;
            if (dyGoal <= 0) {
                continue; // this target isn't above where the chain started -- pillaring can't help toward it
            }
            int horizGoal = Math.abs(goal.x(i) - x) + Math.abs(goal.z(i) - z);
            if (dyGoal > BRIDGE_UP_PRUNE_RATIO * horizGoal) {
                anyTargetJustifiesPillaring = true;
                break;
            }
        }
        if (!anyTargetJustifiesPillaring) {
            return; // no goal point makes pillaring worthwhile yet
        }

        int targetY = y + 1;
        if (targetY + 1 >= world.sizeY) {
            return;
        }
        int target = world.voxelAt(x, targetY, z);
        int head = world.voxelAt(x, targetY + 1, z);
        if (BlockType.isSolid(target) || BlockType.isLava(target) || BlockType.isSolid(head) || BlockType.isLava(head)) {
            return; // nowhere clear to rise into
        }

        double tax = blockTax(blocks);
        double cost = PLACE_TIME + BRIDGE_RISK_PENALTY + tax + JUMP_PENALTY;
        out.accept(StateCodec.pack(x, targetY, z, blocks - 1, false), cost, Action.BRIDGE_UP);
    }

    // ============================================================
    // Reverse (predecessor) edge generation, for bidirectional search's
    // backward half.
    //
    // For SPRINT/MINE/CLIMB/MINE_DOWN/BOAT_CRAWL: probe each of the 8
    // candidate source positions by calling the SAME forward function used
    // by expand() and filtering for edges landing exactly on the target --
    // this guarantees the reverse rules can never silently diverge from
    // forward semantics, since it's literally the same code.
    //
    // FALL is the one case that can't be probed this way cheaply: a FALL
    // source can be at ANY height above the landing spot within the same
    // open column, not just one fixed offset, so reverseFallSources does a
    // bounded incremental upward scan instead (mirrors findLandingY's
    // downward scan, run in reverse).
    // ============================================================

    /**
     * A synthesized backward-frontier source is valid if EITHER it's
     * genuinely resting on solid ground, OR it's a legitimate BRIDGE
     * landing spot -- a position with no solid floor in the static world
     * array (forward's BRIDGE action never mutates the world; it's a pure
     * cost-model abstraction for "you placed a block here and are standing
     * on it"), but with a clear body (feet + head, not solid, not lava) --
     * exactly the same precondition forward's own BRIDGE branch in
     * horizontalEdges uses to generate such a target in the first place.
     *
     * This matters for EVERY reverse probe, not just reverseBridgeSources:
     * once backward can chain through BRIDGE at all, a perfectly ordinary
     * SPRINT/MINE/CLIMB reverse-predecessor can legitimately BE a
     * bridge-landed position (e.g. "sprint away from the spot you just
     * finished bridging to") -- requiring a solid floor there would reject
     * it. Confirmed via direct reproduction: a real 2-BRIDGE-in-a-row path
     * (rolling_terrain_world.wbin) was silently unreachable by backward
     * search until this was generalized, because the SPRINT reverse-probe
     * of the position right after the bridge chain rejected the bridge
     * landing as its source.
     *
     * This is NOT the same risk class as the original "FALL landing has no
     * solid floor" bug this check was first added to prevent (see git
     * history / SESSION_LOG): that bug accepted genuinely-unfounded mid-air
     * positions with no legitimacy condition at all. This one is narrowly
     * scoped to exactly BRIDGE's own generation precondition, and even if a
     * given floating candidate never actually turns out to be
     * reverse-bridgeable all the way back to real ground, accepting it
     * tentatively is harmless: the FORWARD half of any reconstructed path
     * always comes from forward's own independently-validated closed
     * state, not from backward's predecessor-discovery -- backward only
     * ever contributes the segment FROM the meeting point TOWARD the goal,
     * and every edge in that segment (including this one) is validated by
     * the same horizontalEdges/climbEdge/etc forward would use. A
     * candidate source that never turns out to be real just becomes an
     * unproductive dead end in backward's OWN further expansion, not an
     * invalid accepted path.
     *
     * Package-visible (not private): BidirectionalWeightedAStar/
     * ParallelBidirectionalWeightedAStar also call this directly when
     * seeding backward search from each goal point, for the exact same
     * reason -- a goal point is only a legitimate seed if it's someplace
     * an agent could actually be standing.
     */
    static boolean isValidRestingSource(World world, int x, int y, int z, boolean crawling) {
        if (crawling) {
            return true;
        }
        if (BlockType.isSolid(world.voxelAt(x, y - 1, z))) {
            return true;
        }
        return !BlockType.isSolid(world.voxelAt(x, y, z)) && !BlockType.isSolid(world.voxelAt(x, y + 1, z))
                && !BlockType.isLava(world.voxelAt(x, y, z)) && !BlockType.isLava(world.voxelAt(x, y + 1, z));
    }

    private static void filterMatch(World world, long toState, double cost, Action action, Action expected,
                                     int tx, int ty, int tz, int sourceX, int sourceY, int sourceZ,
                                     boolean sourceCrawling, EdgeConsumer out) {
        if (action != expected) {
            return;
        }
        if (StateCodec.unpackX(toState) != tx || StateCodec.unpackY(toState) != ty || StateCodec.unpackZ(toState) != tz) {
            return;
        }
        if (!isValidRestingSource(world, sourceX, sourceY, sourceZ, sourceCrawling)) {
            return;
        }
        out.accept(StateCodec.pack(sourceX, sourceY, sourceZ, 0, sourceCrawling), cost, action);
    }

    /**
     * Sanity cap on reverse-BRIDGE chain length -- NOT a state-space bound
     * (see reverseBridgeSources's javadoc: backward doesn't fork state by
     * bridge count anymore, so there's no combinatorial blowup risk this
     * needs to guard against). This just stops a pathological, effectively
     * infeasible chain from being probed forever; any real search is also
     * bounded by blocksAvailable regardless.
     */
    public static int MAX_REVERSE_BRIDGE_DEPTH = 32;

    /**
     * Predecessors of a non-crawling target (tx,ty,tz): SPRINT, MINE, CLIMB,
     * MINE_DOWN, FALL, BRIDGE. (BOAT_CRAWL never lands with crawling=false,
     * so it's never a predecessor here -- see reverseCrawlSources.)
     *
     * Every one of SPRINT/MINE/CLIMB/FALL/MINE_DOWN/BRIDGE/BRIDGE_UP is
     * ELIGIBLE AND COSTS THE SAME regardless of whether the source was
     * already crawling -- only BOAT_CRAWL's own cost formula reads the
     * source's crawling flag (the startup-tax waiver). So every one of
     * these reverse probes tries BOTH srcCrawling=false and true, not just
     * false: forward search can, at any moment, "stand up" out of a crawl
     * via any of these actions for free (none of their forward branches in
     * horizontalEdges/climbEdge/mineDownEdge/bridgeUpEdge gate on the
     * crawling input at all), so a candidate source that was crawling a
     * moment ago is exactly as physically valid as one that wasn't.
     * CONFIRMED MISSING, not theoretical: mine_vs_crawl_long_world.wbin (see
     * experiments/export_lab_worlds.py) -- an 8-block STONE wall where
     * BOAT_CRAWL is the true cheaper crossing (16.43s vs MINE's 21s+) --
     * showed forward correctly finding the 16.43s BOAT_CRAWL route while
     * BidirectionalWeightedAStar/ParallelBidirectionalWeightedAStar (same
     * epsilon=1.0, same world) settled for a 20.22s MINE-based route
     * instead: backward could never reverse-discover "crawl through, then
     * SPRINT away" because the SPRINT reverse-probe only ever tried a
     * non-crawling source, so it could never credit the cheaper route in
     * the meeting-point comparison before forward's own (correct but
     * slower-to-arrive) direct-hit exploration got there.
     *
     * airPotential (nullable, null = no prune) is threaded through to the
     * MINE reverse-probe ONLY, using the exact same forward horizontalEdges
     * call forward search itself would make from that candidate source --
     * so a source this probe finds is guaranteed to be one forward's own
     * expand() would also have offered, keeping backward's MINE candidates
     * from silently diverging from forward's pruned set (see
     * BidirectionalWeightedAStar's class javadoc). Manhattan pruning is
     * deliberately NOT threaded through here -- it measured as a much
     * smaller win than AirPotential and isn't worth the extra parameter.
     * SPRINT is unaffected either way: horizontalEdges only ever applies
     * airPotential's prune to MINE candidates.
     *
     * bridgesUsedSoFarAtTarget is used ONLY to compute reverseBridgeSources'/
     * reverseBridgeUpSource's worst-case cost estimate -- see their javadocs.
     * It's NOT part of any packed state here; every predecessor this method
     * emits is a plain (x,y,z,crawling) state, same as always. The caller
     * (BidirectionalWeightedAStar) is responsible for tracking bridge count
     * as auxiliary per-state metadata, not identity.
     *
     * goal is needed for reverseBridgeUpSource, which reuses bridgeUpEdge's
     * own goal-relative prune (see its javadoc) so backward's BRIDGE_UP
     * candidates can never diverge from what forward would actually offer.
     */
    private static final boolean[] SRC_CRAWLING_VALUES = {false, true};

    public static void reverseNonCrawlEdges(World world, int tx, int ty, int tz, double toolMultiplier,
                                             AirPotential airPotential, int bridgesUsedSoFarAtTarget,
                                             GoalPoints goal, EdgeConsumer out) {
        for (int d = 0; d < 8; d++) {
            int dx = DIR_DX[d], dz = DIR_DZ[d];
            int sx = tx - dx, sz = tz - dz;
            if (sx < 0 || sx >= world.sizeX || sz < 0 || sz >= world.sizeZ) {
                continue;
            }

            // SPRINT/MINE: source at the same y as the target. Both source-crawling states tried -- see class comment above.
            for (boolean srcCrawling : SRC_CRAWLING_VALUES) {
                horizontalEdges(world, sx, ty, sz, 0, srcCrawling, dx, dz, toolMultiplier,
                        (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.SPRINT, tx, ty, tz, sx, ty, sz, srcCrawling, out));
                horizontalEdges(world, sx, ty, sz, 0, srcCrawling, dx, dz, toolMultiplier, airPotential,
                        (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.MINE, tx, ty, tz, sx, ty, sz, srcCrawling, out));
            }

            // CLIMB: source one level below the target. Both source-crawling states tried -- see class comment above.
            int climbSy = ty - 1;
            if (climbSy >= 0) {
                for (boolean srcCrawling : SRC_CRAWLING_VALUES) {
                    climbEdge(world, sx, climbSy, sz, 0, dx, dz,
                            (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.CLIMB, tx, ty, tz, sx, climbSy, sz, srcCrawling, out));
                }
            }

            // FALL, unlike BRIDGE, can only ever land on genuinely solid
            // ground -- reverseFallSources' scan assumes the target already
            // rests on real floor and never itself re-checks that. Since
            // isValidRestingSource now also accepts non-solid-floored
            // BRIDGE-plausible targets (see its javadoc), that assumption
            // can be false here, and treating a bridge-landed position as a
            // legitimate FALL destination produces a physically-impossible
            // landing (falling always lands on something solid; a placed
            // bridge block is not "something you fell onto"). Confirmed via
            // PathValidator: without this guard, backward could reverse-
            // discover a "FALL" onto a bridge-plausible-but-not-solid
            // target, exactly the "FALL landing has no solid floor" failure
            // class this file has hit before.
            if (BlockType.isSolid(world.voxelAt(tx, ty - 1, tz))) {
                reverseFallSources(world, tx, ty, tz, dx, dz, sx, sz, out);
            }
        }

        reverseBridgeSources(world, tx, ty, tz, toolMultiplier, bridgesUsedSoFarAtTarget, out);
        reverseBridgeUpSource(world, tx, ty, tz, toolMultiplier, bridgesUsedSoFarAtTarget, goal, out);

        // MINE_DOWN: exactly one candidate source, directly above -- always
        // a valid resting source since (tx,ty,tz) being solid+breakable
        // (required for MINE_DOWN to apply at all) is exactly what
        // isValidRestingSource would check here. Both source-crawling
        // states tried -- see class comment above.
        int mdSy = ty + 1;
        if (mdSy < world.sizeY) {
            for (boolean srcCrawling : SRC_CRAWLING_VALUES) {
                mineDownEdge(world, tx, mdSy, tz, 0, toolMultiplier,
                        (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.MINE_DOWN, tx, ty, tz, tx, mdSy, tz, srcCrawling, out));
            }
        }
    }

    /**
     * Predecessors of a crawling target (tx,ty,tz): BOAT_CRAWL only (it's
     * the only action that ever produces crawling=true). The source's own
     * crawling flag matters for cost (startup tax waived if already
     * crawling), so both possibilities are probed. Crawling targets never
     * chain through BRIDGE (forward's BRIDGE always lands non-crawling),
     * so there's no bridge-count parameter here.
     */
    public static void reverseCrawlEdges(World world, int tx, int ty, int tz, double toolMultiplier, EdgeConsumer out) {
        for (int d = 0; d < 8; d++) {
            int dx = DIR_DX[d], dz = DIR_DZ[d];
            int sx = tx - dx, sz = tz - dz;
            if (sx < 0 || sx >= world.sizeX || sz < 0 || sz >= world.sizeZ) {
                continue;
            }
            for (boolean srcCrawling : new boolean[]{false, true}) {
                horizontalEdges(world, sx, ty, sz, 0, srcCrawling, dx, dz, toolMultiplier,
                        (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.BOAT_CRAWL, tx, ty, tz, sx, ty, sz, srcCrawling, out));
            }
        }
    }

    /**
     * Reverse of the flat-crossing BRIDGE branch in horizontalEdges (NOT
     * the rise-above-lava variant -- that lands at a different y than the
     * source, which this single-y probe doesn't scan for; a real but rare
     * missed predecessor, same class of bounded incompleteness as
     * MAX_REVERSE_FALL_SCAN, never a correctness problem). Structurally
     * like the SPRINT/MINE probe (one candidate source per direction, same
     * y as the target) since BRIDGE moves feet-level like SPRINT.
     *
     * BRIDGE was excluded from backward entirely until this: its cost
     * depends on blockTax(blocksRemaining), the ABSOLUTE resource count at
     * the moment of placement, and backward -- working outward from the
     * goal -- has no way to know what forward's absolute remaining will be
     * at the point they meet.
     *
     * The FIRST fix for this tracked blocksNeeded as part of backward's
     * STATE IDENTITY (a whole separate copy of the reachable graph per
     * possible bridge count). That was correct but a disaster on real
     * terrain: caves/overhangs mean a huge fraction of the map is
     * void-adjacent, so forking state by bridge count made backward
     * duplicate large chunks of its own reachable graph per depth level --
     * confirmed on the hardest tested real region (k1_r_neg1_0): even
     * depth=1 nearly doubled expansions, and depth>=4 never converged
     * within a 3M-expansion budget at all.
     *
     * This version tracks bridge count as a plain SCALAR alongside g (see
     * BidirectionalWeightedAStar's bwdBridgesUsed array), exactly like
     * cameFromId/actionUsed already are -- no forking, no extra states,
     * backward's state space stays (x,y,z,crawling) same as always. The
     * cost this method computes is a WORST-CASE estimate: it assumes THIS
     * specific bridge (the bridgesUsedSoFarAtTarget-th one encountered
     * walking outward from goal) is being placed on forward's LAST
     * available block, i.e. blockTax(1); a second bridge further out
     * assumes blockTax(2); and so on -- "assume you're always down to your
     * last few blocks" rather than "assume you have plenty" (the earlier
     * version's admissible-but-wrong-direction choice). This deliberately
     * OVERESTIMATES cost, same direction (though far less extreme) as the
     * original full-exclusion behavior, so it's not perfectly sound for
     * the meeting/termination criterion -- but it's a small, BOUNDED
     * pessimism (unlike full exclusion's effectively-infinite one), and it
     * costs nothing extra in state space. mu/the final reported cost is
     * corrected exactly at reconstruction time regardless (see
     * BidirectionalWeightedAStar.reconstructMeeting), once the true
     * meeting-point blocksRemaining is known.
     */
    public static void reverseBridgeSources(World world, int tx, int ty, int tz, double toolMultiplier,
                                             int bridgesUsedSoFarAtTarget, EdgeConsumer out) {
        int assumedRemaining = bridgesUsedSoFarAtTarget + 1;
        if (assumedRemaining > MAX_REVERSE_BRIDGE_DEPTH) {
            return;
        }
        for (int d = 0; d < 8; d++) {
            int dx = DIR_DX[d], dz = DIR_DZ[d];
            int sx = tx - dx, sz = tz - dz;
            if (sx < 0 || sx >= world.sizeX || sz < 0 || sz >= world.sizeZ) {
                continue;
            }
            // Both source-crawling states -- see reverseNonCrawlEdges' class comment.
            for (boolean srcCrawling : SRC_CRAWLING_VALUES) {
                if (!isValidRestingSource(world, sx, ty, sz, srcCrawling)) {
                    continue;
                }
                horizontalEdges(world, sx, ty, sz, assumedRemaining, srcCrawling, dx, dz, toolMultiplier,
                        (toState, cost, action) -> {
                            if (action != Action.BRIDGE) {
                                return;
                            }
                            if (StateCodec.unpackX(toState) != tx || StateCodec.unpackY(toState) != ty || StateCodec.unpackZ(toState) != tz) {
                                return;
                            }
                            out.accept(StateCodec.pack(sx, ty, sz, 0, srcCrawling), cost, Action.BRIDGE);
                        });
            }
        }
    }

    /**
     * Reverse of bridgeUpEdge: the only candidate source is directly below
     * the target (dx=dz=0, single-y probe -- BRIDGE_UP always moves
     * feet-level-plus-one like CLIMB, never sideways). Calls bridgeUpEdge
     * itself from that candidate source and filters for a match, exactly
     * the same reuse-forward-logic pattern as reverseBridgeSources -- this
     * means backward's BRIDGE_UP candidates automatically inherit
     * bridgeUpEdge's goal-relative prune, since it's the SAME goal
     * coordinates on both sides of the search. Cost uses the same
     * worst-case-tax model and MAX_REVERSE_BRIDGE_DEPTH cap as
     * reverseBridgeSources -- see its javadoc for the full rationale
     * (why a scalar bridge count instead of forking state, why an
     * overestimate is the acceptable direction here).
     *
     * originY here is just the CANDIDATE SOURCE's own y (sy), not a
     * remembered chain start -- unlike forward, which walks a chain
     * start-to-end and can carry its true origin forward, backward
     * discovers a chain end-to-start (closest to goal first), so at the
     * moment this candidate is generated it's not yet known whether the
     * chain extends further, and no true origin is available to use.
     * This is a deliberate, ACCEPTED asymmetry: backward may fail to
     * reverse-discover a long bridge-up chain that forward's own
     * (origin-aware) search would happily walk directly, exactly the
     * same class of "lost speedup opportunity, not a correctness risk"
     * already true of the BRIDGE resource-funding approximation and the
     * MAX_REVERSE_BRIDGE_DEPTH cap -- forward's guaranteed-complete
     * direct-hit fallback still covers it.
     */
    public static void reverseBridgeUpSource(World world, int tx, int ty, int tz, double toolMultiplier,
                                              int bridgesUsedSoFarAtTarget, GoalPoints goal, EdgeConsumer out) {
        int assumedRemaining = bridgesUsedSoFarAtTarget + 1;
        if (assumedRemaining > MAX_REVERSE_BRIDGE_DEPTH) {
            return;
        }
        int sy = ty - 1;
        if (sy < 0) {
            return;
        }
        // Both source-crawling states -- see reverseNonCrawlEdges' class comment.
        for (boolean srcCrawling : SRC_CRAWLING_VALUES) {
            if (!isValidRestingSource(world, tx, sy, tz, srcCrawling)) {
                continue;
            }
            bridgeUpEdge(world, tx, sy, tz, assumedRemaining, toolMultiplier, sy, goal,
                    (toState, cost, action) -> {
                        if (action != Action.BRIDGE_UP) {
                            return;
                        }
                        if (StateCodec.unpackX(toState) != tx || StateCodec.unpackY(toState) != ty || StateCodec.unpackZ(toState) != tz) {
                            return;
                        }
                        out.accept(StateCodec.pack(tx, sy, tz, 0, srcCrawling), cost, Action.BRIDGE_UP);
                    });
        }
    }

    /**
     * Reverse of the FALL branch in horizontalEdges. A FALL landing at
     * (tx,ty,tz) can have come from stepping off at (tx-dx,Y,tz-dz) for any
     * Y in the same open column above ty -- not just one fixed height, so
     * this scans upward instead of probing a single candidate.
     *
     * Mirrors findLandingY's downward scan run in reverse: each step up
     * corresponds to one more step findLandingY would have taken going
     * down, so the solid/lava checks and the passed-lava accumulation are
     * the same checks in the opposite order. Stops as soon as the column
     * stops being a valid open-air segment (solid floor, solid/lava target,
     * or solid/lava head -- any of which means a step-off from here would
     * either not reach this exact landing spot, or not be a legal step-off
     * point at all), or after MAX_REVERSE_FALL_SCAN blocks.
     */
    private static void reverseFallSources(World world, int tx, int ty, int tz, int dx, int dz, int sx, int sz, EdgeConsumer out) {
        double dist = diagDist(dx, dz);
        int landingCell = world.voxelAt(tx, ty, tz);
        int landingHead = world.voxelAt(tx, ty + 1, tz);
        boolean landingIsLava = BlockType.isLava(landingCell) || BlockType.isLava(landingHead);

        boolean passedLava = false;
        for (int dropAmount = 1; dropAmount <= MAX_REVERSE_FALL_SCAN; dropAmount++) {
            int y = ty + dropAmount; // candidate source height
            if (y >= world.sizeY) {
                break;
            }
            int belowTarget = world.voxelAt(tx, y - 1, tz); // == world.voxelAt(tx, y - 1, tz); this is what findLandingY would visit at this depth
            if (BlockType.isSolid(belowTarget)) {
                // The column isn't open all the way down to ty from here --
                // either this itself would land above ty (a shorter fall),
                // or (for dropAmount==1) this literally IS ty's own floor,
                // in which case y=ty+1 is a SPRINT source, not a FALL source
                // (handled separately above), so stop either way.
                break;
            }
            int target = world.voxelAt(tx, y, tz);
            int head = world.voxelAt(tx, y + 1, tz);
            if (BlockType.isSolid(target) || BlockType.isSolid(head)) {
                break; // not a legal step-off point at this height, and nothing above can fall past a solid block here either
            }
            if (BlockType.isLava(belowTarget)) {
                passedLava = true;
            }
            boolean thisPassedLava = passedLava || landingIsLava;

            // The candidate source (sx,y,sz) must itself be a legitimate
            // resting position (solid floor beneath it) before it can be
            // treated as a real backward-frontier node. Forward search never
            // needs this check -- expand() is only ever called on states
            // that were already validated by whatever action reached them --
            // but a synthesized reverse-FALL source has no such history, so
            // skipping this check would let backward search continue
            // expanding from a physically-impossible mid-air position,
            // cascading bogus edges (caught by PathValidator during
            // development: "FALL landing has no solid floor" on a whole
            // chain of stitched steps).
            if (!BlockType.isSolid(world.voxelAt(sx, y - 1, sz))) {
                continue;
            }
            // Forward's horizontalEdges gates EVERY branch (SPRINT/BRIDGE/
            // FALL alike) behind diagonalCornerBlocked before it ever
            // reaches the FALL-specific logic -- reverseFallSources never
            // replicated that gate, so backward search could synthesize a
            // FALL source that forward would have refused to generate at
            // all. Latent since this method was first written; only
            // surfaced on real terrain (k2_r_0_0) once PARKOUR's presence
            // changed backward's exploration order enough to actually reach
            // a blocked-corner case. Height-by-height continue, not break --
            // a wall can block the corner at one candidate height and not
            // another as the scan climbs.
            if (diagonalCornerBlocked(world, sx, y, sz, dx, dz)) {
                continue;
            }

            double fallCost = dist / SPRINT_SPEED + dropAmount * 0.05;
            if (dropAmount > CLUTCH_THRESHOLD) {
                fallCost += CLUTCH_SETUP_TIME;
            }
            if (thisPassedLava) {
                fallCost += LAVA_DEATH_PENALTY;
            }
            // Both source-crawling states -- see reverseNonCrawlEdges' class comment.
            for (boolean srcCrawling : SRC_CRAWLING_VALUES) {
                out.accept(StateCodec.pack(sx, y, sz, 0, srcCrawling), fallCost, Action.FALL);
            }
        }
    }

    private EdgeRules() {}
}
