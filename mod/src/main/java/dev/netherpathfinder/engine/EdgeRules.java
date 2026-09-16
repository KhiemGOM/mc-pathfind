package dev.netherpathfinder.engine;

/**
 * Per-action edge legality/cost computation, extracted from WeightedAStar so
 * forward search and the backward half of a bidirectional search share the
 * EXACT same rules -- hand-writing a separate "reverse" version of each
 * action would risk it silently diverging from forward semantics. Any
 * behavior change here must be re-verified against a full region batch
 * before being trusted.
 */
public final class EdgeRules {

    // 8 horizontal directions (cardinal + diagonal), matches DIRS_H in
    // pathfind.py. Declared first since PARKOUR_KERNEL's static
    // initializer (below) reads DIR_DX/DIR_DZ, and Java runs field
    // initializers in textual declaration order.
    public static final int[] DIR_DX = {1, -1, 0, 0, 1, 1, -1, -1};
    public static final int[] DIR_DZ = {0, 0, 1, -1, 1, -1, 1, -1};

    public static volatile double BOAT_CRAWL_TAX = EngineDefaults.boatCrawlTax();
    public static volatile double BOAT_CRAWL_SPEED = EngineDefaults.boatCrawlSpeed();
    public static volatile double SPRINT_SPEED = EngineDefaults.sprintSpeed();
    public static volatile double WALK_SPEED = EngineDefaults.walkSpeed();
    public static volatile double JUMP_PENALTY = EngineDefaults.jumpPenalty();
    public static volatile double SPEED_BRIDGE_SPEED = EngineDefaults.speedBridgeSpeed();
    public static volatile double TOWER_SPEED = EngineDefaults.towerSpeed();
    public static volatile double PLACE_TIME = EngineDefaults.placeTime();
    public static volatile double BRIDGE_RISK_PENALTY = EngineDefaults.bridgeRiskPenalty();
    public static volatile double BLOCK_TAX_BASE = EngineDefaults.blockTaxBase();
    public static volatile double BLOCK_TAX_SCARCITY_SCALE = EngineDefaults.blockTaxScarcityScale();
    public static volatile int PREFERRED_BASTION_ROUTE_BLOCKS = EngineDefaults.preferredRouteBlocks();
    public static volatile double LOW_BLOCK_PENALTY_PER_BLOCK = EngineDefaults.lowBlockPenaltyPerBlock();
    public static volatile double SOUL_SAND_SPEED = EngineDefaults.soulSandSpeed();
    public static volatile double MAGMA_HAZARD_PENALTY = EngineDefaults.magmaHazardPenalty();
    public static volatile double FIRE_HAZARD_PENALTY = EngineDefaults.fireHazardPenalty();
    public static volatile int CLUTCH_THRESHOLD = EngineDefaults.clutchThreshold();
    public static volatile double CLUTCH_SETUP_TIME = EngineDefaults.clutchSetupTime();
    public static volatile double FALL_PENALTY_PER_BLOCK = EngineDefaults.fallPenaltyPerBlock();
    public static volatile double LAVA_DEATH_PENALTY = EngineDefaults.lavaDeathPenalty();
    public static volatile double MINE_PRUNE_THRESHOLD = EngineDefaults.minePruneThreshold();
    // Not a modeled game mechanic -- a deliberately small artificial
    // tie-breaker against MINE/BOAT_CRAWL so insta-mine (see the max() in
    // horizontalEdges' MINE/BOAT_CRAWL branches) doesn't make digging tie
    // exactly with an equal-time SPRINT/BOAT_CRAWL alternative. Must stay
    // >= JUMP_PENALTY: CLIMB and MINE compete for the exact same target
    // cell on a single-block bump, and a tax smaller than JUMP_PENALTY let
    // an insta-mine systematically undercut CLIMB there. Still small
    // enough to never meaningfully discourage a real dig CLIMB can't cross
    // at all.
    public static volatile double MINE_DURABILITY_TAX = EngineDefaults.mineDurabilityTax();
    // Per-tier overrides on top of the baseline above -- a flat tax can't
    // tell the difference between a Diamond pickaxe (speed 8, 1561
    // durability -- effectively unlimited for one run) and a Golden one
    // (speed 12, the FASTEST tier in the game, but only 32 durability --
    // it can shatter after mining barely 32 blocks). A fast-but-fragile
    // tool makes mineCost collapse toward zero for common nether blocks
    // just like Diamond does, but the real-world durability risk is
    // wildly different, so the tax needs to scale with that risk, not
    // just be "big enough to beat JUMP_PENALTY once." Baseline
    // (MINE_DURABILITY_TAX) still covers DIAMOND/NETHERITE/WOODEN/NONE --
    // see durabilityTaxFor's javadoc for why those default to it rather
    // than getting their own entries. Real vanilla pickaxe durabilities
    // for context: Wooden 59, Golden 32, Stone 131, Iron 250, Diamond
    // 1561, Netherite 2031.
    public static volatile double MINE_DURABILITY_TAX_IRON = EngineDefaults.mineDurabilityTaxIron();
    public static volatile double MINE_DURABILITY_TAX_STONE = EngineDefaults.mineDurabilityTaxStone();
    public static volatile double MINE_DURABILITY_TAX_GOLDEN = EngineDefaults.mineDurabilityTaxGolden();

    // --- PARKOUR (long jump; covers "boat-jump and equivalent long-jump
    // techniques" as one action -- see parkourEdges' javadoc for the split
    // into a cheap short-jump tier and that formula as the fallback tier). ---
    public static final double MIN_PARKOUR_DIST = 2.0;
    public static final double MAX_PARKOUR_DIST = 7.0;
    public static volatile double PARKOUR_SPEED = EngineDefaults.parkourSpeed();
    public static volatile double PARKOUR_RISK_BASE = EngineDefaults.parkourRiskBase();
    public static volatile double PARKOUR_RISK_PER_BLOCK = EngineDefaults.parkourRiskPerBlock();
    // Cheap-tier horizontal-distance ceilings, one per landing dy -- a jump
    // landing level or lower reaches farther than one that also has to gain
    // height, matching real jump-arc physics. Inside these, PARKOUR costs
    // about as much as a single CLIMB-style hop; outside, it falls back to
    // the full risk-scaled formula below.
    public static final double PARKOUR_CHEAP_MAX_DIST_FLAT = Math.sqrt(13.0);
    public static final double PARKOUR_CHEAP_MAX_DIST_UP = Math.sqrt(5.0);
    public static final double PARKOUR_CHEAP_MAX_DIST_DOWN = Math.sqrt(20.0);
    // Cardinal-only (N/S/E/W, the same DIR_DX/DIR_DZ[0..3] axes SPRINT/
    // BRIDGE already use) instead of angle-sampled directions -- diagonal
    // PARKOUR candidates were the majority of an earlier kernel for barely
    // any extra reachability, since SPRINT/BRIDGE/CLIMB already cover
    // diagonal movement cheaply right up to the gap's edge; a real gap
    // almost always has a cardinal-facing crossing point too. Exact
    // integer distances (no angle/cos/sin/round approximation, no dedup
    // needed -- a cardinal direction at a given integer distance is
    // already a single, unique offset) further shrinks and simplifies the
    // kernel: 3,4 short-tier distances (the actual useful cheap-tier
    // range) and a single mid-range long-tier distance (6) for the rare
    // boat-jump fallback, whose entries are also the priciest ones to
    // evaluate (trajectory sampling scales with distance), so sampling it
    // sparsely cuts the most total work per node.
    private static final int[] PARKOUR_SHORT_DISTS = {3, 4};
    private static final int[] PARKOUR_LONG_DISTS = {6};
    // Not a modeled game mechanic -- same tie-breaker role as
    // MINE_DURABILITY_TAX, just for the cheap PARKOUR tier.
    public static volatile double PARKOUR_CHEAP_RISK_TAX = EngineDefaults.parkourCheapRiskTax();

    private static final class ParkourOffset {
        final int dx, dy, dz;
        final double dist;

        ParkourOffset(int dx, int dy, int dz, double dist) {
            this.dx = dx;
            this.dy = dy;
            this.dz = dz;
            this.dist = dist;
        }
    }

    private static final ParkourOffset[] PARKOUR_KERNEL = buildParkourKernel();

    /**
     * Sparse sample of (dx,dy,dz,dist) offsets approximating valid PARKOUR
     * jumps, rather than every integer lattice point in the outer shell
     * (~1000 cells at MAX_PARKOUR_DIST=7 -- intractable per-expansion), and
     * rather than a single uniform angled kernel across the whole range --
     * see the cardinal-only rationale in the constants above. Built once at
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
     * True iff there's a genuine gap to jump, not just an ordinary
     * single-block dip or step. Checking only "does any of the 8 adjacent
     * columns lack solid footing?" fires just as readily on a single
     * missing block or a one-block stair-step as on a real chasm -- both
     * are common on natural terrain, and both are already crossed by
     * FALL/CLIMB for a fraction of PARKOUR's cost, so running the full
     * kernel scan for them is pure wasted work. Requires the SECOND cell
     * out in the same direction to also lack solid footing (a real
     * 2+-block-wide gap), not just the first -- this doesn't change what
     * PARKOUR can ultimately produce (the trajectory/landing checks below
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
     * Anything past that threshold falls back to a risk-scaled long-jump
     * formula (PARKOUR_RISK_BASE + PARKOUR_RISK_PER_BLOCK*dist +
     * dist/PARKOUR_SPEED), covering "boat-jump and equivalent long-jump
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
     *
     * No reverse probe here (unlike the other actions below, which support
     * a bidirectional search's backward half) -- this engine's forward
     * solver never needed one for PARKOUR specifically.
     */
    public static void parkourEdges(World world, int x, int y, int z, int blocks, EdgeConsumer out) {
        if (!hasJumpableGap(world, x, y, z)) {
            return;
        }
        for (ParkourOffset offset : PARKOUR_KERNEL) {
            int nx = x + offset.dx;
            int ny = y + offset.dy;
            int nz = z + offset.dz;
            if (nx < 0 || nx >= world.sizeX || ny < 0 || ny >= world.sizeY || nz < 0 || nz >= world.sizeZ) {
                continue;
            }

            int nSamples = Math.max(2, (int) Math.round(offset.dist));
            boolean trajectoryBlocked = false;
            boolean crossesGap = false;
            for (int s = 1; s < nSamples; s++) {
                double t = (double) s / nSamples;
                int sx = (int) Math.round(x + offset.dx * t);
                int sz = (int) Math.round(z + offset.dz * t);
                int sy = (int) Math.round(y + offset.dy * t);
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
                continue;
            }

            int landTarget = world.voxelAt(nx, ny, nz);
            int landHead = world.voxelAt(nx, ny + 1, nz);
            int landSupport = world.voxelAt(nx, ny - 1, nz);
            if (!BlockType.isSolid(landSupport)) {
                continue;
            }
            if (BlockType.isSolid(landTarget) || BlockType.isLava(landTarget)) {
                continue;
            }
            if (BlockType.isSolid(landHead) || BlockType.isLava(landHead)) {
                continue;
            }

            double dist = offset.dist;
            int dy = offset.dy;
            boolean cheap = (dy == 0 && dist < PARKOUR_CHEAP_MAX_DIST_FLAT)
                || (dy == 1 && dist < PARKOUR_CHEAP_MAX_DIST_UP)
                || (dy == -1 && dist < PARKOUR_CHEAP_MAX_DIST_DOWN);
            double cost = cheap
                ? dist / SPRINT_SPEED + JUMP_PENALTY + PARKOUR_CHEAP_RISK_TAX
                : PARKOUR_RISK_BASE + PARKOUR_RISK_PER_BLOCK * dist + dist / PARKOUR_SPEED;
            out.accept(StateCodec.pack(nx, ny, nz, blocks, false), cost, Action.PARKOUR);
        }
    }

    /**
     * True iff mining from (x,y,z) to (nx,y,nz) strictly decreases Manhattan
     * distance to the goal. Used by the alternate (non-AirPotential) MINE
     * prune: "don't mine backward/sideways relative to the goal". Note this
     * is a STRICT decrease (not <=) -- a MINE that leaves
     * Manhattan distance unchanged (e.g. purely lateral relative to the
     * goal in one axis while irrelevant in another) is treated the same as
     * one that increases it: not making progress, so prune it.
     */
    public static boolean mineMovesCloserToGoal(int x, int y, int z, int nx, int nz, int goalX, int goalY, int goalZ) {
        return mineMovesCloserToGoal(x, y, z, nx, nz,
            GoalPoints.point(goalX, goalY, goalZ));
    }

    public static boolean mineMovesCloserToGoal(int x, int y, int z, int nx, int nz,
                                                 GoalPoints goal) {
        int distBefore = Integer.MAX_VALUE;
        int distAfter = Integer.MAX_VALUE;
        for (int i = 0; i < goal.size(); i++) {
            int goalX = goal.x(i);
            int goalY = goal.y(i);
            int goalZ = goal.z(i);
            distBefore = Math.min(distBefore,
                Math.abs(x - goalX) + Math.abs(y - goalY) + Math.abs(z - goalZ));
            distAfter = Math.min(distAfter,
                Math.abs(nx - goalX) + Math.abs(y - goalY) + Math.abs(nz - goalZ));
        }
        return distAfter < distBefore;
    }

    // Bound on how far reverse-FALL scans upward from a landing spot looking
    // for valid step-off heights. Real nether rooms/caverns are rarely much
    // taller than this; bidirectional search falls back to forward-only if
    // backward search can't connect, so an occasional missed tall-drop
    // predecessor costs a speedup opportunity, never correctness.
    public static volatile int MAX_REVERSE_FALL_SCAN = EngineDefaults.maxReverseFallScan();

    @FunctionalInterface
    public interface EdgeConsumer {
        void accept(long toState, double cost, Action action);
    }

    public static final class LandingResult {
        private final boolean found;
        private final int landingY;
        private final int drop;
        private final boolean passedLava;

        public LandingResult(boolean found, int landingY, int drop, boolean passedLava) {
            this.found = found;
            this.landingY = landingY;
            this.drop = drop;
            this.passedLava = passedLava;
        }

        public boolean found() { return found; }
        public int landingY() { return landingY; }
        public int drop() { return drop; }
        public boolean passedLava() { return passedLava; }
    }

    public static double blockTax(int blocksRemaining) {
        if (blocksRemaining <= 0) {
            return Double.POSITIVE_INFINITY;
        }
        double scarcity = BLOCK_TAX_SCARCITY_SCALE / blocksRemaining;
        int routeShortfall = Math.max(0, PREFERRED_BASTION_ROUTE_BLOCKS - blocksRemaining);
        return BLOCK_TAX_BASE * (1.0 + scarcity)
            + routeShortfall * LOW_BLOCK_PENALTY_PER_BLOCK;
    }

    public static double blockTax(int blocksRemaining, int reservedBlocks) {
        return blockTax(blocksRemaining - Math.max(0, reservedBlocks));
    }

    public static double speedBridgeTime(double distance) {
        return Math.max(0.0d, distance) / SPEED_BRIDGE_SPEED;
    }

    public static double towerTime(int verticalBlocks) {
        return Math.max(0, verticalBlocks) / TOWER_SPEED;
    }

    public static double terrainTraversalCost(World world, int x, int y, int z,
                                              double distance, double normalSpeed) {
        int floor = world.voxelAt(x, y - 1, z);
        int feet = world.voxelAt(x, y, z);
        int head = world.voxelAt(x, y + 1, z);
        double speed = floor == BlockType.SOUL_SAND ? SOUL_SAND_SPEED : normalSpeed;
        return distance / speed + terrainHazardPenalty(floor, feet, head);
    }

    public static double terrainHazardPenalty(int floor, int feet, int head) {
        double penalty = 0.0d;
        if (floor == BlockType.MAGMA) penalty += MAGMA_HAZARD_PENALTY;
        if (feet == BlockType.FIRE || head == BlockType.FIRE) penalty += FIRE_HAZARD_PENALTY;
        return penalty;
    }

    /**
     * Tier-scaled durability tax for MINE/BOAT_CRAWL -- see
     * MINE_DURABILITY_TAX_IRON/STONE/GOLDEN's javadoc for why a flat tax
     * can't tell a fast-but-fragile tool from a durable one. WOODEN,
     * DIAMOND, NETHERITE, NONE (and a null tier, from
     * MineCostModel.scaled's back-compat path) fall back to the plain
     * baseline: DIAMOND/NETHERITE because their durability genuinely is a
     * non-concern for one run; WOODEN despite its low real durability (59)
     * because it's rarely, if ever, the tool actually in hand by the time
     * a search needs to mine anything nontrivial; NONE (bare hands)
     * because mineTime is already so slow there that an extra tax wouldn't
     * change any real decision.
     */
    private static double durabilityTaxFor(MineCostModel.PickaxeTier tier) {
        if (tier == null) {
            return MINE_DURABILITY_TAX;
        }
        switch (tier) {
            case GOLDEN: return MINE_DURABILITY_TAX_GOLDEN;
            case STONE: return MINE_DURABILITY_TAX_STONE;
            case IRON: return MINE_DURABILITY_TAX_IRON;
            default: return MINE_DURABILITY_TAX;
        }
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
                                        int dx, int dz, MineCostModel tools, EdgeConsumer out) {
        horizontalEdges(world, x, y, z, blocks, crawling, dx, dz, tools, null, out);
    }

    /**
     * Same as the 9-arg horizontalEdges, but with an optional AirPotential
     * (null = no adjustment, exactly the original behavior) applied to
     * discount MINE's cost when the dig target is both chunk-BFS-distant
     * (probably a different, unopened air pocket) and Euclidean-close (a
     * thin wall rather than a long tunnel). See AirPotential's javadoc for
     * the full rationale and formula. Only MINE is adjusted -- BOAT_CRAWL,
     * SPRINT, etc. are untouched; this is deliberately scoped narrow since
     * MINE is the single biggest source of wasted candidate generation on
     * real terrain, with only a small fraction ever useful.
     */
    public static void horizontalEdges(World world, int x, int y, int z, int blocks, boolean crawling,
                                        int dx, int dz, MineCostModel tools, AirPotential airPotential, EdgeConsumer out) {
        horizontalEdges(world, x, y, z, blocks, crawling, dx, dz, tools, airPotential, false, 0, 0, 0, out);
    }

    /**
     * Same as the AirPotential overload, but adds an ALTERNATE, independent
     * MINE prune: manhattanPruneEnabled=true skips generating a MINE
     * candidate if it doesn't strictly decrease Manhattan distance to
     * (goalX, goalY, goalZ) -- i.e. "don't mine backward/sideways relative
     * to the goal". This is a much cheaper check than AirPotential (a
     * handful of subtractions, no BFS field lookup) and a much blunter one
     * -- it has no notion of "new pocket vs old ground", it only asks
     * "did this get me closer to the goal in raw Manhattan terms". Tried
     * as an alternative to AirPotential, not a replacement -- both exist
     * side by side so they can be A/B'd. If both airPotential and
     * manhattanPruneEnabled are supplied, a MINE candidate must survive
     * BOTH checks (this method applies airPotential's prune first, then
     * the Manhattan check, short-circuiting on whichever fails first).
     */
    public static void horizontalEdges(World world, int x, int y, int z, int blocks, boolean crawling,
                                        int dx, int dz, MineCostModel tools, AirPotential airPotential,
                                        boolean manhattanPruneEnabled, int goalX, int goalY, int goalZ, EdgeConsumer out) {
        horizontalEdges(world, x, y, z, blocks, crawling, dx, dz, tools,
            airPotential, manhattanPruneEnabled, GoalPoints.point(goalX, goalY, goalZ), out);
    }

    public static void horizontalEdges(World world, int x, int y, int z, int blocks, boolean crawling,
                                        int dx, int dz, MineCostModel tools, AirPotential airPotential,
                                        boolean manhattanPruneEnabled, GoalPoints goal, EdgeConsumer out) {
        int nx = x + dx, nz = z + dz;
        if (nx < 0 || nx >= world.sizeX || nz < 0 || nz >= world.sizeZ) {
            return;
        }
        if (diagonalCornerBlocked(world, x, y, z, dx, dz)) {
            return;
        }
        double dist = Math.sqrt((double) (dx * dx + dz * dz));

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
                // completeness tradeoff this accepts. A soft cost penalty on
                // surviving candidates was tried alongside this and dropped:
                // once pruning exists, penalizing candidates that already
                // passed the potential bar added no measurable benefit, only
                // extra per-call overhead, so MINE cost is left untouched
                // (baseMineCost) for anything that survives the prune.
            } else if (manhattanPruneEnabled && !mineMovesCloserToGoal(x, y, z, nx, nz, goal)) {
                // alternate, independent prune: this dig doesn't strictly
                // decrease Manhattan distance to the goal -- skip it. Much
                // cheaper than AirPotential (no BFS lookup) but blunter: it
                // has no "new pocket" concept, only "closer to goal or not".
            } else {
                // Mining and moving happen concurrently (you can sprint
                // while mining), not sequentially -- the movement time is
                // max()'d with mine time (whichever is the real bottleneck),
                // against SPRINT_SPEED since sprinting while mining is
                // allowed. Dropping this max() (using just mineCost) relies
                // on dist/SPRINT_SPEED (<=0.2525s) always being smaller than
                // mineTime -- true for a bare hand, false once tool speed/
                // Efficiency/Haste push mineTime toward zero: insta-mining
                // (breaking in 0 ticks) is a real, common outcome with
                // MineCostModel's own pickaxe tiers (verified against
                // minecraft.wiki/w/Breaking: a Golden Pickaxe alone, speed
                // 12, already breaks Netherrack, hardness 0.4, in exactly 1
                // tick with zero enchantments; Efficiency I alone crosses
                // the instant-break threshold of speed > 30*hardness=12).
                // Without the max(), a 0-cost MINE edge would undercut real
                // movement. A durability tax (see durabilityTaxFor -- tier-
                // scaled, not flat) is added on top -- not purely a
                // modeled game mechanic, but not purely a tie-breaker
                // either: a fast-but-fragile tool (Golden) makes mineCost
                // collapse toward zero same as a durable one (Diamond)
                // would, so the tax has to do double duty as both "don't
                // tie exactly with an equal-distance SPRINT" and "reflect
                // that this tier's tool might not survive much tunneling."
                // The hazard penalty (magma/fire) is a separate risk cost,
                // not movement time, and is added on top of both.
                double mineCost = tools.mineTime(target);
                if (BlockType.isSolid(head)) {
                    mineCost += tools.mineTime(head);
                }
                double cost = Math.max(mineCost, dist / SPRINT_SPEED) + durabilityTaxFor(tools.tier())
                    + terrainHazardPenalty(belowTarget, target, head);
                out.accept(StateCodec.pack(nx, y, nz, blocks, false), cost, Action.MINE);
            }
            if (BlockType.isSolid(head) && !BlockType.isUnbreakable(target)) {
                // BOAT_CRAWL is also mining -- same concurrent-mining-and-
                // moving, same insta-mine vulnerability, same fix: max()
                // against BOAT_CRAWL_SPEED (not SPRINT_SPEED -- crawling is
                // slower) plus the same tier-scaled durability tax.
                double crawlMineCost = tools.mineTime(target);
                double startupTax = crawling ? 0.0 : BOAT_CRAWL_TAX;
                double cost = Math.max(crawlMineCost, dist / BOAT_CRAWL_SPEED) + startupTax
                    + durabilityTaxFor(tools.tier()) + terrainHazardPenalty(belowTarget, target, head);
                out.accept(StateCodec.pack(nx, y, nz, blocks, true), cost, Action.BOAT_CRAWL);
            }
            return;
        }

        boolean targetOrHeadIsLava = BlockType.isLava(target) || BlockType.isLava(head);
        boolean targetOrHeadBlocked = targetOrHeadIsLava || BlockType.isSolid(head);

        if (BlockType.isSolid(belowTarget) && !targetOrHeadBlocked) {
            double cost = terrainTraversalCost(world, nx, y, nz, dist, SPRINT_SPEED);
            out.accept(StateCodec.pack(nx, y, nz, blocks, false), cost, Action.SPRINT);
            return;
        }

        if (!BlockType.isSolid(belowTarget) && !targetOrHeadBlocked) {
            if (blocks > 0) {
                double tax = blockTax(blocks, goal.minimumBlockReserve());
                double cost = PLACE_TIME + BRIDGE_RISK_PENALTY + tax + speedBridgeTime(dist);
                out.accept(StateCodec.pack(nx, y, nz, blocks - 1, false), cost, Action.BRIDGE);
            }
            LandingResult landing = findLandingY(world, nx, y - 1, nz);
            if (landing.found()) {
                double fallCost = terrainTraversalCost(world, nx, landing.landingY(), nz,
                    dist, SPRINT_SPEED) + landing.drop() * FALL_PENALTY_PER_BLOCK;
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
                    double tax = blockTax(blocks, goal.minimumBlockReserve());
                    double cost = PLACE_TIME + BRIDGE_RISK_PENALTY + tax
                        + speedBridgeTime(dist) + towerTime(climb);
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
        // A diagonal step-up sweeps the body through BOTH heights: it starts
        // at (y, y+1) and jumps through (y+1, y+2). Checking only the start
        // height let a route cut through a corner that was solid at the
        // raised level (seen in-game as the path piercing a wall corner
        // while climbing), so both levels must be clear of the two-solid-
        // flank squeeze.
        if (diagonalCornerBlocked(world, x, y, z, dx, dz)
                || diagonalCornerBlocked(world, x, y + 1, z, dx, dz)) {
            return;
        }
        double dist = Math.sqrt((double) (dx * dx + dz * dz));
        int targetUp = world.voxelAt(nx, y + 1, nz);
        int belowUp = world.voxelAt(nx, y, nz);
        int headClear = world.voxelAt(nx, y + 2, nz);
        if (!BlockType.isSolid(targetUp) && !BlockType.isLava(targetUp)
                && BlockType.isSolid(belowUp)
                && !BlockType.isSolid(headClear) && !BlockType.isLava(headClear)) {
            double cost = terrainTraversalCost(world, nx, y + 1, nz,
                dist, SPRINT_SPEED) + JUMP_PENALTY;
            out.accept(StateCodec.pack(nx, y + 1, nz, blocks, false), cost, Action.CLIMB);
        }
    }

    /** Vertical mining straight down from (x,y,z). */
    public static void mineDownEdge(World world, int x, int y, int z, int blocks, MineCostModel tools, EdgeConsumer out) {
        int belowHere = world.voxelAt(x, y - 1, z);
        if (BlockType.isSolid(belowHere) && !BlockType.isUnbreakable(belowHere)) {
            double cost = tools.mineTime(belowHere) + 0.2;
            out.accept(StateCodec.pack(x, y - 1, z, blocks, false), cost, Action.MINE_DOWN);
        }
    }

    /**
     * How strongly the vertical delta must dominate the horizontal delta
     * for bridgeUpEdge to consider pillaring worthwhile -- fires when
     * (goalY - originY) > BRIDGE_UP_PRUNE_RATIO * horizontalDistanceToGoal.
     * Larger values are STRICTER (require an even more lopsided case before
     * pillaring is offered, so fewer opportunities survive); smaller values
     * are more permissive, down to 0.0 which reduces the check to just
     * "goal is above me at all".
     *
     * Default 0.0 (not 1.0): a real-terrain sweep across many tested
     * regions found 1.0 was too strict and silently missing better paths --
     * on the hardest region, 0.0 found a meaningfully cheaper path using
     * far fewer expansions and running several times faster than 1.0. The
     * effect is NOT monotonic with the ratio, though: an intermediate value
     * measured worse than both 1.0 and 0.0 on that same region, so
     * "somewhere in between" isn't a safe intermediate choice -- only the
     * fully-loose end reliably paid off in that sweep. At least one region
     * got noticeably slower at 0.0 with no quality gain (more BRIDGE_UP
     * candidates explored that never pay off there) -- a real but bounded
     * cost, not a blowup. Mutable/tunable like MINE_PRUNE_THRESHOLD and
     * AirPotential.OLD_GROUND_CUTOFF elsewhere in this file if a specific
     * region ever needs revisiting.
     */
    public static volatile double BRIDGE_UP_PRUNE_RATIO = EngineDefaults.bridgeUpPruneRatio();

    /**
     * Straight-up pillar move: place a block beneath yourself and rise one
     * level in place (dx=dz=0), the real-Minecraft "pillaring" technique.
     * A plain BRIDGE only ever fires for a flat void crossing or rising
     * above LAVA specifically, so without this a plain solid cliff face
     * would have no way to ascend at all.
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
     * metadata -- it inherits from the predecessor's originY if the edge
     * that reached THIS state was itself BRIDGE_UP, or resets to this
     * state's own y otherwise (a fresh potential chain start).
     */
    public static void bridgeUpEdge(World world, int x, int y, int z, int blocks, MineCostModel tools,
                                     int originY, int goalX, int goalY, int goalZ, EdgeConsumer out) {
        bridgeUpEdge(world, x, y, z, blocks, tools, originY,
            GoalPoints.point(goalX, goalY, goalZ), out);
    }

    public static void bridgeUpEdge(World world, int x, int y, int z, int blocks, MineCostModel tools,
                                     int originY, GoalPoints goal, EdgeConsumer out) {
        if (blocks <= 0) {
            return;
        }
        boolean anyTargetJustifiesPillaring = false;
        for (int i = 0; i < goal.size(); i++) {
            int dyGoal = goal.y(i) - originY;
            int horizGoal = Math.abs(goal.x(i) - x) + Math.abs(goal.z(i) - z);
            if (dyGoal > 0 && dyGoal > BRIDGE_UP_PRUNE_RATIO * horizGoal) {
                anyTargetJustifiesPillaring = true;
                break;
            }
        }
        if (!anyTargetJustifiesPillaring) {
            return;
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

        double tax = blockTax(blocks, goal.minimumBlockReserve());
        double cost = PLACE_TIME + BRIDGE_RISK_PENALTY + tax + towerTime(1);
        out.accept(StateCodec.pack(x, targetY, z, blocks - 1, false), cost, Action.BRIDGE_UP);
    }

    // ============================================================
    // Reverse (predecessor) edge generation, for a bidirectional search's
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
     * was silently unreachable by backward search until this was
     * generalized, because the SPRINT reverse-probe of the position right
     * after the bridge chain rejected the bridge landing as its source.
     *
     * This is narrowly scoped to exactly BRIDGE's own generation
     * precondition, and even if a given floating candidate never actually
     * turns out to be reverse-bridgeable all the way back to real ground,
     * accepting it tentatively is harmless: the FORWARD half of any
     * reconstructed path always comes from forward's own independently-
     * validated closed state, not from backward's predecessor-discovery --
     * backward only ever contributes the segment FROM the meeting point
     * TOWARD the goal, and every edge in that segment (including this one)
     * is validated by the same horizontalEdges/climbEdge/etc forward would
     * use. A candidate source that never turns out to be real just becomes
     * an unproductive dead end in backward's OWN further expansion, not an
     * invalid accepted path.
     */
    private static boolean isValidRestingSource(World world, int x, int y, int z, boolean crawling) {
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
     * Sanity cap on reverse-BRIDGE chain length -- not a state-space bound
     * (backward doesn't fork state by bridge count, so there's no
     * combinatorial blowup risk this needs to guard against). This just
     * stops a pathological, effectively infeasible chain from being probed
     * forever; any real search is also bounded by blocksAvailable
     * regardless.
     */
    public static volatile int MAX_REVERSE_BRIDGE_DEPTH = EngineDefaults.maxReverseBridgeDepth();

    /**
     * Predecessors of a non-crawling target (tx,ty,tz): SPRINT, MINE, CLIMB,
     * MINE_DOWN, FALL, BRIDGE. (BOAT_CRAWL never lands with crawling=false,
     * so it's never a predecessor here -- see reverseCrawlSources.)
     *
     * airPotential (nullable, null = no prune) is threaded through to the
     * MINE reverse-probe ONLY, using the exact same forward horizontalEdges
     * call forward search itself would make from that candidate source --
     * so a source this probe finds is guaranteed to be one forward's own
     * expand() would also have offered, keeping backward's MINE candidates
     * from silently diverging from forward's pruned set. Manhattan pruning
     * is deliberately NOT threaded through here -- it measured as a much
     * smaller win than AirPotential and isn't worth the extra parameter.
     * SPRINT is unaffected either way: horizontalEdges only ever applies
     * airPotential's prune to MINE candidates.
     *
     * bridgesUsedSoFarAtTarget is used ONLY to compute reverseBridgeSources'/
     * reverseBridgeUpSource's worst-case cost estimate -- see their
     * javadocs. It's NOT part of any packed state here; every predecessor
     * this method emits is a plain (x,y,z,crawling) state, same as always.
     * The caller is responsible for tracking bridge count as auxiliary
     * per-state metadata, not identity.
     *
     * goalX/goalY/goalZ are needed for reverseBridgeUpSource, which reuses
     * bridgeUpEdge's own goal-relative prune (see its javadoc) so backward's
     * BRIDGE_UP candidates can never diverge from what forward would
     * actually offer.
     */
    public static void reverseNonCrawlEdges(World world, int tx, int ty, int tz, MineCostModel tools,
                                             AirPotential airPotential, int bridgesUsedSoFarAtTarget,
                                             int goalX, int goalY, int goalZ, EdgeConsumer out) {
        for (int d = 0; d < 8; d++) {
            int dx = DIR_DX[d], dz = DIR_DZ[d];
            int sx = tx - dx, sz = tz - dz;
            if (sx < 0 || sx >= world.sizeX || sz < 0 || sz >= world.sizeZ) {
                continue;
            }

            // SPRINT/MINE/CLIMB all cost the same regardless of whether the
            // source was already crawling -- only BOAT_CRAWL's own cost
            // reads the source's crawling flag (see reverseCrawlEdges).
            // Hardcoding false here would mean backward could never
            // discover "stand up out of a crawl, then SPRINT/MINE/CLIMB
            // away" -- silently blind to any route ending a BOAT_CRAWL
            // chain. Try both source-crawling states, same fix as
            // reverseCrawlEdges already applies.
            for (boolean srcCrawling : new boolean[]{false, true}) {
                // SPRINT: source at the same y as the target.
                horizontalEdges(world, sx, ty, sz, 0, srcCrawling, dx, dz, tools,
                        (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.SPRINT, tx, ty, tz, sx, ty, sz, srcCrawling, out));
                horizontalEdges(world, sx, ty, sz, 0, srcCrawling, dx, dz, tools, airPotential,
                        (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.MINE, tx, ty, tz, sx, ty, sz, srcCrawling, out));

                // CLIMB: source one level below the target.
                int climbSy = ty - 1;
                if (climbSy >= 0) {
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
            // bridge block is not "something you fell onto"). Without this
            // guard, backward could reverse-discover a "FALL" onto a
            // bridge-plausible-but-not-solid target -- a physically
            // impossible landing.
            if (BlockType.isSolid(world.voxelAt(tx, ty - 1, tz))) {
                reverseFallSources(world, tx, ty, tz, dx, dz, sx, sz, out);
            }
        }

        reverseBridgeSources(world, tx, ty, tz, tools, bridgesUsedSoFarAtTarget, out);
        reverseBridgeUpSource(world, tx, ty, tz, tools, bridgesUsedSoFarAtTarget, goalX, goalY, goalZ, out);

        // MINE_DOWN: exactly one candidate source, directly above -- always
        // a valid resting source since (tx,ty,tz) being solid+breakable
        // (required for MINE_DOWN to apply at all) is exactly what
        // isValidRestingSource would check here.
        int mdSy = ty + 1;
        if (mdSy < world.sizeY) {
            for (boolean srcCrawling : new boolean[]{false, true}) {
                mineDownEdge(world, tx, mdSy, tz, 0, tools,
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
    public static void reverseCrawlEdges(World world, int tx, int ty, int tz, MineCostModel tools, EdgeConsumer out) {
        for (int d = 0; d < 8; d++) {
            int dx = DIR_DX[d], dz = DIR_DZ[d];
            int sx = tx - dx, sz = tz - dz;
            if (sx < 0 || sx >= world.sizeX || sz < 0 || sz >= world.sizeZ) {
                continue;
            }
            for (boolean srcCrawling : new boolean[]{false, true}) {
                horizontalEdges(world, sx, ty, sz, 0, srcCrawling, dx, dz, tools,
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
     * BRIDGE would otherwise need to be excluded from backward entirely:
     * its cost depends on blockTax(blocksRemaining), the ABSOLUTE resource
     * count at the moment of placement, and backward -- working outward
     * from the goal -- has no way to know what forward's absolute
     * remaining will be at the point they meet.
     *
     * Tracking blocksNeeded as part of backward's STATE IDENTITY (a whole
     * separate copy of the reachable graph per possible bridge count) is
     * correct but a disaster on real terrain: caves/overhangs mean a huge
     * fraction of the map is void-adjacent, so forking state by bridge
     * count makes backward duplicate large chunks of its own reachable
     * graph per depth level, and deep chains never converge within a
     * reasonable expansion budget at all.
     *
     * This version tracks bridge count as a plain SCALAR alongside g,
     * exactly like cameFromId/actionUsed already are -- no forking, no
     * extra states, backward's state space stays (x,y,z,crawling) same as
     * always. The cost this method computes is a WORST-CASE estimate: it
     * assumes THIS specific bridge (the bridgesUsedSoFarAtTarget-th one
     * encountered walking outward from goal) is being placed on forward's
     * LAST available block, i.e. blockTax(1); a second bridge further out
     * assumes blockTax(2); and so on -- "assume you're always down to your
     * last few blocks" rather than "assume you have plenty" (the
     * full-state-forking version's admissible-but-wrong-direction choice).
     * This deliberately OVERESTIMATES cost, so it's not perfectly sound for
     * the meeting/termination criterion -- but it's a small, BOUNDED
     * pessimism (unlike full exclusion's effectively-infinite one), and it
     * costs nothing extra in state space. The final reported cost is
     * corrected exactly at reconstruction time regardless, once the true
     * meeting-point blocksRemaining is known.
     */
    public static void reverseBridgeSources(World world, int tx, int ty, int tz, MineCostModel tools,
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
            for (boolean srcCrawling : new boolean[]{false, true}) {
                if (!isValidRestingSource(world, sx, ty, sz, srcCrawling)) {
                    continue;
                }
                horizontalEdges(world, sx, ty, sz, assumedRemaining, srcCrawling, dx, dz, tools,
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
    public static void reverseBridgeUpSource(World world, int tx, int ty, int tz, MineCostModel tools,
                                              int bridgesUsedSoFarAtTarget, int goalX, int goalY, int goalZ, EdgeConsumer out) {
        int assumedRemaining = bridgesUsedSoFarAtTarget + 1;
        if (assumedRemaining > MAX_REVERSE_BRIDGE_DEPTH) {
            return;
        }
        int sy = ty - 1;
        if (sy < 0) {
            return;
        }
        for (boolean srcCrawling : new boolean[]{false, true}) {
            if (!isValidRestingSource(world, tx, sy, tz, srcCrawling)) {
                continue;
            }
            bridgeUpEdge(world, tx, sy, tz, assumedRemaining, tools, sy, goalX, goalY, goalZ,
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
        double dist = Math.sqrt((double) (dx * dx + dz * dz));
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
            // cascading bogus edges.
            if (!BlockType.isSolid(world.voxelAt(sx, y - 1, sz))) {
                continue;
            }

            double fallCost = dist / SPRINT_SPEED + dropAmount * FALL_PENALTY_PER_BLOCK;
            if (dropAmount > CLUTCH_THRESHOLD) {
                fallCost += CLUTCH_SETUP_TIME;
            }
            if (thisPassedLava) {
                fallCost += LAVA_DEATH_PENALTY;
            }
            for (boolean srcCrawling : new boolean[]{false, true}) {
                out.accept(StateCodec.pack(sx, y, sz, 0, srcCrawling), fallCost, Action.FALL);
            }
        }
    }

    private EdgeRules() {}
}
