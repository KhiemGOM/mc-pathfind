package dev.mcpathfind.pearl;

/**
 * Per-action edge legality/cost computation for the bastion-to-fortress /
 * fortress-to-stronghold route -- forked from dev.mcpathfind.core.EdgeRules,
 * which see for the bridging/parkour-oriented original and the full
 * reasoning behind everything kept here unchanged (SPRINT/MINE/BOAT_CRAWL/
 * CLIMB/MINE_DOWN/FALL cost formulas, the reuse-forward-logic philosophy for
 * every reverse probe, the diagonal-corner-cut rule, etc.).
 *
 * BRIDGE, BRIDGE_UP, and PARKOUR are deliberately dropped -- out of scope
 * for this route, and their removal also means blockTax/BLOCK_TAX_* and the
 * BRIDGE-plausible-floating-position clause in isValidRestingSource go away
 * too (nothing left that ever creates a no-solid-floor resting position).
 *
 * PEARL is new and NOT YET IMPLEMENTED -- pearlEdges/reversePearlSources
 * below are stubs (emit zero edges) so the rest of this solver compiles and
 * is independently testable against the surviving action set first. See
 * their javadocs for the full physics/algorithm design already worked out,
 * so implementing them is a filling-in-the-blanks exercise, not a
 * from-scratch design session.
 */
public final class EdgeRules {

    public static final int[] DIR_DX = {1, -1, 0, 0, 1, 1, -1, -1};
    public static final int[] DIR_DZ = {0, 0, 1, -1, 1, -1, 1, -1};

    public static final double BOAT_CRAWL_TAX = 5.0;
    public static final double BOAT_CRAWL_SPEED = 3.0;
    public static final double SPRINT_SPEED = 5.6;
    public static final double WALK_SPEED = 4.3;
    public static final double JUMP_PENALTY = 0.15;
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

    // ============================================================
    // PEARL: ender pearl throw with a parabolic (gravity+drag) arc that
    // teleports the player to wherever it first hits something. NOT YET
    // IMPLEMENTED -- see pearlEdges' javadoc for the full design.
    // ============================================================

    // Verified against real 1.16.1 physics via tick-based simulation (not
    // the current-version wiki figures, which use a different post-1.21.2
    // update order -- see the design discussion this was forked from).
    // Per-tick update order for 1.16.1: Position, then Drag, then
    // Acceleration -- i.e. pos += vel; vel *= DRAG; vel.y -= GRAVITY.
    public static final double PEARL_GRAVITY = 0.03;
    public static final double PEARL_DRAG = 0.99;
    // g/(1-drag) -- the vertical recurrence's fixed point (terminal fall
    // speed). Confirmed EXACTLY 3.0 for 1.16.1's order, not the 2.97 the
    // current wiki quotes (that's the post-1.21.2 reordered tick).
    public static final double PEARL_TERMINAL_G = PEARL_GRAVITY / (1.0 - PEARL_DRAG);
    // Throw speed: (lookVector, unit length) * POWER. Player's own motion
    // and the small random inaccuracy are both deliberately ignored here --
    // this models a stationary, aimed throw for planning purposes, not live
    // execution precision.
    public static final double PEARL_THROW_SPEED = 1.5;
    // Launch origin: eye height above feet.
    public static final double PEARL_EYE_HEIGHT = 1.62;
    // Both position formulas are EXACT closed forms of the discrete
    // per-tick recurrence above (linear drag + discrete time = geometric
    // series, no simulation loop needed to evaluate a given tick):
    //   horizontal(n) = h0 + vh0 * (1 - PEARL_DRAG^n) / (1 - PEARL_DRAG)
    //   vertical(n)   = y0 + (vy0 + PEARL_TERMINAL_G) * (1 - PEARL_DRAG^n) / (1 - PEARL_DRAG)
    //                        - PEARL_TERMINAL_G * n
    // Solving for a target (dx,dz,dy) is a 1D search over integer tick
    // count n (yaw is exact/trivial -- straight toward the target's
    // horizontal bearing; pitch is recovered directly from n via the
    // horizontal formula, then checked against the vertical formula) --
    // NOT a 2D angle sweep, and NOT a tick-by-tick simulation loop per
    // candidate. Max useful n is small (~60 ticks even at max range).

    // Coarse destination-grid cell size -- PEARL's reachable set is a
    // continuous 2D manifold per source (unlike every other action's small
    // discrete offset table), so candidates are generated by DESTINATION
    // cell instead of by throw angle, same role PARKOUR_KERNEL plays for
    // jumps just at a much coarser, longer-range scale. Starting point per
    // design discussion; expect this to need empirical tuning against real
    // terrain the same way MINE_PRUNE_THRESHOLD/BRIDGE_UP_PRUNE_RATIO did.
    public static double PEARL_CELL_SIZE = 6.0;
    // Pearls are a scarce resource (no infinite supply) -- exclude cells a
    // cheaper action would already cover just as well, incentivizing the
    // long throw pearls are actually useful for. This is a HARD gate
    // (candidates below this distance are never even generated), which is
    // why it's the primary lever against short/marginal throws rather
    // than PEARL_AIM_TAX alone: at 12.0, a min-distance throw's flight
    // time alone (~0.4s) was already so much cheaper than sprinting the
    // same distance (~2.1s) that no reasonable flat tax could fix it
    // without also over-taxing genuinely long, valuable throws (pearls
    // get cheaper per block than sprinting AS distance grows, not less --
    // ~0.04s of flight time per additional block vs sprint's flat
    // ~0.18s/block, so a tax sized to break even at a low min-distance
    // ends up far too small at any greater distance, or a tax sized to
    // matter at longer range makes even genuinely worthwhile close-in
    // throws lose to walking too hard). Raised to 20.0 so "running would
    // plainly be better" distances are excluded outright, leaving
    // PEARL_AIM_TAX to do much lighter work at the distances that remain.
    public static double PEARL_MIN_DISTANCE = 20.0;
    // Coarse cell-level reject-fast gate, generous above the simulated
    // ~52-block max (flat launch/landing) since a downhill throw can go
    // further -- solvePearlArc is the real authority on reachability, this
    // just avoids resolving/checking cells that can't possibly work.
    public static double PEARL_MAX_DISTANCE = 55.0;
    // Grid half-width in cells -- covers PEARL_MAX_DISTANCE with margin at
    // the default cell size (55/6 ~= 9.2).
    public static int PEARL_GRID_RADIUS_CELLS = 10;
    // Search bound for solvePearlArc's tick-count scan -- generous above
    // the simulated ~60-tick max-range flight time.
    public static int PEARL_MAX_TICKS = 120;
    // Generous on purpose: a destination is already a corner of a
    // PEARL_CELL_SIZE-wide coarse cell, not an exact block, so landing
    // anywhere within about one cell's width of it is treated as the same
    // outcome -- matches PEARL_CELL_SIZE's default directly. This also
    // means solvePearlArc's continuous search (see its javadoc) doesn't
    // need to check every single tick to find a match.
    public static double PEARL_LANDING_TOLERANCE = 6.0;
    // Coarse step (in ticks, continuous) for solvePearlArc's flight-time
    // search -- see its javadoc for why this can be much larger than 1
    // tick now that PEARL_LANDING_TOLERANCE is generous.
    public static double PEARL_ARC_TICK_STEP = 8.0;
    // Total blocked-sample thresholds -- see the class-level design notes:
    // a low total across an otherwise-clear path is a discretization
    // near-miss (worth trying a different corner), a high total is a
    // genuine wall. LOS is a cheap reject-fast pre-filter (lenient); the
    // real arc check is the actual authority (stricter) -- independently
    // tunable since they play different roles.
    public static int PEARL_LOS_BLOCK_THRESHOLD = 2;
    // The real parabolic arc is the actual in-game trajectory, not a cheap
    // approximation -- any sample it clips is a genuinely blocked throw, so
    // this stays exact (zero tolerance), unlike the LOS pre-filter above.
    public static int PEARL_ARC_BLOCK_THRESHOLD = 0;
    // Corner-retry budget per cell -- each is a full ballistic solve, not a
    // cheap check, so this stays small.
    public static int PEARL_MAX_RETRY_CORNERS = 3;
    // Flat cost for the time/attention of lining up and committing to a
    // throw, plus a light secondary nudge toward pearl SCARCITY -- unlike
    // BRIDGE's blockTax (which scales with remaining block count),
    // PEARL's state deliberately doesn't track pearls-remaining at all,
    // so there's no resource-aware cost term here, just this flat tax.
    // The PRIMARY defense against short/marginal throws is
    // PEARL_MIN_DISTANCE's hard gate (see its javadoc), not this tax --
    // an earlier attempt tried raising this alone to ~3.0 to force short
    // throws to lose on cost, but pearls get cheaper per block than
    // sprinting as distance grows (not less), so a tax sized to fix the
    // short-throw case ends up needlessly discouraging genuinely long,
    // valuable ones too (confirmed: on k2_r_0_0, tax=3.0 cut PEARL usage
    // from 12 to 8 throws but made the route 33% SLOWER overall). With
    // the min-distance gate now doing the real exclusion work, this only
    // needs to be big enough to avoid a literal tie at the margin right
    // past the gate, not to fight the whole cost gap on its own.
    public static double PEARL_AIM_TAX = 1.5;
    // Fire resistance is always active by the time either PEARL leg runs,
    // so lava along an arc's flight path is no longer a death sentence --
    // no longer grounds to reject the throw at all, just a small flat tax
    // so an arc that clips lava doesn't tie exactly with an otherwise-
    // identical lava-free one. Landing IN lava is still rejected outright
    // (see solvePearlEdge) -- fire res stops the damage, not the fact that
    // lava is a liquid you can't stand on.
    public static double PEARL_LAVA_TAX = 2.0;
    // Steep throws (large launch-angle magnitude, near-vertical lobs) are
    // harder to line up and execute reliably in real play than a flatter
    // throw covering the same distance, so this nudges the search toward
    // "lower" (flatter) pearls over "higher" (steeper) ones purely via
    // cost -- applied AFTER the arc is solved (the angle is an OUTPUT of
    // solvePearlArc, not something known in advance), so it can never
    // change reachability, only which of several already-valid throws
    // wins on cost. Two thresholds, not a hard gate: beyond
    // PEARL_STEEP_ANGLE_DEGREES the throw is genuinely non-optimal (full
    // tax), between that and PEARL_MODERATE_ANGLE_DEGREES it's just a
    // light nudge. Uses the pitch magnitude regardless of sign, since a
    // steep downward throw is just as awkward to aim as a steep upward
    // lob.
    public static double PEARL_STEEP_ANGLE_DEGREES = 45.0;
    public static double PEARL_MODERATE_ANGLE_DEGREES = 30.0;
    public static double PEARL_STEEP_ANGLE_TAX = 2.5;
    public static double PEARL_MODERATE_ANGLE_TAX = 0.75;
    // Continuous-tick step for the real arc collision scan -- see
    // sampleArcCollisions' javadoc for why gap-safety no longer depends
    // on this being small (a bounding-box check between consecutive
    // samples closes that gap regardless of spacing): 1.0 tick matches
    // this project's original per-tick granularity, now with a stronger
    // gap guarantee than that original design ever had (the box check
    // also closes the diagonal-corner-skip case a single-point-per-tick
    // check never covered).
    public static double PEARL_ARC_SAMPLE_TICK_STEP = 1.0;

    private static final double PEARL_DRAG_COMPLEMENT = 1.0 - PEARL_DRAG;

    /** S(n) = (1 - drag^n) / (1 - drag) -- the geometric series sum factor shared by both position formulas. */
    private static double pearlSumFactor(int n) {
        return (1.0 - Math.pow(PEARL_DRAG, n)) / PEARL_DRAG_COMPLEMENT;
    }

    /** Same drag-decay sum, but for continuous (non-integer) flight time -- Math.pow handles a real exponent fine, drag^t is smooth in t. */
    private static double continuousSumFactor(double t) {
        return (1.0 - Math.pow(PEARL_DRAG, t)) / PEARL_DRAG_COMPLEMENT;
    }

    /** Post-solve cost nudge toward flatter throws -- see PEARL_STEEP_ANGLE_TAX's javadoc. */
    private static double angleTax(double pitchRadians) {
        double degrees = Math.abs(Math.toDegrees(pitchRadians));
        if (degrees > PEARL_STEEP_ANGLE_DEGREES) {
            return PEARL_STEEP_ANGLE_TAX;
        }
        if (degrees > PEARL_MODERATE_ANGLE_DEGREES) {
            return PEARL_MODERATE_ANGLE_TAX;
        }
        return 0.0;
    }

    // Package-visible (not private): PathValidator independently re-derives
    // and re-checks the arc for every PEARL step in a returned path, rather
    // than trusting the solver's own bookkeeping -- see its PEARL case.
    record PearlArc(int ticks, double pitch, double yaw, double cost) {}

    /**
     * Finds the (pitch, tick count) that lands a pearl at (dx,dy,dz), or
     * null if nothing within PEARL_MAX_TICKS does. Not a full closed-form
     * solve -- the vertical equation mixes an exponential decay term
     * (drag^t) with a linear one (-G*t), the same transcendental shape as
     * x = a - b*c^x, which has no elementary algebraic solution for t in
     * general (needs Lambert-W or a numerical solve). What IS closed-form
     * is solving for pitch given a FIXED t (the horizontal-distance
     * equation inverts directly -- see continuousSumFactor), so the
     * search only ever needs to sweep the one variable that doesn't have
     * a formula: flight time.
     *
     * That sweep is over CONTINUOUS t, not integer ticks one at a time --
     * drag^t is smooth for any real t, and since PEARL_LANDING_TOLERANCE
     * is generous (any landing within about one destination cell is
     * treated as equivalent, not an exact block), the search can afford
     * to step PEARL_ARC_TICK_STEP ticks at a time instead of checking
     * every single one. This is the dominant cost of a failed candidate
     * (no valid arc means the full range gets swept before giving up), so
     * a coarser step is a direct multiple-times reduction on exactly that
     * worst case. Once a coarse step lands within tolerance, the actual
     * returned tick count is re-solved exactly at the nearest integer
     * ticks (solvePearlArcAtTick) so the reported PearlArc is always
     * internally consistent with a real discrete tick count, never an
     * interpolated approximation. Preferring the smallest such n (rather
     * than the globally best-fitting one) is unchanged from before --
     * cheapest is fastest here, since cost is exactly flight time + a
     * flat tax.
     */
    static PearlArc solvePearlArc(double dx, double dy, double dz) {
        double horizDist = Math.sqrt(dx * dx + dz * dz);
        if (horizDist < 1e-6) {
            return null; // not a meaningful "throw toward a destination"
        }
        double yaw = Math.atan2(dz, dx);

        // Smallest continuous flight time that can even reach horizDist --
        // S(t) rises monotonically toward 1/(1-drag) as t grows, so this
        // is a direct solve (S(t) = horizDist/speed => invert for t), not
        // a search.
        double maxReachableS = 1.0 / PEARL_DRAG_COMPLEMENT;
        double requiredS = horizDist / PEARL_THROW_SPEED;
        if (requiredS >= maxReachableS) {
            return null;
        }
        double tMin = Math.log(1.0 - requiredS * PEARL_DRAG_COMPLEMENT) / Math.log(PEARL_DRAG);

        for (double t = Math.max(1.0, tMin); t <= PEARL_MAX_TICKS; t += PEARL_ARC_TICK_STEP) {
            double s = continuousSumFactor(t);
            double cosPitch = horizDist / (PEARL_THROW_SPEED * s);
            if (cosPitch > 1.0) {
                continue; // not enough tick-time yet to cover this distance even at full speed
            }
            double sinMagnitude = Math.sqrt(Math.max(0.0, 1.0 - cosPitch * cosPitch));
            boolean promising = false;
            for (double sign : new double[] {1.0, -1.0}) {
                double vy0 = sign * sinMagnitude * PEARL_THROW_SPEED;
                double predictedDy = (vy0 + PEARL_TERMINAL_G) * s - PEARL_TERMINAL_G * t;
                if (Math.abs(predictedDy - dy) <= PEARL_LANDING_TOLERANCE) {
                    promising = true;
                }
            }
            if (!promising) {
                continue;
            }
            int nFloor = Math.max(1, (int) Math.floor(t));
            PearlArc arc = solvePearlArcAtTick(nFloor, horizDist, dy, yaw);
            if (arc == null) {
                arc = solvePearlArcAtTick(nFloor + 1, horizDist, dy, yaw);
            }
            if (arc != null) {
                return arc;
            }
            // The continuous estimate suggested this range works but
            // neither neighboring integer tick actually lands within
            // tolerance (a real but rare edge case near the tolerance
            // boundary) -- keep scanning forward rather than giving up.
        }
        return null;
    }

    /**
     * Exact discrete check at one integer tick count -- the same formula
     * the original per-tick loop used, kept as the final authority once
     * solvePearlArc's continuous search has narrowed down a candidate
     * flight time. Returns null if this specific tick count doesn't
     * actually land within tolerance for either aim direction.
     */
    private static PearlArc solvePearlArcAtTick(int n, double horizDist, double dy, double yaw) {
        if (n < 1) {
            return null;
        }
        double s = pearlSumFactor(n);
        double cosPitch = horizDist / (PEARL_THROW_SPEED * s);
        if (cosPitch > 1.0) {
            return null;
        }
        double sinMagnitude = Math.sqrt(Math.max(0.0, 1.0 - cosPitch * cosPitch));
        for (double sign : new double[] {1.0, -1.0}) {
            double sinPitch = sign * sinMagnitude;
            double vy0 = sinPitch * PEARL_THROW_SPEED;
            double predictedDy = (vy0 + PEARL_TERMINAL_G) * s - PEARL_TERMINAL_G * n;
            if (Math.abs(predictedDy - dy) <= PEARL_LANDING_TOLERANCE) {
                double pitch = Math.atan2(sinPitch, cosPitch);
                double flightSeconds = n / 20.0; // Minecraft ticks at 20/sec
                return new PearlArc(n, pitch, yaw, flightSeconds + PEARL_AIM_TAX);
            }
        }
        return null;
    }

    /**
     * Cheap straight-line LOS pre-filter -- see the class-level PEARL
     * design notes for why this is a valid (deliberately incomplete)
     * pre-filter: practical throw angles keep the arc ABOVE a direct
     * sightline for most of the flight, so if straight LOS is already
     * badly blocked the arc almost certainly is too. Solid blocks still
     * count against the (lenient) threshold, but lava no longer does,
     * matching the real arc check below: fire res means lava is never a
     * rejection reason on either check now, just a cost tax applied once
     * the real arc is known. Exits as soon as the count exceeds
     * PEARL_LOS_BLOCK_THRESHOLD -- the candidate is already rejected at
     * that point regardless of what the rest of the line looks like, for
     * any threshold value, so there's nothing left to learn from finishing
     * the scan.
     */
    private static int countLosBlockedSamples(double sourceX, double sourceY, double sourceZ,
                                               int targetX, int targetY, int targetZ, World world) {
        double dx = targetX + 0.5 - sourceX, dy = targetY - sourceY, dz = targetZ + 0.5 - sourceZ;
        double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        int samples = Math.max(2, (int) Math.round(dist));
        int blocked = 0;
        for (int s = 1; s < samples; s++) {
            double t = (double) s / samples;
            int sx = (int) Math.floor(sourceX + dx * t);
            int sy = (int) Math.floor(sourceY + dy * t);
            int sz = (int) Math.floor(sourceZ + dz * t);
            int voxel = world.voxelAt(sx, sy, sz);
            if (BlockType.isSolid(voxel)) {
                blocked++;
                if (blocked > PEARL_LOS_BLOCK_THRESHOLD) {
                    return blocked;
                }
            }
        }
        return blocked;
    }

    record ArcSampleResult(int blockedSolid, int lavaHits) {}

    /**
     * Real arc collision scan -- solid blocks are still a hard reject
     * (blockedSolid), lava is now just counted (lavaHits) for
     * solvePearlEdge to tax instead of reject, per PEARL_LAVA_TAX's
     * javadoc.
     *
     * Checks every cell in the bounding box between each sample and the
     * previous one (starting from the launch point itself at t=0), not
     * just the two sampled points -- a plain two-point check, however
     * densely sampled, can still let the arc "cut a diagonal corner"
     * through a solid seam between two face-adjacent cells without
     * either sample ever landing inside it, the same class of bug
     * diagonalCornerBlocked guards against for ordinary horizontal
     * movement elsewhere in this codebase. The box check closes that gap
     * outright and, unlike a two-point check, doesn't depend on samples
     * being close together for correctness -- so PEARL_ARC_SAMPLE_TICK_STEP
     * can go back to this project's original 1-tick granularity instead
     * of the much denser (and more expensive) fixed step an earlier
     * version of this needed purely to keep single-point gaps small.
     * Consecutive samples landing in the exact same voxel (box
     * degenerates to one cell) just re-check that one cell -- cheap
     * enough on its own not to need a separate same-voxel skip.
     *
     * Exits as soon as blockedSolid exceeds PEARL_ARC_BLOCK_THRESHOLD --
     * same reasoning as countLosBlockedSamples' early exit: the candidate
     * is already rejected at that point for any threshold value, so
     * there's nothing left to learn from finishing the scan.
     */
    static ArcSampleResult sampleArcCollisions(double sourceX, double sourceY, double sourceZ,
                                                double yaw, double pitch, int ticks, World world) {
        double vh0 = PEARL_THROW_SPEED * Math.cos(pitch);
        double vy0 = PEARL_THROW_SPEED * Math.sin(pitch);
        double cosYaw = Math.cos(yaw), sinYaw = Math.sin(yaw);
        int blockedSolid = 0;
        int lavaHits = 0;
        int prevX = (int) Math.floor(sourceX);
        int prevY = (int) Math.floor(sourceY);
        int prevZ = (int) Math.floor(sourceZ);
        double t = 0.0;
        while (t < ticks) {
            t = Math.min(t + PEARL_ARC_SAMPLE_TICK_STEP, ticks);
            double s = continuousSumFactor(t);
            double h = vh0 * s;
            double dy = (vy0 + PEARL_TERMINAL_G) * s - PEARL_TERMINAL_G * t;
            int sx = (int) Math.floor(sourceX + h * cosYaw);
            int sy = (int) Math.floor(sourceY + dy);
            int sz = (int) Math.floor(sourceZ + h * sinYaw);
            int loX = Math.min(prevX, sx), hiX = Math.max(prevX, sx);
            int loY = Math.min(prevY, sy), hiY = Math.max(prevY, sy);
            int loZ = Math.min(prevZ, sz), hiZ = Math.max(prevZ, sz);
            for (int xi = loX; xi <= hiX; xi++) {
                for (int yi = loY; yi <= hiY; yi++) {
                    for (int zi = loZ; zi <= hiZ; zi++) {
                        int voxel = world.voxelAt(xi, yi, zi);
                        if (BlockType.isSolid(voxel)) {
                            blockedSolid++;
                        }
                        else if (BlockType.isLava(voxel)) {
                            lavaHits++;
                        }
                    }
                }
            }
            if (blockedSolid > PEARL_ARC_BLOCK_THRESHOLD) {
                return new ArcSampleResult(blockedSolid, lavaHits);
            }
            prevX = sx;
            prevY = sy;
            prevZ = sz;
        }
        return new ArcSampleResult(blockedSolid, lavaHits);
    }

    private record PearlCorner(int x, int y, int z, double distSq) {}

    /**
     * The (up to 4) candidate landing corners for one cell, resolved to a
     * standable Y and distance-sorted from anchorX/anchorZ (closest first)
     * -- NOT random (would break this solver's determinism), matching the
     * design discussion. Shared between pearlEdges (anchor = the source,
     * looking for TARGET candidates) and reversePearlSources (anchor = the
     * target, looking for SOURCE candidates) -- same corner-finding logic
     * either way, only which role the result plays differs.
     */
    private static java.util.List<PearlCorner> pearlCellCorners(World world, int cellMinX, int cellMinZ,
                                                                  int preferredY, double anchorX, double anchorZ) {
        int cellMaxX = cellMinX + (int) PEARL_CELL_SIZE;
        int cellMaxZ = cellMinZ + (int) PEARL_CELL_SIZE;
        int[][] xz = {{cellMinX, cellMinZ}, {cellMinX, cellMaxZ}, {cellMaxX, cellMinZ}, {cellMaxX, cellMaxZ}};
        java.util.List<PearlCorner> corners = new java.util.ArrayList<>(4);
        for (int[] candidate : xz) {
            int y = world.nearestStandableColumnY(candidate[0], candidate[1], preferredY);
            if (y < 0) {
                continue;
            }
            double dx = candidate[0] - anchorX, dz = candidate[1] - anchorZ;
            corners.add(new PearlCorner(candidate[0], y, candidate[1], dx * dx + dz * dz));
        }
        corners.sort(java.util.Comparator.comparingDouble(PearlCorner::distSq));
        return corners;
    }

    /**
     * The single shared reachability check for one (source, target) pair --
     * distance gate, LOS pre-filter, closed-form arc solve, landing
     * legality, then the real arc collision check, in cheapest-to-most-
     * expensive order (same principle as PARKOUR's hasJumpableGap gate
     * before its kernel scan). Returns the solved arc (cost included) or
     * null. Used by BOTH pearlEdges (candidate = target, source fixed) and
     * reversePearlSources (candidate = source, target fixed) -- see the
     * class-level design notes, point 7: the reverse probe is NOT separate
     * physics, it's this exact same check called with the roles swapped.
     *
     * targetAlreadyVerifiedStandable lets pearlEdges skip re-deriving
     * landing legality for its candidate corners: pearlCellCorners already
     * confirmed each corner via World.isStandableColumn (the exact same
     * solid-floor/clear-headroom condition checked below) when it resolved
     * that corner in the first place, so re-checking it here is pure
     * duplicate work -- profiled at ~34% of total solve time, the single
     * biggest hotspot, almost entirely call-volume (up to 4 corners x
     * PEARL_MAX_RETRY_CORNERS attempts x every cell in the grid), not
     * per-call cost. reversePearlSources CANNOT set this: there, the
     * corner is the SOURCE, and tx/ty/tz (this function's target
     * parameter) is the search's current backward-frontier node, whose
     * standability was never verified by pearlCellCorners at all.
     */
    private static PearlArc solvePearlEdge(World world, double sourceX, double sourceY, double sourceZ,
                                            int targetX, int targetY, int targetZ,
                                            boolean targetAlreadyVerifiedStandable) {
        double dx = targetX + 0.5 - sourceX;
        double dz = targetZ + 0.5 - sourceZ;
        double horizDist = Math.sqrt(dx * dx + dz * dz);
        if (horizDist < PEARL_MIN_DISTANCE || horizDist > PEARL_MAX_DISTANCE) {
            return null;
        }
        if (countLosBlockedSamples(sourceX, sourceY, sourceZ, targetX, targetY, targetZ, world) > PEARL_LOS_BLOCK_THRESHOLD) {
            return null;
        }
        if (!targetAlreadyVerifiedStandable && !world.isStandableColumn(targetX, targetY, targetZ)) {
            return null;
        }
        double dy = targetY - sourceY;
        PearlArc arc = solvePearlArc(dx, dy, dz);
        if (arc == null) {
            return null;
        }
        ArcSampleResult samples = sampleArcCollisions(sourceX, sourceY, sourceZ, arc.yaw(), arc.pitch(), arc.ticks(), world);
        if (samples.blockedSolid() > PEARL_ARC_BLOCK_THRESHOLD) {
            return null;
        }
        double cost = arc.cost() + (samples.lavaHits() > 0 ? PEARL_LAVA_TAX : 0.0) + angleTax(arc.pitch());
        return new PearlArc(arc.ticks(), arc.pitch(), arc.yaw(), cost);
    }

    /**
     * PEARL: ender pearl throw with a parabolic (gravity+drag) arc,
     * teleporting to wherever it lands. Iterates a coarse grid of
     * destination cells around (x,y,z), tries up to PEARL_MAX_RETRY_CORNERS
     * standable corners per cell (closest first), and emits the first one
     * that passes solvePearlEdge -- one candidate edge per cell, not one
     * per corner, so the discretization stays clean (matches how
     * PARKOUR_KERNEL contributes one candidate per kernel entry).
     */
    public static void pearlEdges(World world, int x, int y, int z, EdgeConsumer out) {
        double eyeX = x + 0.5, eyeY = y + PEARL_EYE_HEIGHT, eyeZ = z + 0.5;
        int half = PEARL_GRID_RADIUS_CELLS;
        double cellHalfDiag = PEARL_CELL_SIZE * SQRT2 * 0.5;
        for (int cellX = -half; cellX <= half; cellX++) {
            for (int cellZ = -half; cellZ <= half; cellZ++) {
                double centerX = (cellX + 0.5) * PEARL_CELL_SIZE;
                double centerZ = (cellZ + 0.5) * PEARL_CELL_SIZE;
                double centerDist = Math.hypot(centerX, centerZ);
                if (centerDist + cellHalfDiag < PEARL_MIN_DISTANCE || centerDist - cellHalfDiag > PEARL_MAX_DISTANCE) {
                    continue; // whole cell is provably outside range -- skip before resolving any corner
                }
                int cellMinX = x + (int) Math.round(cellX * PEARL_CELL_SIZE);
                int cellMinZ = z + (int) Math.round(cellZ * PEARL_CELL_SIZE);
                java.util.List<PearlCorner> corners = pearlCellCorners(world, cellMinX, cellMinZ, y, x, z);
                int attempts = 0;
                for (PearlCorner corner : corners) {
                    if (attempts >= PEARL_MAX_RETRY_CORNERS) {
                        break;
                    }
                    attempts++;
                    PearlArc arc = solvePearlEdge(world, eyeX, eyeY, eyeZ, corner.x(), corner.y(), corner.z(), true);
                    if (arc != null) {
                        out.accept(StateCodec.pack(corner.x(), corner.y(), corner.z(), false), arc.cost(), Action.PEARL);
                        break; // this cell's candidate is settled -- next cell
                    }
                }
            }
        }
    }

    /**
     * Predecessors of a PEARL landing at (tx,ty,tz) -- NOT separate
     * backward physics. Runs the exact same coarse-grid corner search
     * centered on the TARGET instead of the source, and for each candidate
     * source corner calls solvePearlEdge with the roles swapped (candidate
     * as source, tx/ty/tz as the fixed target) to check it actually
     * reaches the target -- identical reuse-forward-logic pattern as every
     * other reverse probe in this file (reverseNonCrawlEdges/
     * reverseCrawlEdges). Both source-crawling states are valid
     * predecessors (PEARL's physics/cost don't depend on crawling at all),
     * same as SPRINT/MINE/CLIMB's reverse probes.
     */
    public static void reversePearlSources(World world, int tx, int ty, int tz, EdgeConsumer out) {
        int half = PEARL_GRID_RADIUS_CELLS;
        double cellHalfDiag = PEARL_CELL_SIZE * SQRT2 * 0.5;
        for (int cellX = -half; cellX <= half; cellX++) {
            for (int cellZ = -half; cellZ <= half; cellZ++) {
                double centerX = (cellX + 0.5) * PEARL_CELL_SIZE;
                double centerZ = (cellZ + 0.5) * PEARL_CELL_SIZE;
                double centerDist = Math.hypot(centerX, centerZ);
                if (centerDist + cellHalfDiag < PEARL_MIN_DISTANCE || centerDist - cellHalfDiag > PEARL_MAX_DISTANCE) {
                    continue;
                }
                int cellMinX = tx + (int) Math.round(cellX * PEARL_CELL_SIZE);
                int cellMinZ = tz + (int) Math.round(cellZ * PEARL_CELL_SIZE);
                java.util.List<PearlCorner> corners = pearlCellCorners(world, cellMinX, cellMinZ, ty, tx, tz);
                int attempts = 0;
                for (PearlCorner corner : corners) {
                    if (attempts >= PEARL_MAX_RETRY_CORNERS) {
                        break;
                    }
                    attempts++;
                    double sourceEyeX = corner.x() + 0.5, sourceEyeY = corner.y() + PEARL_EYE_HEIGHT, sourceEyeZ = corner.z() + 0.5;
                    PearlArc arc = solvePearlEdge(world, sourceEyeX, sourceEyeY, sourceEyeZ, tx, ty, tz, false);
                    if (arc != null) {
                        for (boolean srcCrawling : new boolean[] {false, true}) {
                            out.accept(StateCodec.pack(corner.x(), corner.y(), corner.z(), srcCrawling), arc.cost(), Action.PEARL);
                        }
                        break;
                    }
                }
            }
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
     * BOAT_CRAWL, FALL. Direct port of dev.mcpathfind.core.EdgeRules'
     * horizontalEdges with the BRIDGE branches (both flat-crossing and
     * rise-above-lava) removed -- everything else, including the
     * head-clearance fix, is unchanged.
     */
    public static void horizontalEdges(World world, int x, int y, int z, boolean crawling,
                                        int dx, int dz, double toolMultiplier, EdgeConsumer out) {
        horizontalEdges(world, x, y, z, crawling, dx, dz, toolMultiplier, null, out);
    }

    /**
     * Same as the 8-arg horizontalEdges, but with an optional AirPotential
     * (null = no adjustment) applied to discount MINE's cost -- see
     * dev.mcpathfind.core.EdgeRules' equivalent overload for the full
     * rationale, unchanged here.
     */
    public static void horizontalEdges(World world, int x, int y, int z, boolean crawling,
                                        int dx, int dz, double toolMultiplier, AirPotential airPotential, EdgeConsumer out) {
        horizontalEdges(world, x, y, z, crawling, dx, dz, toolMultiplier, airPotential, false, null, out);
    }

    /**
     * Same as the AirPotential overload, but adds an ALTERNATE, independent
     * MINE prune: manhattanPruneEnabled=true skips generating a MINE
     * candidate if it doesn't strictly decrease Manhattan distance to the
     * nearest goal point. See dev.mcpathfind.core.EdgeRules' equivalent
     * overload for the full rationale, unchanged here. goal may be null iff
     * manhattanPruneEnabled is false.
     */
    public static void horizontalEdges(World world, int x, int y, int z, boolean crawling,
                                        int dx, int dz, double toolMultiplier, AirPotential airPotential,
                                        boolean manhattanPruneEnabled, GoalPoints goal, EdgeConsumer out) {
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
                // anywhere useful -- see AirPotential.shouldPruneMine's javadoc.
            } else if (manhattanPruneEnabled && !mineMovesCloserToGoal(x, y, z, nx, nz, goal)) {
                // alternate, independent prune -- see mineMovesCloserToGoal's javadoc.
            } else {
                // See dev.mcpathfind.core.EdgeRules' equivalent branch for the
                // full max()/insta-mine/MINE_DURABILITY_TAX rationale, unchanged here.
                double mineCost = BlockType.mineTime(target) * toolMultiplier;
                if (BlockType.isSolid(head)) {
                    mineCost += BlockType.mineTime(head) * toolMultiplier;
                }
                double cost = Math.max(mineCost, dist / SPRINT_SPEED) + MINE_DURABILITY_TAX;
                out.accept(StateCodec.pack(nx, y, nz, false), cost, Action.MINE);
            }
            if (BlockType.isSolid(head) && !BlockType.isUnbreakable(target)) {
                double crawlMineCost = BlockType.mineTime(target) * toolMultiplier;
                double startupTax = crawling ? 0.0 : BOAT_CRAWL_TAX;
                double cost = Math.max(crawlMineCost, dist / BOAT_CRAWL_SPEED)
                    + startupTax + MINE_DURABILITY_TAX;
                out.accept(StateCodec.pack(nx, y, nz, true), cost, Action.BOAT_CRAWL);
            }
            return;
        }

        boolean targetOrHeadIsLava = BlockType.isLava(target) || BlockType.isLava(head);
        boolean targetOrHeadBlocked = targetOrHeadIsLava || BlockType.isSolid(head);

        if (BlockType.isSolid(belowTarget) && !targetOrHeadBlocked) {
            double cost = dist / SPRINT_SPEED;
            out.accept(StateCodec.pack(nx, y, nz, false), cost, Action.SPRINT);
            return;
        }

        if (!BlockType.isSolid(belowTarget) && !targetOrHeadBlocked) {
            LandingResult landing = findLandingY(world, nx, y - 1, nz);
            if (landing.found()) {
                double fallCost = dist / SPRINT_SPEED + landing.drop() * 0.05;
                if (landing.drop() > CLUTCH_THRESHOLD) {
                    fallCost += CLUTCH_SETUP_TIME;
                }
                if (landing.passedLava()) {
                    fallCost += LAVA_DEATH_PENALTY;
                }
                out.accept(StateCodec.pack(nx, landing.landingY(), nz, false), fallCost, Action.FALL);
            }
            return;
        }
        // targetOrHeadIsLava with no clear climb-above-lava alternative here
        // (that was BRIDGE's rise-above-lava branch, dropped along with the
        // rest of BRIDGE) -- simply no edge in that case.
    }

    /** CLIMB (step-up) for one direction from (x,y,z). */
    public static void climbEdge(World world, int x, int y, int z, int dx, int dz, EdgeConsumer out) {
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
            out.accept(StateCodec.pack(nx, y + 1, nz, false), cost, Action.CLIMB);
        }
    }

    /** Vertical mining straight down from (x,y,z). */
    public static void mineDownEdge(World world, int x, int y, int z, double toolMultiplier, EdgeConsumer out) {
        int belowHere = world.voxelAt(x, y - 1, z);
        if (BlockType.isSolid(belowHere) && !BlockType.isUnbreakable(belowHere)) {
            double cost = BlockType.mineTime(belowHere) * toolMultiplier + 0.2;
            out.accept(StateCodec.pack(x, y - 1, z, false), cost, Action.MINE_DOWN);
        }
    }

    // ============================================================
    // Reverse (predecessor) edge generation, for bidirectional search's
    // backward half -- see dev.mcpathfind.core.EdgeRules' equivalent
    // section for the full "reuse forward logic" rationale, unchanged here.
    // ============================================================

    /**
     * A synthesized backward-frontier source is valid iff it's genuinely
     * resting on solid ground. Simpler than dev.mcpathfind.core.EdgeRules'
     * equivalent -- that version also accepts BRIDGE-plausible floating
     * positions (no solid floor, but a clear body), which don't exist here
     * since BRIDGE is dropped: nothing in this action set ever creates a
     * no-solid-floor resting position, so there's nothing to accept beyond
     * "is there solid ground beneath."
     */
    static boolean isValidRestingSource(World world, int x, int y, int z, boolean crawling) {
        if (crawling) {
            return true;
        }
        return BlockType.isSolid(world.voxelAt(x, y - 1, z));
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
        out.accept(StateCodec.pack(sourceX, sourceY, sourceZ, sourceCrawling), cost, action);
    }

    /**
     * Predecessors of a non-crawling target (tx,ty,tz): SPRINT, MINE, CLIMB,
     * MINE_DOWN, FALL, PEARL. (BOAT_CRAWL never lands with crawling=false,
     * so it's never a predecessor here -- see reverseCrawlEdges.)
     *
     * Every one of SPRINT/MINE/CLIMB/FALL/MINE_DOWN is ELIGIBLE AND COSTS
     * THE SAME regardless of whether the source was already crawling --
     * only BOAT_CRAWL's own cost formula reads the source's crawling flag
     * (the startup-tax waiver). So every one of these reverse probes tries
     * BOTH srcCrawling=false and true -- see
     * dev.mcpathfind.core.EdgeRules.reverseNonCrawlEdges' javadoc for the
     * confirmed-not-theoretical bug this guards against.
     *
     * airPotential (nullable, null = no prune) is threaded through to the
     * MINE reverse-probe ONLY, using the exact same forward horizontalEdges
     * call forward search itself would make -- see the core version's
     * javadoc for the full rationale, unchanged here.
     */
    private static final boolean[] SRC_CRAWLING_VALUES = {false, true};

    public static void reverseNonCrawlEdges(World world, int tx, int ty, int tz, double toolMultiplier,
                                             AirPotential airPotential, EdgeConsumer out) {
        for (int d = 0; d < 8; d++) {
            int dx = DIR_DX[d], dz = DIR_DZ[d];
            int sx = tx - dx, sz = tz - dz;
            if (sx < 0 || sx >= world.sizeX || sz < 0 || sz >= world.sizeZ) {
                continue;
            }

            for (boolean srcCrawling : SRC_CRAWLING_VALUES) {
                horizontalEdges(world, sx, ty, sz, srcCrawling, dx, dz, toolMultiplier,
                        (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.SPRINT, tx, ty, tz, sx, ty, sz, srcCrawling, out));
                horizontalEdges(world, sx, ty, sz, srcCrawling, dx, dz, toolMultiplier, airPotential,
                        (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.MINE, tx, ty, tz, sx, ty, sz, srcCrawling, out));
            }

            int climbSy = ty - 1;
            if (climbSy >= 0) {
                for (boolean srcCrawling : SRC_CRAWLING_VALUES) {
                    climbEdge(world, sx, climbSy, sz, dx, dz,
                            (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.CLIMB, tx, ty, tz, sx, climbSy, sz, srcCrawling, out));
                }
            }

            if (BlockType.isSolid(world.voxelAt(tx, ty - 1, tz))) {
                reverseFallSources(world, tx, ty, tz, dx, dz, sx, sz, out);
            }
        }

        reversePearlSources(world, tx, ty, tz, out);

        int mdSy = ty + 1;
        if (mdSy < world.sizeY) {
            for (boolean srcCrawling : SRC_CRAWLING_VALUES) {
                mineDownEdge(world, tx, mdSy, tz, toolMultiplier,
                        (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.MINE_DOWN, tx, ty, tz, tx, mdSy, tz, srcCrawling, out));
            }
        }
    }

    /**
     * Predecessors of a crawling target (tx,ty,tz): BOAT_CRAWL only (it's
     * the only action that ever produces crawling=true).
     */
    public static void reverseCrawlEdges(World world, int tx, int ty, int tz, double toolMultiplier, EdgeConsumer out) {
        for (int d = 0; d < 8; d++) {
            int dx = DIR_DX[d], dz = DIR_DZ[d];
            int sx = tx - dx, sz = tz - dz;
            if (sx < 0 || sx >= world.sizeX || sz < 0 || sz >= world.sizeZ) {
                continue;
            }
            for (boolean srcCrawling : SRC_CRAWLING_VALUES) {
                horizontalEdges(world, sx, ty, sz, srcCrawling, dx, dz, toolMultiplier,
                        (toState, cost, action) -> filterMatch(world, toState, cost, action, Action.BOAT_CRAWL, tx, ty, tz, sx, ty, sz, srcCrawling, out));
            }
        }
    }

    /**
     * Reverse of the FALL branch in horizontalEdges -- unchanged from
     * dev.mcpathfind.core.EdgeRules.reverseFallSources (including the
     * diagonalCornerBlocked guard it took a real bug on k2_r_0_0 to add;
     * see its javadoc there).
     */
    private static void reverseFallSources(World world, int tx, int ty, int tz, int dx, int dz, int sx, int sz, EdgeConsumer out) {
        double dist = diagDist(dx, dz);
        int landingCell = world.voxelAt(tx, ty, tz);
        int landingHead = world.voxelAt(tx, ty + 1, tz);
        boolean landingIsLava = BlockType.isLava(landingCell) || BlockType.isLava(landingHead);

        boolean passedLava = false;
        for (int dropAmount = 1; dropAmount <= MAX_REVERSE_FALL_SCAN; dropAmount++) {
            int y = ty + dropAmount;
            if (y >= world.sizeY) {
                break;
            }
            int belowTarget = world.voxelAt(tx, y - 1, tz);
            if (BlockType.isSolid(belowTarget)) {
                break;
            }
            int target = world.voxelAt(tx, y, tz);
            int head = world.voxelAt(tx, y + 1, tz);
            if (BlockType.isSolid(target) || BlockType.isSolid(head)) {
                break;
            }
            if (BlockType.isLava(belowTarget)) {
                passedLava = true;
            }
            boolean thisPassedLava = passedLava || landingIsLava;

            if (!BlockType.isSolid(world.voxelAt(sx, y - 1, sz))) {
                continue;
            }
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
            for (boolean srcCrawling : SRC_CRAWLING_VALUES) {
                out.accept(StateCodec.pack(sx, y, sz, srcCrawling), fallCost, Action.FALL);
            }
        }
    }

    private EdgeRules() {}
}
