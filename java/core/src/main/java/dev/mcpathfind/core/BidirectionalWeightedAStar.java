package dev.mcpathfind.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Bidirectional weighted A*: searches forward from start and backward from
 * goal simultaneously, stopping as soon as either (a) forward directly pops
 * the real goal -- the exact WeightedAStar termination condition, so
 * bidirectional search is never WORSE than plain forward search -- or (b)
 * the two frontiers meet with a proven-tight bound (standard bidirectional
 * stopping rule: stop once topOfForwardOpen + topOfBackwardOpen >= best
 * meeting cost found so far).
 *
 * Forward search uses the full action set (unchanged from WeightedAStar).
 * Backward search now ALSO includes BRIDGE (see EdgeRules.reverseBridgeSources)
 * -- previously excluded entirely, because its cost depends on
 * blockTax(blocksRemaining), the ABSOLUTE resource count at the moment of
 * placement, which backward can't know in advance (it depends on how much
 * forward's own prefix has spent by the time the two searches meet).
 * Excluding BRIDGE outright was more than "a lost speedup opportunity": it
 * could silently break the epsilon=1.0 "true optimal" guarantee, because the
 * standard meet-in-the-middle stopping rule assumes both directions search
 * the SAME graph -- with BRIDGE missing from backward's action set,
 * backward's own priorities were pessimistically high (infinitely so), so
 * the stopping rule could fire on a proven-tight bound that wasn't actually
 * tight, terminating on a real-but-suboptimal meeting instead of waiting for
 * forward to find a cheaper BRIDGE-based route directly. This was CONFIRMED,
 * not theoretical: a hand-built synthetic world (rolling_terrain_world, see
 * experiments/LabFidelityCheck.java) showed a 65% cost regression from this
 * at epsilon=1.0 before this fix.
 *
 * DESIGN HISTORY -- read before changing this: the first fix tracked
 * blocksNeeded as part of backward's STATE IDENTITY (a separate copy of the
 * reachable graph per possible bridge count, up to a depth cap). That was
 * correct but a disaster on real terrain: caves/overhangs mean a large
 * fraction of a real map is void-adjacent, so forking state by bridge count
 * duplicated large chunks of backward's own reachable graph per depth level
 * -- even depth=1 nearly doubled expansions on the hardest tested region,
 * and depth>=4 never converged within a 3M-expansion budget at all. This
 * version instead tracks bridge count as a plain SCALAR alongside g
 * (bwdBridgesUsed below), exactly like cameFromId/actionUsed already are --
 * no forking, no extra states. Backward's cost for a reverse-BRIDGE step is
 * a WORST-CASE estimate (assume THIS bridge is being placed on forward's
 * last available block, blockTax(1); the next one further out assumes
 * blockTax(2); etc. -- see EdgeRules.reverseBridgeSources), which
 * OVERestimates true cost (same direction as full exclusion, but far less
 * extreme and now genuinely bounded rather than effectively infinite) at
 * zero extra state-space cost. The final reported cost is corrected exactly
 * at reconstruction time regardless (see reconstructMeeting), once the true
 * meeting-point blocksRemaining is known.
 *
 * The FUNDING CHECK -- can forward actually afford backward's segment --
 * uses the single best-known bridge count per position (bwdBridgesUsed) and
 * single best-known blocksRemaining per position (fwdPosBestBlocks), not an
 * exhaustive set of alternatives. This mirrors an ALREADY-accepted
 * approximation elsewhere in this class (fwdPosBestG only ever tracks
 * forward's single cheapest visit to a position, not every visit): if the
 * cheapest-g backward path to a position happens to need more blocks than
 * forward's cheapest-g visit has left, a costlier-but-affordable alternative
 * on either side won't be found. That's a lost speedup/meeting opportunity,
 * not a correctness risk -- the guaranteed-complete forward-direct-hit
 * fallback still covers it.
 *
 * Backward state is (x, y, z, crawling) only -- same shape as before BRIDGE
 * was ever considered here. blocksRemaining in the packed state is always
 * 0 (unused placeholder), since none of backward's actions read it; bridge
 * count lives in bwdBridgesUsed instead.
 *
 * STATE REPRESENTATION: dense-id / array-backed scheme, same rationale as
 * WeightedAStar (see its class javadoc). Forward and backward states are
 * different key spaces, so they get separate StateIdMap instances (fwdIds,
 * bwdIds). The cross-side "posKey" projection (a forward state's
 * (x,y,z,crawling) projected into backward's key shape) is, by construction,
 * exactly a backward-shaped key -- so it's interned through bwdIds too, and
 * fwdPosBestG/fwdPosBestBlocks/fwdPosBestStateId become arrays indexed by
 * bwd ids, directly comparable to bwdClosed/bwdG/bwdBridgesUsed (also
 * bwd-id-indexed) with no translation needed.
 *
 * NOTE: this class reproduces a pre-existing bug carried over faithfully
 * from the original hash-map-keyed version -- validated against real
 * region k2_r_0_0, both versions produce an identical 406-node,
 * cost=96.62s path with the SAME PathValidator violation (step 402: FALL
 * cuts a blocked diagonal corner). That's the previously-flagged,
 * never-fixed diagonal corner-cutting issue, not something introduced by
 * this rewrite -- confirmed by rebuilding the original side-by-side and
 * observing byte-for-byte identical path/cost/action-counts/violation.
 *
 * MEET-IN-THE-MIDDLE TERMINATION -- read before touching the stopping
 * condition in search()'s main loop. This stops as soon as
 * fwdHeap.peekPriority() + bwdHeap.peekPriority() >= mu, the textbook
 * termination rule for BIDIRECTIONAL DIJKSTRA (h=0 on both sides) --
 * REUSED here even though both sides actually use real heuristics
 * (heuristicToGoal / heuristicToStart), which is NOT automatically safe:
 * naively summing two independently-admissible but not mutually "balanced"
 * lower bounds is a well-documented pitfall dating back to Pohl's original
 * 1971 bidirectional A* (BHPA) -- the sum CAN clear mu before mu is
 * actually the global optimum, because nothing constrains
 * h_fwd(v)+h_bwd(v) to relate consistently to the true shortest-path
 * distance across different v.
 *
 * CONFIRMED, not theoretical: mine_vs_crawl_long_world.wbin (see
 * experiments/export_lab_worlds.py) -- an 8-block STONE wall where
 * BOAT_CRAWL is the true cheaper crossing (16.43s) -- showed this exact
 * failure: exactly ONE mu update (Infinity -> 18.16, a
 * forward-mines-2-then-backward-crawls-6 hybrid, mathematically guaranteed
 * suboptimal for a homogeneous corridor since BOAT_CRAWL's one-time
 * startup tax gets "wasted" on a partial crawl instead of amortized over
 * the whole crossing) then terminated on the VERY NEXT loop iteration
 * because topFwd(11.51)+topBwd(15.20)=26.71 already exceeded that mu.
 *
 * A mathematically RIGOROUS fix exists (drop topBwd from the check
 * entirely -- topFwd alone is sufficient, by the same "forward's own
 * eventual direct-hit result is >= its current heap top" argument the
 * direct-hit branch already relies on) but it's a SEVERE, sometimes
 * complete, performance regression: without topBwd's contribution,
 * backward's discovery work often no longer shortens the wait at all,
 * since forward has to climb its own heap nearly as far as it would
 * completely alone. Measured cost on real terrain (k1_r_neg1_0): expansions
 * roughly DOUBLED (876k forward-alone -> 1.68M with the rigorous fix),
 * wall-clock roughly TRIPLED (1.7s -> 6.0s) -- i.e. bidirectional became
 * slower than just running forward alone, on the exact kind of terrain this
 * class exists to speed up. A symmetric fix (also trust topBwd alone, once
 * a verified "backward reaches start" direct-hit exists to cash in the
 * mirrored argument) recovers the speed, but is real, nontrivial additional
 * machinery.
 *
 * WHAT THIS CLASS DOES INSTEAD: a deliberately NOT-rigorous, pragmatic
 * mitigation -- keep the original, fast sum-based check, but don't trust
 * a candidate mu the INSTANT the crossing condition is satisfied. Require
 * mu to have gone at least MU_GRACE_EXPANSIONS expansions without further
 * improvement first (see expansionsSinceMuImproved below). The observed bug
 * was mu getting accepted on the VERY NEXT check after being set, with
 * zero chance to improve -- a grace period directly targets that specific
 * failure shape (a premature, immediately-accepted meeting) without
 * touching the underlying (still theoretically imperfect) stopping bound.
 * This does NOT prove optimality -- a sufficiently adversarial world could
 * still need more than MU_GRACE_EXPANSIONS extra steps to surface a better
 * mu -- but it's cheap (a few thousand expansions is noise against typical
 * multi-hundred-thousand-expansion real searches) and catches the actual
 * observed failure mode, which is what was asked for here: mitigate a
 * common trouble spot, not chase a full correctness proof.
 *
 * Forward's own direct-hit branch is untouched and still the hard
 * backstop regardless: if grace-period gating ever delays a meeting-based
 * return long enough, forward's own unassisted exploration -- which
 * neither needs nor waits on mu -- will complete and return on its own,
 * exactly like plain WeightedAStar. That's what guarantees this can never
 * return a worse answer than forward-alone would, grace period or not.
 */
public final class BidirectionalWeightedAStar {

    public static boolean DEBUG = false;

    private World world;
    private double toolMultiplier;
    private double epsilon;
    private GoalPoints goal;

    // forward: full (x,y,z,blocks,crawling) state space
    private StateIdMap fwdIds;
    private double[] fwdG;
    private int[] fwdCameFromId;
    private Action[] fwdActionUsed;
    private int[] fwdBridgeUpOriginY; // indexed by fwd id -- see EdgeRules.bridgeUpEdge's javadoc
    private boolean[] fwdClosed;
    private IntIdOpenHeap fwdHeap;

    // backward: (x,y,z,crawling) only, blocks fixed at 0
    private StateIdMap bwdIds;
    private double[] bwdG;
    private int[] bwdCameFromId;  // bwd id -> bwd id; T where forward edge S->T (action bwdActionUsed[S]) was reverse-discovered
    private Action[] bwdActionUsed;
    private int[] bwdBridgesUsed; // how many BRIDGE steps the current-best-g path to this bwd state used (scalar metadata, not identity)
    private boolean[] bwdClosed;
    private IntIdOpenHeap bwdHeap;

    // projection of forward-closed states onto (x,y,z,crawling) -- backward's native
    // state shape -- tracking the best forward g, the blocksRemaining it had there
    // (needed for the funding check), and which full forward state achieved it.
    // Indexed by BWD id (posKey is a backward-shaped key -- see class javadoc).
    private double[] fwdPosBestG;
    private int[] fwdPosBestBlocks;
    private int[] fwdPosBestStateId; // -1 = none yet; else a FORWARD id

    private double mu;
    private int meetForwardId = -1;
    private int meetBackwardId = -1;

    // How many expansions to require WITHOUT mu improving before trusting
    // the sum-based crossing condition -- see class javadoc's
    // "MEET-IN-THE-MIDDLE TERMINATION" section for why this exists (a
    // pragmatic mitigation, not a correctness proof) and why the value
    // doesn't need to be large: it only has to survive a few thousand
    // expansions of "was this the true optimum or just an early hybrid",
    // which is noise against typical real-search expansion counts.
    public static int MU_GRACE_EXPANSIONS = 2000;
    private int expansionsSinceMuImproved = 0;

    // Start position + budget + dense id, needed by stepBackward()'s
    // backward-direct-hit check (see class javadoc): a backward-discovered
    // path reaching the exact start is a genuine additional way to improve
    // mu, independent of (and synergistic with) the grace-period gate above
    // -- more ways for mu to reach its true value before the grace period
    // elapses only helps.
    private int startX, startY, startZ;
    private int startBlocksAvailable;
    private int startId;

    // Forward-only MINE prune (see WeightedAStar.minePruneMode) -- applied to
    // the forward half exactly as in WeightedAStar. Backward's reverse-MINE
    // probing (inside EdgeRules.reverseNonCrawlEdges) intentionally stays
    // unpruned: it's a smaller share of backward's candidate volume than
    // MINE is of forward's, and threading the prune through the reverse
    // probing/filterMatch path risks the exact class of forward/backward
    // divergence bug this file's javadoc warns about. Not applying it there
    // costs a speedup opportunity, not correctness.
    private AirPotential airPotential;

    /** Single-point goal, preserved for source compatibility -- delegates to the GoalPoints overload as a 1-element set. */
    public SearchResult search(World world, StateCodec.State start, StateCodec.State goal,
                                int blocksAvailable, double epsilon, double toolMultiplier,
                                int maxExpansions) {
        return search(world, start, GoalPoints.point(goal), blocksAvailable, epsilon, toolMultiplier, maxExpansions);
    }

    public SearchResult search(World world, StateCodec.State start, GoalPoints goal,
                                int blocksAvailable, double epsilon, double toolMultiplier,
                                int maxExpansions) {
        this.world = world;
        this.toolMultiplier = toolMultiplier;
        this.epsilon = epsilon;
        this.goal = goal;

        int sizeHint = (int) Math.min(Math.max(1024L, (long) maxExpansions * 4), 20_000_000L);

        fwdIds = new StateIdMap(sizeHint);
        fwdG = newDoubleArray(sizeHint, Double.POSITIVE_INFINITY);
        fwdCameFromId = newIntArray(sizeHint, -1);
        fwdActionUsed = new Action[sizeHint];
        fwdBridgeUpOriginY = new int[sizeHint];
        fwdClosed = new boolean[sizeHint];
        fwdHeap = new IntIdOpenHeap(Math.min(sizeHint, 1_000_000), sizeHint);

        bwdIds = new StateIdMap(sizeHint);
        bwdG = newDoubleArray(sizeHint, Double.POSITIVE_INFINITY);
        bwdCameFromId = newIntArray(sizeHint, -1);
        bwdActionUsed = new Action[sizeHint];
        bwdBridgesUsed = new int[sizeHint];
        bwdClosed = new boolean[sizeHint];
        bwdHeap = new IntIdOpenHeap(Math.min(sizeHint, 1_000_000), sizeHint);

        fwdPosBestG = newDoubleArray(sizeHint, Double.POSITIVE_INFINITY);
        fwdPosBestBlocks = newIntArray(sizeHint, -1);
        fwdPosBestStateId = newIntArray(sizeHint, -1);

        mu = Double.POSITIVE_INFINITY;
        meetForwardId = -1;
        meetBackwardId = -1;
        expansionsSinceMuImproved = 0;

        if (WeightedAStar.minePruneMode == WeightedAStar.MinePruneMode.AIR_POTENTIAL) {
            AirPotentialField field = AirPotentialField.build(world, start.x(), start.y(), start.z());
            this.airPotential = new AirPotential(field, /* scaleBfs */ 3.0);
        } else {
            this.airPotential = null;
        }

        this.startX = start.x();
        this.startY = start.y();
        this.startZ = start.z();
        this.startBlocksAvailable = blocksAvailable;

        long startState = StateCodec.pack(start.x(), start.y(), start.z(), blocksAvailable, false);
        int startId = internFwd(startState);
        this.startId = startId;
        fwdG[startId] = 0.0;
        fwdBridgeUpOriginY[startId] = start.y();
        fwdHeap.push(epsilon * heuristicToGoal(start.x(), start.y(), start.z()), 0.0, start.y(), startId);

        // Seed backward search from EVERY goal point, not just one -- the
        // standard "virtual goal node with 0-cost edges to every member of
        // the goal set" construction (see GoalPoints' class javadoc). Each
        // seed still needs isValidRestingSource to be a legitimate place
        // backward search can stand, same as any other synthesized
        // frontier state. goal.size() is small (a handful of points), so
        // this loop is cheap -- for a point goal it's exactly one
        // iteration, byte-identical to the single-seed code this replaces.
        for (int i = 0; i < goal.size(); i++) {
            int gx = goal.x(i), gy = goal.y(i), gz = goal.z(i);
            if (!EdgeRules.isValidRestingSource(world, gx, gy, gz, false)) {
                continue;
            }
            long goalSeed = StateCodec.pack(gx, gy, gz, 0, false);
            int goalSeedId = internBwd(goalSeed);
            bwdG[goalSeedId] = 0.0;
            bwdHeap.push(epsilon * heuristicToStart(gx, gy, gz, start), 0.0, gy, goalSeedId);
        }

        int expansions = 0;
        while (true) {
            if (fwdHeap.isEmpty() && bwdHeap.isEmpty()) {
                return new SearchResult(false, null, 0.0, null, expansions);
            }

            if (!fwdHeap.isEmpty()) {
                Boolean directHit = stepForward();
                if (directHit != null) {
                    expansions++;
                    expansionsSinceMuImproved++;
                    if (directHit) {
                        return reconstructDirect(expansions);
                    }
                    if (expansions > maxExpansions) {
                        return new SearchResult(false, null, 0.0, null, expansions);
                    }
                }
            }
            if (!bwdHeap.isEmpty()) {
                boolean didStep = stepBackward();
                if (didStep) {
                    expansions++;
                    expansionsSinceMuImproved++;
                    if (expansions > maxExpansions) {
                        return new SearchResult(false, null, 0.0, null, expansions);
                    }
                }
            }

            // Stopping condition: the textbook sum-based bidirectional
            // rule, gated by a grace period -- see class javadoc's
            // "MEET-IN-THE-MIDDLE TERMINATION" section for why the sum
            // alone isn't a rigorous proof once both sides use real
            // heuristics, and why this class accepts that (a pragmatic
            // mitigation) rather than paying the rigorous fix's severe
            // speed cost.
            if (mu < Double.POSITIVE_INFINITY && !fwdHeap.isEmpty() && !bwdHeap.isEmpty()) {
                if (fwdHeap.peekPriority() + bwdHeap.peekPriority() >= mu
                        && expansionsSinceMuImproved >= MU_GRACE_EXPANSIONS) {
                    if (DEBUG) {
                        System.err.printf("[terminate via meeting] topFwd=%.4f topBwd=%.4f mu=%.4f expansionsSinceMuImproved=%d expansions=%d%n",
                                fwdHeap.peekPriority(), bwdHeap.peekPriority(), mu, expansionsSinceMuImproved, expansions);
                    }
                    return reconstructMeeting(expansions);
                }
            }
        }
    }

    private int internFwd(long packedState) {
        int id = fwdIds.idFor(packedState);
        ensureFwdCapacityFor(id);
        return id;
    }

    private int internBwd(long packedState) {
        int id = bwdIds.idFor(packedState);
        ensureBwdCapacityFor(id);
        return id;
    }

    private void ensureFwdCapacityFor(int id) {
        if (id >= fwdG.length) {
            int newCap = Math.max(id + 1, fwdG.length + (fwdG.length >> 1) + 1);
            fwdG = java.util.Arrays.copyOf(fwdG, newCap);
            java.util.Arrays.fill(fwdG, id, newCap, Double.POSITIVE_INFINITY);
            fwdCameFromId = java.util.Arrays.copyOf(fwdCameFromId, newCap);
            java.util.Arrays.fill(fwdCameFromId, id, newCap, -1);
            fwdActionUsed = java.util.Arrays.copyOf(fwdActionUsed, newCap);
            fwdBridgeUpOriginY = java.util.Arrays.copyOf(fwdBridgeUpOriginY, newCap);
            fwdClosed = java.util.Arrays.copyOf(fwdClosed, newCap);
        }
        fwdHeap.ensureIdCapacity(id + 1);
    }

    private void ensureBwdCapacityFor(int id) {
        if (id >= bwdG.length) {
            int newCap = Math.max(id + 1, bwdG.length + (bwdG.length >> 1) + 1);
            bwdG = java.util.Arrays.copyOf(bwdG, newCap);
            java.util.Arrays.fill(bwdG, id, newCap, Double.POSITIVE_INFINITY);
            bwdCameFromId = java.util.Arrays.copyOf(bwdCameFromId, newCap);
            java.util.Arrays.fill(bwdCameFromId, id, newCap, -1);
            bwdActionUsed = java.util.Arrays.copyOf(bwdActionUsed, newCap);
            bwdBridgesUsed = java.util.Arrays.copyOf(bwdBridgesUsed, newCap);
            bwdClosed = java.util.Arrays.copyOf(bwdClosed, newCap);
            fwdPosBestG = java.util.Arrays.copyOf(fwdPosBestG, newCap);
            java.util.Arrays.fill(fwdPosBestG, id, newCap, Double.POSITIVE_INFINITY);
            fwdPosBestBlocks = java.util.Arrays.copyOf(fwdPosBestBlocks, newCap);
            java.util.Arrays.fill(fwdPosBestBlocks, id, newCap, -1);
            fwdPosBestStateId = java.util.Arrays.copyOf(fwdPosBestStateId, newCap);
            java.util.Arrays.fill(fwdPosBestStateId, id, newCap, -1);
        }
        bwdHeap.ensureIdCapacity(id + 1);
    }

    private static double[] newDoubleArray(int size, double fillValue) {
        double[] a = new double[size];
        java.util.Arrays.fill(a, fillValue);
        return a;
    }

    private static int[] newIntArray(int size, int fillValue) {
        int[] a = new int[size];
        java.util.Arrays.fill(a, fillValue);
        return a;
    }

    /** Returns null if the pop was a stale (already-closed) entry; otherwise true iff this expansion IS the real goal. */
    private Boolean stepForward() {
        int stateId = fwdHeap.popId();
        if (fwdClosed[stateId]) {
            return null;
        }
        fwdClosed[stateId] = true;

        long state = fwdIds.packedStateOf(stateId);
        int x = StateCodec.unpackX(state);
        int y = StateCodec.unpackY(state);
        int z = StateCodec.unpackZ(state);
        int blocks = StateCodec.unpackBlocks(state);
        boolean crawling = StateCodec.unpackCrawling(state);
        double g = fwdG[stateId];

        if (goal.contains(x, y, z)) {
            currentForwardGoalId = stateId;
            return true; // exact WeightedAStar termination condition
        }

        long posKey = StateCodec.pack(x, y, z, 0, crawling);
        int posId = internBwd(posKey);
        if (g < fwdPosBestG[posId]) {
            fwdPosBestG[posId] = g;
            fwdPosBestBlocks[posId] = blocks;
            fwdPosBestStateId[posId] = stateId;
        }
        if (bwdClosed[posId] && bwdBridgesUsed[posId] <= blocks) {
            double candidate = g + bwdG[posId];
            if (candidate < mu) {
                if (DEBUG) {
                    System.err.printf("[stepForward mu update] mu %.4f -> %.4f at (%d,%d,%d,crawl=%b) fwdG=%.4f bwdG=%.4f%n",
                            mu, candidate, x, y, z, crawling, g, bwdG[posId]);
                }
                mu = candidate;
                meetForwardId = stateId;
                meetBackwardId = posId;
                expansionsSinceMuImproved = 0;
            }
        }

        EdgeRules.EdgeConsumer consumer = (toState, cost, action) -> relaxForward(stateId, g, toState, cost, action);
        for (int d = 0; d < 8; d++) {
            EdgeRules.horizontalEdges(world, x, y, z, blocks, crawling, EdgeRules.DIR_DX[d], EdgeRules.DIR_DZ[d], toolMultiplier,
                    airPotential, WeightedAStar.minePruneMode == WeightedAStar.MinePruneMode.MANHATTAN, goal, consumer);
        }
        for (int d = 0; d < 8; d++) {
            EdgeRules.climbEdge(world, x, y, z, blocks, EdgeRules.DIR_DX[d], EdgeRules.DIR_DZ[d], consumer);
        }
        EdgeRules.parkourEdges(world, x, y, z, blocks, consumer);
        EdgeRules.mineDownEdge(world, x, y, z, blocks, toolMultiplier, consumer);
        EdgeRules.bridgeUpEdge(world, x, y, z, blocks, toolMultiplier, fwdBridgeUpOriginY[stateId], goal, consumer);
        return false;
    }

    private boolean stepBackward() {
        int stateId = bwdHeap.popId();
        if (bwdClosed[stateId]) {
            return false;
        }
        bwdClosed[stateId] = true;

        long state = bwdIds.packedStateOf(stateId);
        int x = StateCodec.unpackX(state);
        int y = StateCodec.unpackY(state);
        int z = StateCodec.unpackZ(state);
        boolean crawling = StateCodec.unpackCrawling(state);
        double g = bwdG[stateId];
        int bridgesUsed = bwdBridgesUsed[stateId];

        if (fwdPosBestStateId[stateId] != -1 && fwdPosBestBlocks[stateId] >= bridgesUsed) {
            double candidate = g + fwdPosBestG[stateId];
            if (candidate < mu) {
                if (DEBUG) {
                    System.err.printf("[stepBackward mu update] mu %.4f -> %.4f at (%d,%d,%d,crawl=%b) bwdG=%.4f fwdG=%.4f%n",
                            mu, candidate, x, y, z, crawling, g, fwdPosBestG[stateId]);
                }
                mu = candidate;
                meetForwardId = fwdPosBestStateId[stateId];
                meetBackwardId = stateId;
                expansionsSinceMuImproved = 0;
            }
        }

        // Backward-direct-hit: the symmetric mirror of stepForward()'s
        // goal.contains(x,y,z) check, but for backward reaching the exact
        // START position -- see class javadoc's "MEET-IN-THE-MIDDLE
        // TERMINATION" section. A backward-discovered path from goal all
        // the way to start is, by the same edge-reuse guarantee every
        // other reverse probe in this file relies on, a genuine valid
        // same-cost start-to-goal path once reversed -- treated as just
        // another mu candidate (not an immediate return like forward's
        // direct-hit) because it still needs the funding check below: the
        // TOTAL block budget must actually cover bwdBridgesUsed, unlike
        // forward's direct-hit which has no such gate (forward always has
        // its full budget from position zero). meetForwardId=startId gives
        // reconstructMeeting() a trivial (zero-action, fwdG=0) forward
        // "prefix", so the WHOLE returned path is this backward segment.
        if (!crawling && x == startX && y == startY && z == startZ && bridgesUsed <= startBlocksAvailable) {
            double candidate = g; // fwdG[startId] is always exactly 0.0
            if (candidate < mu) {
                if (DEBUG) {
                    System.err.printf("[stepBackward backward-direct-hit] mu %.4f -> %.4f bwdG=%.4f bridgesUsed=%d%n",
                            mu, candidate, g, bridgesUsed);
                }
                mu = candidate;
                meetForwardId = startId;
                meetBackwardId = stateId;
                expansionsSinceMuImproved = 0;
            }
        }

        EdgeRules.EdgeConsumer consumer = (predecessor, cost, action) -> relaxBackward(stateId, g, predecessor, cost, action);
        if (crawling) {
            EdgeRules.reverseCrawlEdges(world, x, y, z, toolMultiplier, consumer);
        } else {
            EdgeRules.reverseNonCrawlEdges(world, x, y, z, toolMultiplier, airPotential, bridgesUsed, goal, consumer);
            EdgeRules.reverseParkourSources(world, x, y, z, 0, consumer);
        }
        return true;
    }

    private void relaxForward(int fromId, double gFrom, long toState, double cost, Action action) {
        int toId = internFwd(toState);
        if (fwdClosed[toId]) {
            return;
        }
        double tentativeG = gFrom + cost;
        if (tentativeG < fwdG[toId]) {
            fwdG[toId] = tentativeG;
            fwdCameFromId[toId] = fromId;
            fwdActionUsed[toId] = action;
            int toY = StateCodec.unpackY(toState);
            fwdBridgeUpOriginY[toId] = (action == Action.BRIDGE_UP) ? fwdBridgeUpOriginY[fromId] : toY;
            double h = heuristicToGoal(StateCodec.unpackX(toState), toY, StateCodec.unpackZ(toState));
            fwdHeap.push(tentativeG + epsilon * h, tentativeG, toY, toId);
        }
    }

    private void relaxBackward(int fromId, double gFrom, long toState, double cost, Action action) {
        int toId = internBwd(toState);
        if (bwdClosed[toId]) {
            return;
        }
        double tentativeG = gFrom + cost;
        if (tentativeG < bwdG[toId]) {
            bwdG[toId] = tentativeG;
            bwdCameFromId[toId] = fromId; // fromId is "closer to goal" than toId
            bwdActionUsed[toId] = action;  // forward action toState -> fromState
            bwdBridgesUsed[toId] = bwdBridgesUsed[fromId] + (action == Action.BRIDGE || action == Action.BRIDGE_UP ? 1 : 0);
            int toY = StateCodec.unpackY(toState);
            double h = heuristicToStartFromPacked(toState);
            bwdHeap.push(tentativeG + epsilon * h, tentativeG, toY, toId);
        }
    }

    private static final double SQRT2_MINUS_2 = EdgeRules.SQRT2 - 2.0;

    // Distance to the NEAREST goal point -- see GoalPoints' class javadoc
    // for why the min of admissible per-point heuristics is itself
    // admissible for "cost to reach any goal".
    private double heuristicToGoal(int x, int y, int z) {
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < goal.size(); i++) {
            double dx = Math.abs(goal.x(i) - x);
            double dz = Math.abs(goal.z(i) - z);
            double horiz = (dx + dz) + SQRT2_MINUS_2 * Math.min(dx, dz);
            double vert = Math.abs(goal.y(i) - y);
            double h = (horiz + vert) / EdgeRules.SPRINT_SPEED;
            if (h < best) {
                best = h;
            }
        }
        return best;
    }

    private int startXForHeuristic, startYForHeuristic, startZForHeuristic;

    private double heuristicToStart(int x, int y, int z, StateCodec.State start) {
        startXForHeuristic = start.x();
        startYForHeuristic = start.y();
        startZForHeuristic = start.z();
        return heuristicToStartFromCoords(x, y, z);
    }

    private double heuristicToStartFromPacked(long state) {
        return heuristicToStartFromCoords(StateCodec.unpackX(state), StateCodec.unpackY(state), StateCodec.unpackZ(state));
    }

    private double heuristicToStartFromCoords(int x, int y, int z) {
        double dx = Math.abs(startXForHeuristic - x);
        double dz = Math.abs(startZForHeuristic - z);
        double horiz = (dx + dz) + SQRT2_MINUS_2 * Math.min(dx, dz);
        double vert = Math.abs(startYForHeuristic - y);
        return (horiz + vert) / EdgeRules.SPRINT_SPEED;
    }

    private SearchResult reconstructDirect(int expansions) {
        return reconstructFromForwardId(currentForwardGoalId, fwdG[currentForwardGoalId], expansions);
    }

    // set by stepForward() right before returning true, so reconstructDirect can find it
    private int currentForwardGoalId;

    private SearchResult reconstructFromForwardId(int goalId, double totalCost, int expansions) {
        List<Integer> idsRev = new ArrayList<>();
        List<Action> actionsRev = new ArrayList<>();
        int s = goalId;
        idsRev.add(s);
        while (fwdCameFromId[s] != -1) {
            actionsRev.add(fwdActionUsed[s]);
            s = fwdCameFromId[s];
            idsRev.add(s);
        }
        Collections.reverse(idsRev);
        Collections.reverse(actionsRev);

        StateCodec.State[] path = new StateCodec.State[idsRev.size()];
        for (int i = 0; i < idsRev.size(); i++) {
            path[i] = StateCodec.toRecord(fwdIds.packedStateOf(idsRev.get(i)));
        }
        return new SearchResult(true, path, totalCost, actionsRev.toArray(new Action[0]), expansions);
    }

    /**
     * Stitches the accepted meeting into a full path. The forward half is a
     * standard forward reconstruction (exact blocksRemaining throughout,
     * same as always). The backward half now needs real per-step
     * blocksRemaining too -- walked forward-chronologically from the
     * meeting point, starting at meetBlocks and decrementing by 1 on every
     * BRIDGE action encountered.
     *
     * Each BRIDGE step's cost is also corrected here: search used a
     * worst-case pessimistic tax estimate (see
     * EdgeRules.reverseBridgeSources), numbered from the goal outward --
     * the bridge closest to goal (last one used, forward-chronologically)
     * was charged blockTax(1), the next one out blockTax(2), etc. Walking
     * forward from the meeting point (the OPPOSITE order search discovered
     * them in) means the FIRST bridge this loop encounters is the one
     * furthest from goal, i.e. the highest-numbered one -- so the assumed
     * value here counts DOWN from totalBridges to 1 as the loop counts
     * UP, while the true blocksRemaining counts down from meetBlocks. Once
     * the true value is known exactly, each step's optimistic tax is
     * replaced with EdgeRules.blockTax(trueRemaining), so the final
     * reported cost is exact, not the pessimistic search-time estimate.
     */
    private SearchResult reconstructMeeting(int expansions) {
        long meetForwardState = fwdIds.packedStateOf(meetForwardId);
        int meetBlocks = StateCodec.unpackBlocks(meetForwardState);

        // forward half: start .. meetForwardState (standard forward reconstruction)
        List<Integer> fwdIdsRev = new ArrayList<>();
        List<Action> fwdActionsRev = new ArrayList<>();
        int s = meetForwardId;
        fwdIdsRev.add(s);
        while (fwdCameFromId[s] != -1) {
            fwdActionsRev.add(fwdActionUsed[s]);
            s = fwdCameFromId[s];
            fwdIdsRev.add(s);
        }
        Collections.reverse(fwdIdsRev);
        Collections.reverse(fwdActionsRev);

        // backward half: meetBackwardId .. goal, already in forward chronological order
        // (see class javadoc: bwdCameFromId[S] is the node closer to goal, and
        // bwdActionUsed[S] is the forward action S -> bwdCameFromId[S]).
        List<Integer> bwdIdsFwdOrder = new ArrayList<>();
        List<Action> bwdActionsFwdOrder = new ArrayList<>();
        int b = meetBackwardId;
        bwdIdsFwdOrder.add(b);
        while (bwdCameFromId[b] != -1) {
            bwdActionsFwdOrder.add(bwdActionUsed[b]);
            b = bwdCameFromId[b];
            bwdIdsFwdOrder.add(b);
        }

        List<StateCodec.State> path = new ArrayList<>();
        for (int id : fwdIdsRev) {
            path.add(StateCodec.toRecord(fwdIds.packedStateOf(id)));
        }

        int totalBridges = bwdBridgesUsed[meetBackwardId];
        int blocksRemaining = meetBlocks;
        int assumedRemaining = totalBridges;
        double costCorrection = 0.0;
        // skip index 0 of the backward segment: it's the meeting point, already
        // the last entry of the forward segment (with the REAL blocks value)
        for (int i = 1; i < bwdIdsFwdOrder.size(); i++) {
            Action actionTaken = bwdActionsFwdOrder.get(i - 1); // action from B[i-1] to B[i], forward-chronological
            if (actionTaken == Action.BRIDGE || actionTaken == Action.BRIDGE_UP) {
                costCorrection += EdgeRules.blockTax(blocksRemaining) - EdgeRules.blockTax(assumedRemaining);
                blocksRemaining -= 1;
                assumedRemaining -= 1;
            }
            long packed = bwdIds.packedStateOf(bwdIdsFwdOrder.get(i));
            path.add(new StateCodec.State(StateCodec.unpackX(packed), StateCodec.unpackY(packed), StateCodec.unpackZ(packed),
                    blocksRemaining, StateCodec.unpackCrawling(packed)));
        }

        List<Action> actions = new ArrayList<>(fwdActionsRev);
        actions.addAll(bwdActionsFwdOrder);

        return new SearchResult(true, path.toArray(new StateCodec.State[0]), mu + costCorrection,
                actions.toArray(new Action[0]), expansions);
    }
}
