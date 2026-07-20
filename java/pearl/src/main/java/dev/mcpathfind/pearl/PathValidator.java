package dev.mcpathfind.pearl;

/**
 * Forked from dev.mcpathfind.core.PathValidator for the trimmed action set
 * -- see there for the full rationale (independent cross-check, doesn't
 * call into WeightedAStar's own expand/relax), unchanged here. BRIDGE/
 * BRIDGE_UP/PARKOUR cases dropped along with the actions themselves; PEARL
 * is a placeholder (landing-only check) until pearlEdges' actual trajectory
 * math exists -- see EdgeRules.pearlEdges' javadoc.
 */
public final class PathValidator {

    public record Violation(int stepIndex, String reason) {}

    public static java.util.List<Violation> validate(World world, StateCodec.State[] path, Action[] actions) {
        java.util.List<Violation> violations = new java.util.ArrayList<>();
        if (path.length == 0) {
            return violations;
        }
        if (actions.length != path.length - 1) {
            violations.add(new Violation(-1, "actions.length=" + actions.length + " != path.length-1=" + (path.length - 1)));
            return violations;
        }
        for (int i = 1; i < path.length; i++) {
            StateCodec.State from = path[i - 1];
            StateCodec.State to = path[i];
            Action action = actions[i - 1];
            String reason = checkStep(world, from, to, action);
            if (reason != null) {
                violations.add(new Violation(i, reason));
            }
        }
        return violations;
    }

    private static String checkStep(World world, StateCodec.State from, StateCodec.State to, Action action) {
        int dx = to.x() - from.x();
        int dz = to.z() - from.z();
        int dy = to.y() - from.y();

        switch (action) {
            case SPRINT: {
                if (dy != 0) return "SPRINT with dy=" + dy;
                if (Math.abs(dx) > 1 || Math.abs(dz) > 1) return "SPRINT distance too large: dx=" + dx + " dz=" + dz;
                if (diagonalCornerBlocked(world, from, dx, dz)) return "SPRINT cuts a blocked diagonal corner";
                int below = world.voxelAt(to.x(), to.y() - 1, to.z());
                int target = world.voxelAt(to.x(), to.y(), to.z());
                int head = world.voxelAt(to.x(), to.y() + 1, to.z());
                if (!BlockType.isSolid(below)) return "SPRINT onto non-solid floor";
                if (BlockType.isSolid(target) || BlockType.isLava(target)) return "SPRINT into blocked/lava target";
                if (BlockType.isSolid(head) || BlockType.isLava(head)) return "SPRINT into blocked/lava head";
                return null;
            }
            case CLIMB: {
                if (dy != 1) return "CLIMB with dy=" + dy;
                if (Math.abs(dx) > 1 || Math.abs(dz) > 1) return "CLIMB distance too large";
                if (diagonalCornerBlocked(world, from, dx, dz)) return "CLIMB cuts a blocked diagonal corner";
                int belowUp = world.voxelAt(to.x(), to.y() - 1, to.z());
                int targetUp = world.voxelAt(to.x(), to.y(), to.z());
                int headClear = world.voxelAt(to.x(), to.y() + 1, to.z());
                if (!BlockType.isSolid(belowUp)) return "CLIMB landing has no solid floor";
                if (BlockType.isSolid(targetUp) || BlockType.isLava(targetUp)) return "CLIMB target blocked/lava";
                if (BlockType.isSolid(headClear) || BlockType.isLava(headClear)) return "CLIMB head blocked/lava";
                return null;
            }
            case MINE: {
                if (dy != 0) return "MINE with dy=" + dy;
                if (Math.abs(dx) > 1 || Math.abs(dz) > 1) return "MINE distance too large";
                if (diagonalCornerBlocked(world, from, dx, dz)) return "MINE cuts a blocked diagonal corner";
                int target = world.voxelAt(to.x(), to.y(), to.z());
                int head = world.voxelAt(to.x(), to.y() + 1, to.z());
                if (!BlockType.isSolid(target)) return "MINE target wasn't solid before mining";
                if (BlockType.isUnbreakable(target)) return "MINE target is unbreakable";
                if (BlockType.isUnbreakable(head) || BlockType.isLava(head)) return "MINE head is unbreakable/lava";
                return null;
            }
            case MINE_DOWN: {
                if (dx != 0 || dz != 0 || dy != -1) return "MINE_DOWN not a straight-down single step";
                int below = world.voxelAt(to.x(), to.y(), to.z());
                if (!BlockType.isSolid(below) || BlockType.isUnbreakable(below)) return "MINE_DOWN target not solid+breakable";
                return null;
            }
            case BOAT_CRAWL: {
                if (dy != 0) return "BOAT_CRAWL with dy=" + dy;
                if (diagonalCornerBlocked(world, from, dx, dz)) return "BOAT_CRAWL cuts a blocked diagonal corner";
                int target = world.voxelAt(to.x(), to.y(), to.z());
                int head = world.voxelAt(to.x(), to.y() + 1, to.z());
                if (!BlockType.isSolid(target) || BlockType.isUnbreakable(target)) return "BOAT_CRAWL target not solid+breakable";
                if (!BlockType.isSolid(head)) return "BOAT_CRAWL head wasn't solid (should stay solid)";
                return null;
            }
            case FALL: {
                if (dy > 0) return "FALL with positive dy";
                if (diagonalCornerBlocked(world, from, dx, dz)) return "FALL cuts a blocked diagonal corner";
                int landingSupport = world.voxelAt(to.x(), to.y() - 1, to.z());
                if (!BlockType.isSolid(landingSupport)) return "FALL landing has no solid floor";
                return null;
            }
            case PEARL: {
                int landSupport = world.voxelAt(to.x(), to.y() - 1, to.z());
                int landTarget = world.voxelAt(to.x(), to.y(), to.z());
                int landHead = world.voxelAt(to.x(), to.y() + 1, to.z());
                if (!BlockType.isSolid(landSupport)) return "PEARL landing has no solid floor";
                if (BlockType.isSolid(landTarget) || BlockType.isLava(landTarget)) return "PEARL landing target blocked/lava";
                if (BlockType.isSolid(landHead) || BlockType.isLava(landHead)) return "PEARL landing head blocked/lava";

                // Independently re-derive AND re-check the actual flight
                // path -- deliberately does NOT trust that EdgeRules'
                // solver found a clean arc, it re-solves from scratch
                // (calling EdgeRules' pure geometry/physics functions is
                // fine here, same as the core validator referencing
                // EdgeRules.MIN_PARKOUR_DIST -- "independent" means not
                // relying on WeightedAStar's own search-state bookkeeping,
                // not never touching EdgeRules' math at all). The
                // collision scan itself, though, is NOT shared with
                // EdgeRules.sampleArcCollisions -- see
                // strictPearlCollisionCheck's javadoc for why reusing the
                // solver's own (speed-tuned) collision code here would
                // defeat the point of an independent check.
                double sourceEyeX = from.x() + 0.5, sourceEyeY = from.y() + EdgeRules.PEARL_EYE_HEIGHT, sourceEyeZ = from.z() + 0.5;
                double ddx = to.x() + 0.5 - sourceEyeX;
                double ddy = to.y() - sourceEyeY;
                double ddz = to.z() + 0.5 - sourceEyeZ;
                EdgeRules.PearlArc arc = EdgeRules.solvePearlArc(ddx, ddy, ddz);
                if (arc == null) return "PEARL: no physically valid arc reaches this target";
                return strictPearlCollisionCheck(sourceEyeX, sourceEyeY, sourceEyeZ,
                        arc.yaw(), arc.pitch(), arc.ticks(), world);
            }
            case START:
                return "START action should never appear mid-path";
            default:
                return "unhandled action " + action;
        }
    }

    // Deliberately NOT EdgeRules.PEARL_ARC_SAMPLE_TICK_STEP (the solver's
    // speed-tuned 1-tick step): strictPearlCollisionCheck runs once per
    // already-accepted PEARL edge during validation, not once per
    // candidate during a live search, so it can afford to be far more
    // paranoid. 1000x finer than the solver's step keeps consecutive
    // samples within a tiny fraction of a block of each other everywhere
    // in the flight, regardless of instantaneous speed.
    private static final double STRICT_PEARL_SAMPLE_TICK_STEP = 0.001;

    /**
     * Fully independent re-check of a PEARL arc's collision-freedom --
     * deliberately does NOT call EdgeRules.sampleArcCollisions (the
     * solver's own, speed-tuned collision check). If a bug ever crept
     * into that check, calling it here would validate the bug as
     * "correct" by definition, defeating the entire point of an
     * independent validator -- this project has already hit two distinct
     * collision-check bugs this way (an adaptive step size that only
     * accounted for horizontal speed, missing steep-arc ceilings; then a
     * coarse per-tick point check that could cut a diagonal corner
     * between two face-adjacent cells), both found by manual review
     * rather than by this validator, precisely because the validator was
     * trusting the same code it was supposed to be checking.
     *
     * Re-implemented from scratch here, sharing only the pure closed-form
     * physics (the geometric-series position formula, and
     * PEARL_DRAG/PEARL_TERMINAL_G/PEARL_THROW_SPEED -- constants and
     * exact math, not a sampling strategy or approximation), at an
     * extremely fine fixed time step with a full bounding-box check
     * between consecutive samples (same gap-closing technique the solver
     * uses, just dialed up ~1000x since performance is irrelevant for a
     * one-shot validation pass, not a search-time hot path). Returns a
     * description of the first solid block hit, or null if the whole
     * flight is genuinely clear.
     */
    private static String strictPearlCollisionCheck(double sourceX, double sourceY, double sourceZ,
                                                      double yaw, double pitch, int ticks, World world) {
        double vh0 = EdgeRules.PEARL_THROW_SPEED * Math.cos(pitch);
        double vy0 = EdgeRules.PEARL_THROW_SPEED * Math.sin(pitch);
        double cosYaw = Math.cos(yaw), sinYaw = Math.sin(yaw);
        double dragComplement = 1.0 - EdgeRules.PEARL_DRAG;
        int prevX = (int) Math.floor(sourceX);
        int prevY = (int) Math.floor(sourceY);
        int prevZ = (int) Math.floor(sourceZ);
        double t = 0.0;
        while (t < ticks) {
            t = Math.min(t + STRICT_PEARL_SAMPLE_TICK_STEP, ticks);
            double s = (1.0 - Math.pow(EdgeRules.PEARL_DRAG, t)) / dragComplement;
            double h = vh0 * s;
            double dy = (vy0 + EdgeRules.PEARL_TERMINAL_G) * s - EdgeRules.PEARL_TERMINAL_G * t;
            int sx = (int) Math.floor(sourceX + h * cosYaw);
            int sy = (int) Math.floor(sourceY + dy);
            int sz = (int) Math.floor(sourceZ + h * sinYaw);
            int loX = Math.min(prevX, sx), hiX = Math.max(prevX, sx);
            int loY = Math.min(prevY, sy), hiY = Math.max(prevY, sy);
            int loZ = Math.min(prevZ, sz), hiZ = Math.max(prevZ, sz);
            for (int xi = loX; xi <= hiX; xi++) {
                for (int yi = loY; yi <= hiY; yi++) {
                    for (int zi = loZ; zi <= hiZ; zi++) {
                        if (BlockType.isSolid(world.voxelAt(xi, yi, zi))) {
                            return "PEARL trajectory clips a solid block at (" + xi + "," + yi + "," + zi
                                + ") near t=" + String.format(java.util.Locale.ROOT, "%.3f", t) + " ticks";
                        }
                    }
                }
            }
            prevX = sx;
            prevY = sy;
            prevZ = sz;
        }
        return null;
    }

    private static boolean diagonalCornerBlocked(World world, StateCodec.State from, int dx, int dz) {
        if (dx == 0 || dz == 0) return false;
        int x = from.x(), y = from.y(), z = from.z();
        boolean corner1 = BlockType.isSolid(world.voxelAt(x + dx, y, z)) || BlockType.isSolid(world.voxelAt(x + dx, y + 1, z));
        boolean corner2 = BlockType.isSolid(world.voxelAt(x, y, z + dz)) || BlockType.isSolid(world.voxelAt(x, y + 1, z + dz));
        return corner1 && corner2;
    }

    private PathValidator() {}
}
