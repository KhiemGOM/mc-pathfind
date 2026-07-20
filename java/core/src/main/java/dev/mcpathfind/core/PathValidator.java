package dev.mcpathfind.core;

/**
 * Independently re-checks that every consecutive state pair in a returned
 * path is a legal transition under the world rules for its claimed action --
 * deliberately does NOT call into WeightedAStar's expand/relax, so this is a
 * genuine cross-check rather than "the same code agreeing with itself".
 * Does not recompute exact costs, only structural/geometric legality
 * (solid/lava/void state of the relevant cells, distance, action-specific
 * preconditions) plus the diagonal corner-cutting rule.
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
            case BRIDGE: {
                if (to.blocksRemaining() != from.blocksRemaining() - 1) return "BRIDGE didn't consume exactly 1 block";
                if (diagonalCornerBlocked(world, from, dx, dz)) return "BRIDGE cuts a blocked diagonal corner";
                int target = world.voxelAt(to.x(), to.y(), to.z());
                int head = world.voxelAt(to.x(), to.y() + 1, to.z());
                if (BlockType.isSolid(target) || BlockType.isLava(target)) return "BRIDGE target blocked/lava";
                if (BlockType.isSolid(head) || BlockType.isLava(head)) return "BRIDGE head blocked/lava";
                return null;
            }
            case BRIDGE_UP: {
                if (dx != 0 || dz != 0 || dy != 1) return "BRIDGE_UP not a straight-up single step";
                if (to.blocksRemaining() != from.blocksRemaining() - 1) return "BRIDGE_UP didn't consume exactly 1 block";
                int target = world.voxelAt(to.x(), to.y(), to.z());
                int head = world.voxelAt(to.x(), to.y() + 1, to.z());
                if (BlockType.isSolid(target) || BlockType.isLava(target)) return "BRIDGE_UP target blocked/lava";
                if (BlockType.isSolid(head) || BlockType.isLava(head)) return "BRIDGE_UP head blocked/lava";
                return null;
            }
            case FALL: {
                if (dy > 0) return "FALL with positive dy";
                if (diagonalCornerBlocked(world, from, dx, dz)) return "FALL cuts a blocked diagonal corner";
                int landingSupport = world.voxelAt(to.x(), to.y() - 1, to.z());
                if (!BlockType.isSolid(landingSupport)) return "FALL landing has no solid floor";
                return null;
            }
            case PARKOUR: {
                if (dy < -1 || dy > 1) return "PARKOUR with dy=" + dy + " outside {-1,0,1}";
                double dist = Math.sqrt((double) (dx * dx + dz * dz));
                if (dist <= EdgeRules.MIN_PARKOUR_DIST || dist > EdgeRules.MAX_PARKOUR_DIST) {
                    return "PARKOUR distance " + dist + " outside (MIN_PARKOUR_DIST, MAX_PARKOUR_DIST]";
                }
                int landSupport = world.voxelAt(to.x(), to.y() - 1, to.z());
                int landTarget = world.voxelAt(to.x(), to.y(), to.z());
                int landHead = world.voxelAt(to.x(), to.y() + 1, to.z());
                if (!BlockType.isSolid(landSupport)) return "PARKOUR landing has no solid floor";
                if (BlockType.isSolid(landTarget) || BlockType.isLava(landTarget)) return "PARKOUR landing target blocked/lava";
                if (BlockType.isSolid(landHead) || BlockType.isLava(landHead)) return "PARKOUR landing head blocked/lava";
                int nSamples = Math.max(2, (int) Math.round(dist));
                for (int s = 1; s < nSamples; s++) {
                    double t = (double) s / nSamples;
                    int sx = (int) Math.round(from.x() + dx * t);
                    int sz = (int) Math.round(from.z() + dz * t);
                    int sy = (int) Math.round(from.y() + dy * t);
                    int sampleFoot = world.voxelAt(sx, sy, sz);
                    int sampleHead = world.voxelAt(sx, sy + 1, sz);
                    if (BlockType.isSolid(sampleFoot) || BlockType.isLava(sampleFoot)
                            || BlockType.isSolid(sampleHead) || BlockType.isLava(sampleHead)) {
                        return "PARKOUR trajectory blocked at sample " + s + "/" + nSamples;
                    }
                }
                return null;
            }
            case START:
                return "START action should never appear mid-path";
            default:
                return "unhandled action " + action;
        }
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
