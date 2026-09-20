package dev.netherpathfinder.client;

/**
 * Keeps a search from eating the machine. The solver holds roughly 250-300 bytes per expansion on
 * test terrain (more on real terrain), so the budget is capped by what the JVM heap AND the OS have
 * free -- using at most half of the smaller, leaving the rest for the game and for the previous
 * rung's garbage.
 */
final class MemoryGuard {
    private MemoryGuard() {}

    /** Conservative bytes per expansion (measured ~220-290 on synthetic terrain; real terrain branches more). */
    private static final long BYTES_PER_EXPANSION = 450;
    private static final int FLOOR = 200_000;

    /** Physical memory the OS reports as available, or Long.MAX_VALUE if it can't be read. */
    static long physicalFreeBytes() {
        try {
            var os = java.lang.management.ManagementFactory.getOperatingSystemMXBean();
            if (os instanceof com.sun.management.OperatingSystemMXBean sun) return sun.getFreeMemorySize();
        } catch (Throwable ignored) {
            // fall through: heap headroom alone still bounds the search
        }
        return Long.MAX_VALUE;
    }

    static long heapHeadroomBytes() {
        Runtime rt = Runtime.getRuntime();
        return rt.maxMemory() - (rt.totalMemory() - rt.freeMemory());
    }

    /** The largest expansion budget that is safe to start right now, never above `requested`. */
    static int cap(int requested) {
        long usable = Math.min(heapHeadroomBytes(), physicalFreeBytes()) / 2;
        long byMemory = Math.max(0, usable) / BYTES_PER_EXPANSION;
        return (int) Math.max(FLOOR, Math.min(requested, byMemory));
    }
}
