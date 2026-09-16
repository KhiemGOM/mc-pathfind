package dev.netherpathfinder.engine;

/** Primitive long-to-int map plus dense reverse lookup for pathfinder states. */
public final class StateIdMap {
    private static final float LOAD_FACTOR = 0.65f;

    private long[] keys;
    private int[] values;
    private boolean[] used;
    private int mask;
    private int resizeAt;
    private int mapSize;
    private long[] packedOf;
    private int nextId;

    public StateIdMap(int initialCapacity) {
        int expected = Math.max(16, Math.min(initialCapacity, 1_048_576));
        int tableSize = 16;
        while (tableSize < expected / LOAD_FACTOR && tableSize < (1 << 25)) {
            tableSize <<= 1;
        }
        keys = new long[tableSize];
        values = new int[tableSize];
        used = new boolean[tableSize];
        mask = tableSize - 1;
        resizeAt = Math.max(1, (int) (tableSize * LOAD_FACTOR));
        packedOf = new long[Math.max(16, initialCapacity)];
    }

    public int idFor(long packedState) {
        int slot = findSlot(packedState);
        if (used[slot]) return values[slot];
        if (mapSize >= resizeAt) {
            rehash(keys.length << 1);
            slot = findSlot(packedState);
        }
        int id = nextId++;
        used[slot] = true;
        keys[slot] = packedState;
        values[slot] = id;
        mapSize++;
        if (id == packedOf.length) {
            packedOf = java.util.Arrays.copyOf(
                packedOf, packedOf.length + (packedOf.length >> 1) + 1);
        }
        packedOf[id] = packedState;
        return id;
    }

    public int existingIdFor(long packedState) {
        int slot = findSlot(packedState);
        return used[slot] ? values[slot] : -1;
    }

    public long packedStateOf(int id) {
        return packedOf[id];
    }

    public int idCount() {
        return nextId;
    }

    private int findSlot(long key) {
        int slot = mix(key) & mask;
        while (used[slot] && keys[slot] != key) slot = (slot + 1) & mask;
        return slot;
    }

    private void rehash(int requestedSize) {
        if (requestedSize <= keys.length || requestedSize <= 0) return;
        long[] oldKeys = keys;
        int[] oldValues = values;
        boolean[] oldUsed = used;
        keys = new long[requestedSize];
        values = new int[requestedSize];
        used = new boolean[requestedSize];
        mask = requestedSize - 1;
        resizeAt = Math.max(1, (int) (requestedSize * LOAD_FACTOR));
        mapSize = 0;
        for (int i = 0; i < oldKeys.length; i++) {
            if (!oldUsed[i]) continue;
            int slot = findSlot(oldKeys[i]);
            used[slot] = true;
            keys[slot] = oldKeys[i];
            values[slot] = oldValues[i];
            mapSize++;
        }
    }

    private static int mix(long value) {
        value ^= value >>> 33;
        value *= 0xff51afd7ed558ccdL;
        value ^= value >>> 33;
        value *= 0xc4ceb9fe1a85ec53L;
        value ^= value >>> 33;
        return (int) value;
    }
}
