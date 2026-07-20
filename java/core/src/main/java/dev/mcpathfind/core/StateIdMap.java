package dev.mcpathfind.core;

import it.unimi.dsi.fastutil.longs.Long2IntOpenHashMap;

/**
 * Hands out small dense integer IDs (0, 1, 2, ...) for packed state longs,
 * one hash-map lookup per DISTINCT state ever seen. Once a state has an ID,
 * every other per-state structure in the search (gScore, cameFrom, closed,
 * the heap's position index, ...) can be a plain array indexed by that ID
 * instead of a hash map keyed by the raw packed long -- array reads/writes
 * are far cheaper than hashing+bucket-search, and unlike a hash map they
 * cost the same whether this is the 1st or the 10,000th touch of that id.
 *
 * This is the ONLY hash map in the redesigned WeightedAStar hot path, and
 * it's only ever consulted once per distinct state (on first sight); every
 * subsequent read/write for that state goes through the returned int id.
 */
public final class StateIdMap {
    private final Long2IntOpenHashMap idOf;
    private long[] packedOf;
    private int nextId;

    public StateIdMap(int initialCapacity) {
        idOf = new Long2IntOpenHashMap(Math.max(16, initialCapacity));
        idOf.defaultReturnValue(-1);
        packedOf = new long[Math.max(16, initialCapacity)];
        nextId = 0;
    }

    /** Returns this state's existing id, or allocates and returns a new one. */
    public int idFor(long packedState) {
        int id = idOf.get(packedState);
        if (id != -1) {
            return id;
        }
        id = nextId++;
        idOf.put(packedState, id);
        if (id == packedOf.length) {
            packedOf = java.util.Arrays.copyOf(packedOf, packedOf.length + (packedOf.length >> 1) + 1);
        }
        packedOf[id] = packedState;
        return id;
    }

    /** Returns -1 if this state has never been assigned an id. */
    public int existingIdFor(long packedState) {
        return idOf.get(packedState);
    }

    public long packedStateOf(int id) {
        return packedOf[id];
    }

    public int idCount() {
        return nextId;
    }
}
