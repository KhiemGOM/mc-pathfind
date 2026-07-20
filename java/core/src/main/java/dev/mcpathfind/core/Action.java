package dev.mcpathfind.core;

/** Matches the 8 action-label strings yielded by get_neighbors in pathfind.py, plus BRIDGE_UP (Java-only addition, not in the Python reference). */
public enum Action {
    START, // synthetic action for the start node only (matches Python's PQItem seed action)
    SPRINT,
    MINE,
    MINE_DOWN,
    BOAT_CRAWL,
    BRIDGE,
    FALL,
    CLIMB,
    PARKOUR, // long jump; see EdgeRules.parkourEdges
    BRIDGE_UP, // straight-up pillar move -- see EdgeRules.bridgeUpEdge for why this didn't exist until now
}
