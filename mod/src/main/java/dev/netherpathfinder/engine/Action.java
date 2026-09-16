package dev.netherpathfinder.engine;

/** Movement/action labels produced by the pathfinder. */
public enum Action {
    START, // synthetic action for the start node only
    SPRINT,
    MINE,
    MINE_DOWN,
    BOAT_CRAWL,
    BRIDGE,
    FALL,
    CLIMB,
    PARKOUR, // long jump; see EdgeRules.parkourEdges
    BRIDGE_UP, // straight-up pillar move -- see EdgeRules.bridgeUpEdge
    PEARL, // stationary ender pearl throw -- reserved for a future pearl-pathing variant
    SPRINT_JUMP_PEARL, // compound run-up + jump + first-airborne-tick pearl throw
}
