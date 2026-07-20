package dev.mcpathfind.pearl;

/**
 * Action set for the bastion-to-fortress / fortress-to-stronghold variant --
 * see dev.mcpathfind.core.Action for the bridging/parkour-oriented original
 * this was forked from. BRIDGE, BRIDGE_UP, and PARKOUR are deliberately
 * dropped (out of scope for this route); PEARL is new.
 */
public enum Action {
    START, // synthetic action for the start node only (matches Python's PQItem seed action)
    SPRINT,
    MINE,
    MINE_DOWN,
    BOAT_CRAWL,
    FALL,
    CLIMB,
    PEARL, // ender pearl throw with a parabolic (gravity+drag) arc; see EdgeRules.pearlEdges
}
