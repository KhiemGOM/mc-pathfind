package dev.mcpathfind.core;

/**
 * found=false covers both budget-exceeded and open-set-exhausted cases
 * (Python returns (None,None,None,expansions) for both) -- expansions is
 * always populated even on failure, for diagnostics.
 */
public record SearchResult(
        boolean found,
        StateCodec.State[] path,
        double totalCost,
        Action[] actions,
        int expansions
) {}
