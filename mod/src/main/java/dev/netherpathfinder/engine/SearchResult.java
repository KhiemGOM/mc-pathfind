package dev.netherpathfinder.engine;

/** Immutable result of one weighted-A* search. */
public final class SearchResult {
    public enum TerminationReason {
        NONE,
        EXPANSION_LIMIT,
        TIME_LIMIT
    }

    private final boolean found;
    private final StateCodec.State[] path;
    private final double totalCost;
    private final Action[] actions;
    private final int expansions;
    private final TerminationReason terminationReason;

    public SearchResult(boolean found, StateCodec.State[] path, double totalCost,
                        Action[] actions, int expansions) {
        this(found, path, totalCost, actions, expansions, TerminationReason.NONE);
    }

    public SearchResult(boolean found, StateCodec.State[] path, double totalCost,
                        Action[] actions, int expansions,
                        TerminationReason terminationReason) {
        this.found = found;
        this.path = path;
        this.totalCost = totalCost;
        this.actions = actions;
        this.expansions = expansions;
        this.terminationReason = terminationReason == null
            ? TerminationReason.NONE : terminationReason;
    }

    public boolean found() { return found; }
    public StateCodec.State[] path() { return path; }
    public double totalCost() { return totalCost; }
    public Action[] actions() { return actions; }
    public int expansions() { return expansions; }
    public TerminationReason terminationReason() { return terminationReason; }
    public boolean budgetExhausted() {
        return terminationReason == TerminationReason.EXPANSION_LIMIT
            || terminationReason == TerminationReason.TIME_LIMIT;
    }
    public boolean timedOut() {
        return terminationReason == TerminationReason.TIME_LIMIT;
    }
}
