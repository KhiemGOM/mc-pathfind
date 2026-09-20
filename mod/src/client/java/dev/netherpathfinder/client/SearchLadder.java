package dev.netherpathfinder.client;

import dev.netherpathfinder.config.PathfinderSettings;
import dev.netherpathfinder.engine.GoalPoints;
import dev.netherpathfinder.engine.MineCostModel;
import dev.netherpathfinder.engine.SearchResult;
import dev.netherpathfinder.engine.StateCodec;
import dev.netherpathfinder.engine.WeightedAStar;
import dev.netherpathfinder.engine.World;

import java.util.concurrent.CancellationException;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

/**
 * Retry ladder for the demo planner: start at the (near-)optimal weight and
 * loosen it rung by rung until a route is found, so a hard terrain costs
 * time rather than failing outright. Each rung gets a fresh solver (the
 * expansion budget is per rung); one shared deadline covers the whole ladder.
 */
final class SearchLadder {
    private SearchLadder() {}

    /** Weight added to ladderStartWeight on each successive rung. */
    private static final double[] RUNG_OFFSETS = {0.0, 0.4, 0.9, 1.9};
    /** Each rung's expansion budget is demoMaxExpansions times this: later rungs get more room. */
    private static final double[] BUDGET_SCALE = {1.0, 1.0, 1.25, 1.5};
    private static final long STATUS_PERIOD_MS = 250;
    private static final double MAX_WEIGHT = 5.0;

    private static final ScheduledExecutorService DEADLINES = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread t = new Thread(r, "netherpathfinder-ladder-deadline");
        t.setDaemon(true);
        return t;
    });

    record Outcome(SearchResult result, double weight, int rung, int totalExpansions, boolean timedOut) {}

    private static String clock(double seconds) {
        long s = Math.max(0, Math.round(seconds));
        return String.format(java.util.Locale.ROOT, "%d:%02d", s / 60, s % 60);
    }

    static Outcome run(World world, StateCodec.State start, GoalPoints goal, int blocks,
                       MineCostModel tools, PathfinderSettings.Snapshot cfg, Consumer<String> progress,
                       Consumer<String> status) {
        Thread worker = Thread.currentThread();
        boolean[] deadlineHit = {false};
        Future<?> deadline = DEADLINES.schedule(() -> {
            deadlineHit[0] = true;
            worker.interrupt();
        }, Math.round(cfg.demoTimeoutSeconds() * 1000), TimeUnit.MILLISECONDS);

        long startedNanos = System.nanoTime();
        double limitSeconds = cfg.demoTimeoutSeconds();
        // Live readout: which rung, how many expansions, elapsed against the deadline.
        final WeightedAStar[] running = {null};
        final int[] rungInfo = {0, 0}; // {rung index, this rung's budget}
        final double[] rungWeight = {cfg.ladderStartWeight()};
        final int[] finished = {0};    // expansions from completed rungs
        Future<?> monitor = DEADLINES.scheduleAtFixedRate(() -> {
            WeightedAStar s = running[0];
            if (s == null) return;
            int now = s.expansionsSoFar();
            double elapsed = (System.nanoTime() - startedNanos) / 1.0e9;
            status.accept(String.format(java.util.Locale.ROOT,
                "Bastion route | rung %d/%d (weight %.2f) | %,d expansions (rung: %,d / %,d) | %s / %s",
                rungInfo[0] + 1, RUNG_OFFSETS.length, rungWeight[0], finished[0] + now, now, rungInfo[1],
                clock(elapsed), clock(limitSeconds)));
        }, 0, STATUS_PERIOD_MS, TimeUnit.MILLISECONDS);

        int totalExpansions = 0;
        SearchResult last = null;
        double lastWeight = cfg.ladderStartWeight();
        try {
            for (int rung = 0; rung < RUNG_OFFSETS.length; rung++) {
                double weight = Math.min(MAX_WEIGHT, cfg.ladderStartWeight() + RUNG_OFFSETS[rung]);
                if (rung > 0 && weight <= lastWeight) continue; // clamped duplicate of the previous rung
                lastWeight = weight;
                // The previous rung's arrays are garbage now; reclaim them BEFORE measuring headroom and allocating.
                running[0] = null;
                if (rung > 0) System.gc();
                int wanted = (int) Math.min(Integer.MAX_VALUE - 1, Math.round(cfg.demoMaxExpansions() * BUDGET_SCALE[rung]));
                int budget = MemoryGuard.cap(wanted);
                progress.accept(String.format(java.util.Locale.ROOT,
                    "search rung %d/%d (weight %.2f, up to %,d expansions%s)...",
                    rung + 1, RUNG_OFFSETS.length, weight, budget,
                    budget < wanted ? String.format(java.util.Locale.ROOT, "; capped from %,d to fit free memory", wanted) : ""));
                WeightedAStar solver = new WeightedAStar();
                rungInfo[0] = rung;
                rungInfo[1] = budget;
                rungWeight[0] = weight;
                finished[0] = totalExpansions;
                running[0] = solver;
                try {
                    last = solver.search(world, start, goal, blocks, weight, tools, budget);
                } catch (CancellationException e) {
                    if (deadlineHit[0]) {
                        Thread.interrupted(); // clear the deadline's interrupt so the caller can keep working
                        return new Outcome(last, weight, rung, totalExpansions, true);
                    }
                    throw e;
                }
                totalExpansions += last.expansions();
                if (last.found()) return new Outcome(last, weight, rung, totalExpansions, false);
            }
            return new Outcome(last, lastWeight, RUNG_OFFSETS.length - 1, totalExpansions, false);
        } finally {
            monitor.cancel(false);
            deadline.cancel(false);
            if (deadlineHit[0]) Thread.interrupted();
        }
    }
}
