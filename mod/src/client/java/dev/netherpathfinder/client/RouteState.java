package dev.netherpathfinder.client;

import dev.netherpathfinder.engine.Action;

import java.util.concurrent.atomic.AtomicReference;

/**
 * The currently active route (if any), shared between the background search
 * thread, the command thread, and the render thread. A route is a sequence
 * of world-space waypoints plus the Action that produced each step from the
 * previous one (actions.length == points.length - 1) -- pure data, this
 * class holds it, it doesn't compute or draw it.
 */
public final class RouteState {
    public static final RouteState INSTANCE = new RouteState();

    public enum Status { IDLE, SEARCHING, FOUND, NOT_FOUND, ERROR }

    public static final class Route {
        public final double[][] points; // world-space [x,y,z] per waypoint, feet position
        public final Action[] actions;
        public final Status status;
        public final String message; // human-readable detail (expansions used, error text, etc.)

        public Route(double[][] points, Action[] actions, Status status, String message) {
            this.points = points;
            this.actions = actions;
            this.status = status;
            this.message = message;
        }
    }

    private static final Route IDLE_ROUTE = new Route(new double[0][], new Action[0], Status.IDLE, null);

    private final AtomicReference<Route> current = new AtomicReference<>(IDLE_ROUTE);

    private RouteState() {}

    public Route get() {
        return current.get();
    }

    public void set(Route route) {
        current.set(route);
    }

    public void clear() {
        current.set(IDLE_ROUTE);
    }
}
