package dev.netherpathfinder.engine;

/**
 * Binary min-heap over dense int state IDs (see StateIdMap), supporting
 * decrease-key in O(log n) via a flat position index.
 *
 * positionOf[id] == -1 means that state currently has no open heap entry.
 * positionOf must be sized (or grown) by the caller to cover every id it
 * will ever push -- WeightedAStar does this by growing it in lockstep with
 * StateIdMap's own growth, since both are driven by the same "new state
 * seen" event.
 */
public final class IntIdOpenHeap {
    private double[] priority;
    private double[] g;
    private int[] y;
    private long[] counter;
    private int[] id;
    private int size;
    private long nextCounter;

    private int[] positionOf; // indexed by state id -> this state's slot in the arrays above, or -1

    public IntIdOpenHeap(int initialCapacity, int idCapacityHint) {
        priority = new double[initialCapacity];
        g = new double[initialCapacity];
        y = new int[initialCapacity];
        counter = new long[initialCapacity];
        id = new int[initialCapacity];
        size = 0;
        nextCounter = 0;
        positionOf = new int[Math.max(16, idCapacityHint)];
        java.util.Arrays.fill(positionOf, -1);
    }

    public boolean isEmpty() {
        return size == 0;
    }

    public double peekPriority() {
        return priority[0];
    }

    /** Must be called (or ensured some other way) before any id >= current positionOf.length is pushed. */
    public void ensureIdCapacity(int idCount) {
        if (idCount > positionOf.length) {
            int newCap = Math.max(idCount, positionOf.length + (positionOf.length >> 1) + 1);
            int oldLen = positionOf.length;
            positionOf = java.util.Arrays.copyOf(positionOf, newCap);
            java.util.Arrays.fill(positionOf, oldLen, newCap, -1);
        }
    }

    /**
     * Pushes a new (priority, g, y, stateId), or -- if stateId already has
     * an open entry -- decreases it in place. Caller is expected to only
     * call this when the new priority is a genuine improvement, but this
     * also double-checks.
     */
    public void push(double p, double gValue, int yValue, int stateId) {
        int pos = positionOf[stateId];
        if (pos != -1) {
            if (p < priority[pos]) {
                priority[pos] = p;
                g[pos] = gValue;
                y[pos] = yValue;
                counter[pos] = nextCounter++;
                siftUp(pos); // priority only decreased -- never needs siftDown
            }
            return;
        }
        if (size == priority.length) {
            grow();
        }
        int i = size++;
        priority[i] = p;
        g[i] = gValue;
        y[i] = yValue;
        counter[i] = nextCounter++;
        id[i] = stateId;
        positionOf[stateId] = i;
        siftUp(i);
    }

    /** Caller must check !isEmpty() first. Returns the popped state's id. */
    public int popId() {
        int top = id[0];
        positionOf[top] = -1;
        int last = --size;
        priority[0] = priority[last];
        g[0] = g[last];
        y[0] = y[last];
        counter[0] = counter[last];
        id[0] = id[last];
        if (size > 0) {
            positionOf[id[0]] = 0;
            siftDown(0);
        }
        return top;
    }

    /**
     * Tie-break chain: priority, then HIGHER y, then larger g, then
     * insertion order. The y tiebreak reflects a real domain heuristic: for
     * similar terrain, higher ground is almost always at least as good as
     * lower ground, since falling back down is cheap (a FALL action) but
     * climbing back up never is -- so among otherwise-tied candidates,
     * explore the higher one first.
     */
    private boolean less(int a, int b) {
        if (priority[a] != priority[b]) {
            return priority[a] < priority[b];
        }
        if (y[a] != y[b]) {
            return y[a] > y[b]; // prefer higher y
        }
        if (g[a] != g[b]) {
            return g[a] > g[b]; // prefer larger g (deeper node) on tied priority
        }
        return counter[a] < counter[b];
    }

    private void swap(int a, int b) {
        double tp = priority[a]; priority[a] = priority[b]; priority[b] = tp;
        double tg = g[a]; g[a] = g[b]; g[b] = tg;
        int ty = y[a]; y[a] = y[b]; y[b] = ty;
        long tc = counter[a]; counter[a] = counter[b]; counter[b] = tc;
        int ti = id[a]; id[a] = id[b]; id[b] = ti;
        positionOf[id[a]] = a;
        positionOf[id[b]] = b;
    }

    private void siftUp(int i) {
        while (i > 0) {
            int parent = (i - 1) >>> 1;
            if (less(i, parent)) {
                swap(i, parent);
                i = parent;
            } else {
                break;
            }
        }
    }

    private void siftDown(int i) {
        while (true) {
            int left = 2 * i + 1;
            int right = left + 1;
            int smallest = i;
            if (left < size && less(left, smallest)) smallest = left;
            if (right < size && less(right, smallest)) smallest = right;
            if (smallest == i) break;
            swap(i, smallest);
            i = smallest;
        }
    }

    private void grow() {
        int newCap = priority.length + (priority.length >> 1) + 1;
        priority = java.util.Arrays.copyOf(priority, newCap);
        g = java.util.Arrays.copyOf(g, newCap);
        y = java.util.Arrays.copyOf(y, newCap);
        counter = java.util.Arrays.copyOf(counter, newCap);
        id = java.util.Arrays.copyOf(id, newCap);
    }
}
