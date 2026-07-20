package dev.mcpathfind.core;

/**
 * Hand-rolled binary min-heap over parallel primitive arrays -- no boxing,
 * no per-push object allocation (the Python equivalent, heapq over PQItem
 * dataclass instances, allocates one PQItem per push).
 *
 * Ordering: primary key priority ascending (standard weighted-A* order);
 * ties broken by g DESCENDING (prefer the deeper/larger-g node), a well-
 * established trick that reduces re-expansions in weighted A* by pushing
 * the frontier forward instead of re-exploring shallow nodes on tied
 * priorities; final tiebreak is insertion order (counter ascending), for
 * full determinism on the rare case priority AND g both match exactly.
 * (Earlier version of this heap only compared (priority, counter), matching
 * Python's heapq + insertion-counter idiom -- the g-based tiebreak is a
 * deliberate improvement over that, not a port of anything in pathfind.py.)
 */
public final class LongDoubleOpenHeap {
    private double[] priority;
    private double[] g;
    private long[] counter;
    private long[] state;
    private int size;
    private long nextCounter;

    public LongDoubleOpenHeap(int initialCapacity) {
        priority = new double[initialCapacity];
        g = new double[initialCapacity];
        counter = new long[initialCapacity];
        state = new long[initialCapacity];
        size = 0;
        nextCounter = 0;
    }

    public int size() {
        return size;
    }

    public boolean isEmpty() {
        return size == 0;
    }

    /** Caller must check !isEmpty() first. Used by bidirectional search's meet-in-the-middle termination check. */
    public double peekPriority() {
        return priority[0];
    }

    public void push(double p, double gValue, long s) {
        if (size == priority.length) {
            grow();
        }
        int i = size++;
        priority[i] = p;
        g[i] = gValue;
        counter[i] = nextCounter++;
        state[i] = s;
        siftUp(i);
    }

    /** Caller must check !isEmpty() first. Returns the popped state; priority/g/counter are discarded. */
    public long popState() {
        long top = state[0];
        int last = --size;
        priority[0] = priority[last];
        g[0] = g[last];
        counter[0] = counter[last];
        state[0] = state[last];
        if (size > 0) {
            siftDown(0);
        }
        return top;
    }

    private boolean less(int a, int b) {
        if (priority[a] != priority[b]) {
            return priority[a] < priority[b];
        }
        if (g[a] != g[b]) {
            return g[a] > g[b]; // prefer larger g (deeper node) on tied priority
        }
        return counter[a] < counter[b];
    }

    private void swap(int a, int b) {
        double tp = priority[a]; priority[a] = priority[b]; priority[b] = tp;
        double tg = g[a]; g[a] = g[b]; g[b] = tg;
        long tc = counter[a]; counter[a] = counter[b]; counter[b] = tc;
        long ts = state[a]; state[a] = state[b]; state[b] = ts;
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
        counter = java.util.Arrays.copyOf(counter, newCap);
        state = java.util.Arrays.copyOf(state, newCap);
    }
}
