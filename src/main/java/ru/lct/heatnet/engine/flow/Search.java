package ru.lct.heatnet.engine.flow;

import java.util.Arrays;

/**
 * Дейкстра по графу видимости с динамическим источником (произвольная точка плоскости).
 * Массивы переиспользуются между запросами: сброс за O(1) через эпоху, куча индексная с
 * decrease-key. Итератор {@link #next()} отдаёт вершины в порядке неубывания расстояния —
 * вызывающий сам решает, когда остановиться (оценка снизу уже хуже найденного кандидата).
 */
final class Search {

    interface LegFilter {
        boolean allowed(double ax, double ay, double bx, double by);
    }

    final VisGraph g;
    final int source;
    final double[] dist;
    final int[] prev;
    private final int[] seen;
    private final boolean[] done;
    private final int[] heap;
    private final int[] pos;
    private int heapSize;
    private int epoch;
    final boolean[] blocked;
    private LegFilter filter;
    /** Запомненный ответ фильтра по ребру CSR: 0 — не спрашивали, 1 — можно, 2 — нельзя. */
    private byte[] memo;
    private byte[] memoBuf;
    private double srcX;
    private double srcY;
    private boolean srcTap;

    Search(VisGraph g) {
        this.g = g;
        this.source = g.n;
        int n = g.n + 1;
        dist = new double[n];
        prev = new int[n];
        seen = new int[n];
        done = new boolean[n];
        heap = new int[n];
        pos = new int[n];
        blocked = new boolean[g.n];
    }

    void filter(LegFilter f) {
        this.filter = f;
    }

    /**
     * Ответы фильтра по рёбрам графа запоминаются до выключения: пока лес не меняется, серия
     * запусков проверяет каждое ребро не больше одного раза.
     */
    void memoize(boolean on) {
        if (!on) {
            memo = null;
            return;
        }
        if (memoBuf == null) {
            memoBuf = new byte[g.adj.length];
        } else {
            Arrays.fill(memoBuf, (byte) 0);
        }
        memo = memoBuf;
    }

    private boolean edgeAllowed(int k, int v, int u) {
        if (memo == null) {
            return filter.allowed(g.x[v], g.y[v], g.x[u], g.y[u]);
        }
        byte s = memo[k];
        if (s == 0) {
            s = filter.allowed(g.x[v], g.y[v], g.x[u], g.y[u]) ? (byte) 1 : (byte) 2;
            memo[k] = s;
        }
        return s == 1;
    }

    /** Источник — точка (x,y) со стартовой стоимостью init (метры). */
    void start(double x, double y, boolean tapPoint, double init) {
        reset();
        srcX = x;
        srcY = y;
        srcTap = tapPoint;
        touch(source);
        dist[source] = init;
        prev[source] = -1;
        push(source);
    }

    /** Несколько источников-вершин графа со своими стартовыми стоимостями. */
    void startNodes(int[] nodes, double[] init, int count) {
        reset();
        srcX = Double.NaN;
        srcY = Double.NaN;
        for (int i = 0; i < count; i++) {
            int v = nodes[i];
            touch(v);
            if (init[i] < dist[v]) {
                dist[v] = init[i];
                prev[v] = -1;
                if (pos[v] >= 0) {
                    up(pos[v]);
                } else {
                    push(v);
                }
            }
        }
    }

    double sourceX() {
        return srcX;
    }

    double sourceY() {
        return srcY;
    }

    private void reset() {
        epoch++;
        if (epoch == Integer.MAX_VALUE) {
            Arrays.fill(seen, 0);
            epoch = 1;
        }
        heapSize = 0;
    }

    private void touch(int v) {
        if (seen[v] != epoch) {
            seen[v] = epoch;
            dist[v] = Double.POSITIVE_INFINITY;
            prev[v] = -1;
            done[v] = false;
            pos[v] = -1;
        }
    }

    boolean reached(int v) {
        return seen[v] == epoch && done[v];
    }

    double distTo(int v) {
        return seen[v] == epoch ? dist[v] : Double.POSITIVE_INFINITY;
    }

    double x(int v) {
        return v == source ? srcX : g.x[v];
    }

    double y(int v) {
        return v == source ? srcY : g.y[v];
    }

    /** Следующая окончательная вершина или -1. */
    int next() {
        if (heapSize == 0) {
            return -1;
        }
        int v = pop();
        done[v] = true;
        if (v == source) {
            relaxFromSource();
        } else {
            double dv = dist[v];
            for (int k = g.off[v]; k < g.off[v + 1]; k++) {
                int u = g.adj[k];
                if (blocked[u]) {
                    continue;
                }
                touch(u);
                if (done[u]) {
                    continue;
                }
                double nd = dv + g.w[k];
                if (nd < dist[u] - 1e-9) {
                    if (filter != null && !edgeAllowed(k, v, u)) {
                        continue;
                    }
                    dist[u] = nd;
                    prev[u] = v;
                    if (pos[u] >= 0) {
                        up(pos[u]);
                    } else {
                        push(u);
                    }
                }
            }
        }
        return v;
    }

    private void relaxFromSource() {
        double d0 = dist[source];
        for (int u = 0; u < g.n; u++) {
            if (blocked[u]) {
                continue;
            }
            if (!g.tangentAt(u, srcX, srcY)) {
                continue;
            }
            double c = g.legCost(srcX, srcY, g.x[u], g.y[u], srcTap, false);
            if (Double.isNaN(c)) {
                continue;
            }
            if (filter != null && !filter.allowed(srcX, srcY, g.x[u], g.y[u])) {
                continue;
            }
            touch(u);
            double nd = d0 + c;
            if (nd < dist[u] - 1e-9) {
                dist[u] = nd;
                prev[u] = source;
                if (pos[u] >= 0) {
                    up(pos[u]);
                } else {
                    push(u);
                }
            }
        }
    }

    /** Ломаная от источника до вершины v (x,y пары). */
    double[] path(int v) {
        int count = 0;
        for (int u = v; u >= 0; u = prev[u]) {
            count++;
            if (u == source) {
                break;
            }
        }
        double[] out = new double[count * 2];
        int i = count - 1;
        for (int u = v; u >= 0 && i >= 0; u = prev[u], i--) {
            out[i * 2] = x(u);
            out[i * 2 + 1] = y(u);
            if (u == source) {
                break;
            }
        }
        return out;
    }

    /** Вершина-исток пути до v (для многоисточникового запуска). */
    int origin(int v) {
        int u = v;
        while (u >= 0 && prev[u] >= 0) {
            u = prev[u];
        }
        return u;
    }

    private void push(int v) {
        int i = heapSize++;
        heap[i] = v;
        pos[v] = i;
        up(i);
    }

    private int pop() {
        int top = heap[0];
        pos[top] = -1;
        heapSize--;
        if (heapSize > 0) {
            int last = heap[heapSize];
            heap[0] = last;
            pos[last] = 0;
            down(0);
        }
        return top;
    }

    private void up(int i) {
        int v = heap[i];
        double d = dist[v];
        while (i > 0) {
            int p = (i - 1) >>> 1;
            int pv = heap[p];
            if (dist[pv] <= d) {
                break;
            }
            heap[i] = pv;
            pos[pv] = i;
            i = p;
        }
        heap[i] = v;
        pos[v] = i;
    }

    private void down(int i) {
        int v = heap[i];
        double d = dist[v];
        int half = heapSize >>> 1;
        while (i < half) {
            int c = 2 * i + 1;
            int cv = heap[c];
            int r = c + 1;
            if (r < heapSize && dist[heap[r]] < dist[cv]) {
                c = r;
                cv = heap[c];
            }
            if (d <= dist[cv]) {
                break;
            }
            heap[i] = cv;
            pos[cv] = i;
            i = c;
        }
        heap[i] = v;
        pos[v] = i;
    }
}
