package ru.lct.heatnet.engine.flow;

import java.util.Arrays;

/**
 * Оптимальные деревья для группы ОКС внутри окна графа видимости: динамика Дрейфуса — Вагнера по
 * подмножествам группы.
 * <p>
 * {@code a[S][v]} — самое дешёвое дерево, соединяющее ОКС множества S, верхний участок которого
 * приходит в вершину v; {@code m2[S][v]} — два поддерева, сходящиеся в v, без камеры. Камера в v
 * принимает два или три нижних участка и стоит по DN суммарного расхода S; метр трубы выше неё —
 * по тому же DN плюс вес длины цели. {@code a[S]} получается из камер одной Дейкстрой с многими источниками.
 * <p>
 * Время O(3^k · w + 2^k · (e + w log w)), память O(2^k · w): k ≤ {@link #MAX_UNITS} ОКС, w и e —
 * вершины и рёбра окна. Массивы переиспользуются между запусками.
 */
final class SubsetDp {

    static final int MAX_UNITS = 8;
    private static final double MARGIN_M = 120;
    private static final double INF = Double.POSITIVE_INFINITY;

    private final VisGraph g;
    private final Search search;
    private final Ports ports;
    private final Prices prices;
    /** Вершина графа → индекс окна или −1; вне запуска все −1. */
    private final int[] winIdx;
    private int[] outside = new int[256];

    int k;
    int[] terms;
    /**
     * Рубли за метр сверх цены трубы: вес длины в S. Динамика тогда минимизирует ту же цель,
     * что и смета, а не только стоимость трубы и камеры.
     */
    double meterAdd;
    double[] flow = new double[0];
    double[] price = new double[0];
    double[] chamber = new double[0];
    int w;
    int[] win = new int[256];
    double[] a = new double[0];
    double[] m2 = new double[0];
    /** Предыдущая вершина пути a[S] (индекс окна) или −1 в начале пути. */
    int[] prev = new int[0];
    /** Разбиение для m2[S][v]: одна из двух частей. */
    int[] s2 = new int[0];
    /** Камера на три участка: часть из двух поддеревьев (разбита по s2) или −1, если камера на два. */
    int[] s3 = new int[0];
    private double[] m3 = new double[0];
    private int[] startBuf = new int[64];
    private double[] initBuf = new double[64];

    SubsetDp(VisGraph g, Search search, Ports ports, Prices prices) {
        this.g = g;
        this.search = search;
        this.ports = ports;
        this.prices = prices;
        winIdx = new int[g.n];
        Arrays.fill(winIdx, -1);
    }

    /**
     * Динамика для ОКС terms. Занятые лесом вершины и выходы ИТП вызывающий уже закрыл в
     * {@code search.blocked} и выставил фильтр рёбер; вершины вне окна закрываются здесь на время
     * расчёта. Индексы окна действуют до {@link #release()}. false — какой-то ОКС группы не выходит
     * в окно.
     */
    boolean run(int[] terms) {
        release();
        this.terms = terms;
        k = terms.length;
        int subsets = 1 << k;
        if (flow.length < subsets) {
            flow = new double[subsets];
            price = new double[subsets];
            chamber = new double[subsets];
        }
        for (int s = 1; s < subsets; s++) {
            int low = Integer.numberOfTrailingZeros(s);
            flow[s] = flow[s & (s - 1)] + ports.terms.get(terms[low]).flow;
            int dn = prices.dnFor(flow[s]);
            price[s] = prices.perM(dn) + meterAdd;
            chamber[s] = prices.chamber(dn);
        }
        int closed = window();
        try {
            ensure(subsets * w);
            for (int i = 0; i < k; i++) {
                Ports.Terminal t = ports.terms.get(terms[i]);
                growStarts(t.count);
                for (int p = 0; p < t.count; p++) {
                    startBuf[p] = g.portBase + t.first + p;
                    initBuf[p] = ports.stub[t.first + p];
                }
                search.startNodes(startBuf, initBuf, t.count);
                drain(1 << i);
                if (!reachesWindow(1 << i)) {
                    return false;
                }
            }
            for (int s = 3; s < subsets; s++) {
                if ((s & (s - 1)) != 0) {
                    merge(s);
                }
            }
            return true;
        } finally {
            for (int i = 0; i < closed; i++) {
                search.blocked[outside[i]] = false;
            }
        }
    }

    void release() {
        for (int j = 0; j < w; j++) {
            winIdx[win[j]] = -1;
        }
        w = 0;
    }

    /** Вершины окна: прямоугольник ОКС группы с запасом; свободные вершины и выходы ИТП группы. */
    private int window() {
        double minX = INF;
        double minY = INF;
        double maxX = -INF;
        double maxY = -INF;
        boolean[] own = new boolean[ports.terms.size()];
        for (int t : terms) {
            Ports.Terminal term = ports.terms.get(t);
            own[t] = true;
            minX = Math.min(minX, term.x);
            minY = Math.min(minY, term.y);
            maxX = Math.max(maxX, term.x);
            maxY = Math.max(maxY, term.y);
        }
        minX -= MARGIN_M;
        minY -= MARGIN_M;
        maxX += MARGIN_M;
        maxY += MARGIN_M;
        w = 0;
        int closed = 0;
        for (int v = 0; v < g.n; v++) {
            boolean inside = g.x[v] >= minX && g.x[v] <= maxX && g.y[v] >= minY && g.y[v] <= maxY;
            boolean ownPort = g.kind[v] == VisGraph.PORT && own[g.owner[v]];
            if (inside && (!search.blocked[v] || ownPort)) {
                if (w == win.length) {
                    win = Arrays.copyOf(win, w * 2);
                }
                winIdx[v] = w;
                win[w++] = v;
            } else if (!search.blocked[v]) {
                if (closed == outside.length) {
                    outside = Arrays.copyOf(outside, closed * 2);
                }
                outside[closed++] = v;
                search.blocked[v] = true;
            }
        }
        return closed;
    }

    private void ensure(int size) {
        if (a.length < size) {
            int cap = Math.max(size, a.length * 3 / 2);
            a = new double[cap];
            m2 = new double[cap];
            prev = new int[cap];
            s2 = new int[cap];
            s3 = new int[cap];
        }
        if (m3.length < w) {
            m3 = new double[Math.max(w, m3.length * 3 / 2)];
        }
        growStarts(w);
    }

    private void growStarts(int n) {
        if (startBuf.length < n) {
            startBuf = new int[n];
            initBuf = new double[n];
        }
    }

    /** Две или три части S сходятся в камере; затем трубы по DN расхода S от камер. */
    private void merge(int s) {
        int base = s * w;
        Arrays.fill(m2, base, base + w, INF);
        Arrays.fill(m3, 0, w, INF);
        Arrays.fill(s3, base, base + w, -1);
        int low = s & -s;
        for (int s1 = (s - 1) & s; s1 > 0; s1 = (s1 - 1) & s) {
            int b1 = s1 * w;
            int br = (s ^ s1) * w;
            if ((s1 & low) != 0) {
                for (int j = 0; j < w; j++) {
                    double v = a[b1 + j] + a[br + j];
                    if (v < m2[base + j]) {
                        m2[base + j] = v;
                        s2[base + j] = s1;
                    }
                }
            }
            if ((s1 & (s1 - 1)) != 0) {
                for (int j = 0; j < w; j++) {
                    double v = m2[b1 + j] + a[br + j];
                    if (v < m3[j]) {
                        m3[j] = v;
                        s3[base + j] = s1;
                    }
                }
            }
        }
        double ps = price[s];
        int count = 0;
        for (int j = 0; j < w; j++) {
            double c = m2[base + j];
            if (m3[j] < c) {
                c = m3[j];
            } else {
                s3[base + j] = -1;
            }
            if (c < INF && g.kind[win[j]] != VisGraph.PORT) {
                startBuf[count] = win[j];
                initBuf[count] = (c + chamber[s]) / ps;
                count++;
            }
        }
        search.startNodes(startBuf, initBuf, count);
        drain(s);
    }

    private void drain(int s) {
        while (search.next() >= 0) {
            // до исчерпания
        }
        int base = s * w;
        double ps = price[s];
        for (int j = 0; j < w; j++) {
            int v = win[j];
            double d = search.distTo(v);
            a[base + j] = d < INF ? d * ps : INF;
            int p = d < INF ? search.prev[v] : -1;
            prev[base + j] = p >= 0 && p < g.n ? winIdx[p] : -1;
        }
    }

    private boolean reachesWindow(int s) {
        int base = s * w;
        for (int j = 0; j < w; j++) {
            if (a[base + j] < INF) {
                return true;
            }
        }
        return false;
    }

    /** Индекс окна вершины графа или −1. */
    int index(int v) {
        return v >= 0 && v < winIdx.length ? winIdx[v] : -1;
    }

    /** Путь дерева a[S] до вершины окна j: индексы окна от начала (камера или выход ИТП) до j. */
    int[] chain(int s, int j) {
        int base = s * w;
        int count = 1;
        for (int u = j; prev[base + u] >= 0; u = prev[base + u]) {
            count++;
        }
        int[] out = new int[count];
        int u = j;
        for (int i = count - 1; i >= 0; i--) {
            out[i] = u;
            u = prev[base + u];
        }
        return out;
    }
}
