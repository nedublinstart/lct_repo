package ru.lct.heatnet.engine.flow;

import java.util.Arrays;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Polygon;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Сокращённый граф видимости: вершины — выпуклые углы запретов (с зазором) и выходы ИТП на фасад,
 * рёбра — только битангенты (ребро касается препятствия в обоих концах). Кратчайший путь среди
 * многоугольников проходит только по таким рёбрам, поэтому сокращение не теряет оптимума.
 * Хранение CSR: {@code off[n+1]}, {@code adj[m]}, {@code w[m]} — O(V + E) памяти без объектов на ребро.
 */
final class VisGraph {

    private static final Logger log = LoggerFactory.getLogger(VisGraph.class);

    static final int CORNER = 0;
    static final int PORT = 1;
    /** Редкая точка в свободном поле: на большом городе связывает пустые промежутки. */
    static final int LATTICE = 2;
    /**
     * Ниже этого числа вершин граф полный: каждая допустимая битангента.
     * Выше — связи в радиусе {@link #LINK_M}, цепочки углов вдоль контура и редкая сетка.
     * Конкурсный набор остаётся в полном графе.
     */
    static int exactVertexLimit = 8_000;
    /** Радиус локальной битангенты на большом городе, м. */
    static final double LINK_M = 500;
    /** Шаг свободной сетки, м. */
    static final double LATTICE_M = 400;
    /** Потолок рёбер, чтобы список не вышел за память контейнера 12 ГБ. */
    static final int MAX_EDGES = 20_000_000;

    final FreeSpace space;
    int n;
    /** Индекс первого выхода ИТП; выходы идут подряд в порядке {@link Ports}. */
    int portBase;
    /** Открытая адресация: координаты вершины → индекс + 1. */
    private int[] slots;
    double[] x;
    double[] y;
    int[] kind;
    int[] owner;
    /** Соседи угла по кольцу (для проверки касания); NaN у выходов ИТП. */
    double[] px;
    double[] py;
    double[] qx;
    double[] qy;
    int[] off;
    int[] adj;
    double[] w;

    VisGraph(FreeSpace space) {
        this.space = space;
    }

    static VisGraph build(FreeSpace space, double[] portX, double[] portY, int[] portOwner) {
        VisGraph g = new VisGraph(space);
        Growable nodes = new Growable();
        addPolygonCorners(space, space.hard, nodes);
        for (FreeSpace.Zone z : space.zones) {
            addPolygonCorners(space, z.cornerBand, nodes);
        }
        boolean local = nodes.size + portX.length > exactVertexLimit;
        if (local) {
            addLattice(space, nodes);
        }
        g.portBase = nodes.size;
        for (int i = 0; i < portX.length; i++) {
            nodes.add(portX[i], portY[i], Double.NaN, Double.NaN, Double.NaN, Double.NaN, PORT, portOwner[i]);
        }
        g.n = nodes.size;
        g.x = Arrays.copyOf(nodes.x, g.n);
        g.y = Arrays.copyOf(nodes.y, g.n);
        g.px = Arrays.copyOf(nodes.px, g.n);
        g.py = Arrays.copyOf(nodes.py, g.n);
        g.qx = Arrays.copyOf(nodes.qx, g.n);
        g.qy = Arrays.copyOf(nodes.qy, g.n);
        g.kind = Arrays.copyOf(nodes.kind, g.n);
        g.owner = Arrays.copyOf(nodes.owner, g.n);
        g.index();
        if (local) {
            log.info("Крупная сцена: {} вершин, связи до {} м, сетка {} м", g.n, (int) LINK_M, (int) LATTICE_M);
            g.connectLocal(nodes.linkA, nodes.linkB, nodes.links);
        } else {
            g.connectAll();
        }
        return g;
    }

    private void index() {
        int cap = Integer.highestOneBit(Math.max(4, n * 2)) << 1;
        slots = new int[cap];
        for (int i = 0; i < n; i++) {
            int h = hash(x[i], y[i]) & (cap - 1);
            while (slots[h] != 0) {
                if (x[slots[h] - 1] == x[i] && y[slots[h] - 1] == y[i]) {
                    break;
                }
                h = (h + 1) & (cap - 1);
            }
            if (slots[h] == 0) {
                slots[h] = i + 1;
            }
        }
    }

    private static int hash(double px, double py) {
        long b = Double.doubleToLongBits(px) * 0x9E3779B97F4A7C15L ^ Double.doubleToLongBits(py);
        b ^= b >>> 31;
        return (int) (b ^ (b >>> 29));
    }

    /** Вершина графа с точно такими координатами или -1. */
    int nodeAt(double px, double py) {
        int cap = slots.length;
        int h = hash(px, py) & (cap - 1);
        while (slots[h] != 0) {
            int i = slots[h] - 1;
            if (x[i] == px && y[i] == py) {
                return i;
            }
            h = (h + 1) & (cap - 1);
        }
        return -1;
    }

    private static void addPolygonCorners(FreeSpace space, Geometry g, Growable out) {
        if (g == null) {
            return;
        }
        for (int i = 0; i < g.getNumGeometries(); i++) {
            Geometry part = g.getGeometryN(i);
            if (!(part instanceof Polygon)) {
                continue;
            }
            Polygon p = (Polygon) part;
            addConvexCorners(space, FreeSpace.coords(p.getExteriorRing().getCoordinates()), out, false);
            for (int h = 0; h < p.getNumInteriorRing(); h++) {
                addConvexCorners(space, FreeSpace.coords(p.getInteriorRingN(h).getCoordinates()), out, true);
            }
        }
    }

    /** Углы, которые обходит кратчайший путь: выпуклые у оболочки, вогнутые у дыры. */
    private static void addConvexCorners(FreeSpace space, double[] ring, Growable out, boolean hole) {
        int m = ring.length / 2 - 1;
        if (m < 3) {
            return;
        }
        double area = 0;
        for (int i = 0; i < m; i++) {
            int j = (i + 1) % m;
            area += ring[i * 2] * ring[j * 2 + 1] - ring[j * 2] * ring[i * 2 + 1];
        }
        if (hole) {
            area = -area;
        }
        int prevKept = -1;
        int firstKept = -1;
        for (int i = 0; i < m; i++) {
            int a = (i + m - 1) % m;
            int c = (i + 1) % m;
            double ax = ring[a * 2];
            double ay = ring[a * 2 + 1];
            double bx = ring[i * 2];
            double by = ring[i * 2 + 1];
            double cx = ring[c * 2];
            double cy = ring[c * 2 + 1];
            double turn = Geo.cross(ax, ay, bx, by, cx, cy);
            boolean convex = area > 0 ? turn > 1e-9 : turn < -1e-9;
            if (!convex) {
                continue;
            }
            if (!space.roi.contains(bx, by)) {
                continue;
            }
            if (!space.nodeFree(bx, by)) {
                continue;
            }
            int id = out.add(bx, by, ax, ay, cx, cy, CORNER, -1);
            if (prevKept >= 0) {
                out.link(prevKept, id);
            } else {
                firstKept = id;
            }
            prevKept = id;
        }
        if (firstKept >= 0 && prevKept >= 0 && firstKept != prevKept) {
            out.link(prevKept, firstKept);
        }
    }

    /** Точки в пустом поле, чтобы длинный пустырь не разрывал граф. */
    private static void addLattice(FreeSpace space, Growable out) {
        Envelope roi = space.roi;
        if (roi == null || roi.isNull()) {
            return;
        }
        double w = Math.max(1, roi.getWidth());
        double h = Math.max(1, roi.getHeight());
        double step = LATTICE_M;
        if (w * h / (step * step) > 20_000) {
            step = Math.sqrt(w * h / 20_000.0);
        }
        double x0 = Math.floor(roi.getMinX() / step) * step;
        double y0 = Math.floor(roi.getMinY() / step) * step;
        for (double x = x0; x <= roi.getMaxX(); x += step) {
            for (double y = y0; y <= roi.getMaxY(); y += step) {
                if (!space.nodeFree(x, y)) {
                    continue;
                }
                out.add(x, y, Double.NaN, Double.NaN, Double.NaN, Double.NaN, LATTICE, -1);
            }
        }
    }

    /** Касание в угле v: оба соседа по кольцу по одну сторону от прямой v→(tx,ty). */
    boolean tangentAt(int v, double tx, double ty) {
        if (kind[v] != CORNER) {
            return true;
        }
        double s1 = Geo.cross(x[v], y[v], tx, ty, px[v], py[v]);
        double s2 = Geo.cross(x[v], y[v], tx, ty, qx[v], qy[v]);
        return !(s1 > 1e-9 && s2 < -1e-9) && !(s1 < -1e-9 && s2 > 1e-9);
    }

    /** Эффективная длина (длина + надбавка спецпроходов) или NaN, если отрезок недопустим. */
    double legCost(double ax, double ay, double bx, double by, boolean tapA, boolean tapB) {
        if (!space.segmentFree(ax, ay, bx, by)) {
            return Double.NaN;
        }
        double extra = space.specialExtra(ax, ay, bx, by, tapA, tapB);
        if (Double.isNaN(extra)) {
            return Double.NaN;
        }
        return Geo.dist(ax, ay, bx, by) + extra;
    }

    private void connectAll() {
        int[] ea = new int[1024];
        int[] eb = new int[1024];
        double[] ew = new double[1024];
        int m = 0;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                if (!tangentAt(i, x[j], y[j]) || !tangentAt(j, x[i], y[i])) {
                    continue;
                }
                double c = legCost(x[i], y[i], x[j], y[j], false, false);
                if (Double.isNaN(c)) {
                    continue;
                }
                if (m == ea.length) {
                    ea = Arrays.copyOf(ea, m * 2);
                    eb = Arrays.copyOf(eb, m * 2);
                    ew = Arrays.copyOf(ew, m * 2);
                }
                ea[m] = i;
                eb[m] = j;
                ew[m] = c;
                m++;
            }
        }
        pack(ea, eb, ew, m);
    }

    /**
     * Большой город: битангенты короче {@link #LINK_M}, длинные стороны контура и сетка пустырей.
     * Полный перебор пар на сотнях тысяч углов не помещается в память контейнера.
     */
    private void connectLocal(int[] chainA, int[] chainB, int chainN) {
        int[] ea = new int[1024];
        int[] eb = new int[1024];
        double[] ew = new double[1024];
        int[] box = new int[]{0};
        int m = 0;
        for (int c = 0; c < chainN && m < MAX_EDGES; c++) {
            m = offer(ea, eb, ew, m, chainA[c], chainB[c], true, box);
            ea = grow(ea, m);
            eb = grow(eb, m);
            ew = growD(ew, m);
        }
        if (n > 0 && m < MAX_EDGES) {
            double minX = x[0];
            double minY = y[0];
            double maxX = x[0];
            double maxY = y[0];
            for (int i = 1; i < n; i++) {
                minX = Math.min(minX, x[i]);
                minY = Math.min(minY, y[i]);
                maxX = Math.max(maxX, x[i]);
                maxY = Math.max(maxY, y[i]);
            }
            double cell = Math.max(LINK_M, 1);
            int nx = Math.max(1, (int) Math.floor((maxX - minX) / cell) + 2);
            int ny = Math.max(1, (int) Math.floor((maxY - minY) / cell) + 2);
            if ((long) nx * ny > 2_000_000L) {
                cell = Math.max(LINK_M, Math.sqrt(Math.max(1, (maxX - minX) * (maxY - minY)) / 1_000_000.0));
                nx = Math.max(1, (int) Math.floor((maxX - minX) / cell) + 2);
                ny = Math.max(1, (int) Math.floor((maxY - minY) / cell) + 2);
            }
            int[] head = new int[nx * ny];
            Arrays.fill(head, -1);
            int[] next = new int[n];
            int[] gx = new int[n];
            int[] gy = new int[n];
            for (int i = 0; i < n; i++) {
                int ix = (int) Math.floor((x[i] - minX) / cell);
                int iy = (int) Math.floor((y[i] - minY) / cell);
                if (ix < 0) {
                    ix = 0;
                } else if (ix >= nx) {
                    ix = nx - 1;
                }
                if (iy < 0) {
                    iy = 0;
                } else if (iy >= ny) {
                    iy = ny - 1;
                }
                gx[i] = ix;
                gy[i] = iy;
                int k = iy * nx + ix;
                next[i] = head[k];
                head[k] = i;
            }
            double link2 = LINK_M * LINK_M;
            for (int i = 0; i < n && m < MAX_EDGES; i++) {
                int x0 = Math.max(0, gx[i] - 1);
                int x1 = Math.min(nx - 1, gx[i] + 1);
                int y0 = Math.max(0, gy[i] - 1);
                int y1 = Math.min(ny - 1, gy[i] + 1);
                for (int iy = y0; iy <= y1 && m < MAX_EDGES; iy++) {
                    for (int ix = x0; ix <= x1 && m < MAX_EDGES; ix++) {
                        for (int j = head[iy * nx + ix]; j >= 0 && m < MAX_EDGES; j = next[j]) {
                            if (j <= i) {
                                continue;
                            }
                            double dx = x[i] - x[j];
                            double dy = y[i] - y[j];
                            if (dx * dx + dy * dy > link2) {
                                continue;
                            }
                            m = offer(ea, eb, ew, m, i, j, false, box);
                            ea = grow(ea, m);
                            eb = grow(eb, m);
                            ew = growD(ew, m);
                        }
                    }
                }
            }
        }
        if (m >= MAX_EDGES) {
            log.info("Список рёбер остановлен на {} — дальше память контейнера", MAX_EDGES);
        }
        pack(ea, eb, ew, m);
    }

    private int offer(int[] ea, int[] eb, double[] ew, int m, int i, int j, boolean longChain, int[] box) {
        if (i < 0 || j < 0 || i == j || m >= MAX_EDGES || m >= ea.length) {
            return m;
        }
        if (longChain) {
            double dx = x[i] - x[j];
            double dy = y[i] - y[j];
            if (dx * dx + dy * dy <= LINK_M * LINK_M) {
                return m;
            }
        }
        if (!tangentAt(i, x[j], y[j]) || !tangentAt(j, x[i], y[i])) {
            return m;
        }
        double c = legCost(x[i], y[i], x[j], y[j], false, false);
        if (Double.isNaN(c)) {
            return m;
        }
        ea[m] = i;
        eb[m] = j;
        ew[m] = c;
        box[0] = m + 1;
        return m + 1;
    }

    private static int[] grow(int[] a, int m) {
        return m == a.length ? Arrays.copyOf(a, m * 2) : a;
    }

    private static double[] growD(double[] a, int m) {
        return m == a.length ? Arrays.copyOf(a, m * 2) : a;
    }

    private void pack(int[] ea, int[] eb, double[] ew, int m) {
        off = new int[n + 1];
        for (int k = 0; k < m; k++) {
            off[ea[k] + 1]++;
            off[eb[k] + 1]++;
        }
        for (int i = 0; i < n; i++) {
            off[i + 1] += off[i];
        }
        adj = new int[2 * m];
        w = new double[2 * m];
        int[] fill = Arrays.copyOf(off, n);
        for (int k = 0; k < m; k++) {
            int a = ea[k];
            int b = eb[k];
            adj[fill[a]] = b;
            w[fill[a]++] = ew[k];
            adj[fill[b]] = a;
            w[fill[b]++] = ew[k];
        }
    }

    int edgeCount() {
        return adj.length / 2;
    }

    private static final class Growable {
        double[] x = new double[256];
        double[] y = new double[256];
        double[] px = new double[256];
        double[] py = new double[256];
        double[] qx = new double[256];
        double[] qy = new double[256];
        int[] kind = new int[256];
        int[] owner = new int[256];
        int[] linkA = new int[256];
        int[] linkB = new int[256];
        int size;
        int links;

        int add(double vx, double vy, double ax, double ay, double cx, double cy, int k, int o) {
            if (size == x.length) {
                int cap = size * 2;
                x = Arrays.copyOf(x, cap);
                y = Arrays.copyOf(y, cap);
                px = Arrays.copyOf(px, cap);
                py = Arrays.copyOf(py, cap);
                qx = Arrays.copyOf(qx, cap);
                qy = Arrays.copyOf(qy, cap);
                kind = Arrays.copyOf(kind, cap);
                owner = Arrays.copyOf(owner, cap);
            }
            x[size] = vx;
            y[size] = vy;
            px[size] = ax;
            py[size] = ay;
            qx[size] = cx;
            qy[size] = cy;
            kind[size] = k;
            owner[size] = o;
            return size++;
        }

        void link(int a, int b) {
            if (links == linkA.length) {
                linkA = Arrays.copyOf(linkA, links * 2);
                linkB = Arrays.copyOf(linkB, links * 2);
            }
            linkA[links] = a;
            linkB[links] = b;
            links++;
        }
    }
}
