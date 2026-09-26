package ru.lct.heatnet.engine.flow;

import java.util.Arrays;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Polygon;

/**
 * Сокращённый граф видимости: вершины — выпуклые углы запретов (с зазором) и выходы ИТП на фасад,
 * рёбра — только битангенты (ребро касается препятствия в обоих концах). Кратчайший путь среди
 * многоугольников проходит только по таким рёбрам, поэтому сокращение не теряет оптимума.
 * Хранение CSR: {@code off[n+1]}, {@code adj[m]}, {@code w[m]} — O(V + E) памяти без объектов на ребро.
 */
final class VisGraph {

    static final int CORNER = 0;
    static final int PORT = 1;

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
        g.connectAll();
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
            out.add(bx, by, ax, ay, cx, cy, CORNER, -1);
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
        int size;

        void add(double vx, double vy, double ax, double ay, double cx, double cy, int k, int o) {
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
            size++;
        }
    }
}
