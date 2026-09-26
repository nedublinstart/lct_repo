package ru.lct.heatnet.engine.flow;

import java.util.Arrays;

/**
 * Равномерная сетка над набором отрезков: CSR-массивы {@code cellStart/cellItems}, координаты в одном
 * {@code double[]}. Память O(E + C), обход луча — DDA по клеткам (Amanatides–Woo), каждый отрезок
 * проверяется один раз за запрос за счёт штампа.
 */
final class EdgeGrid {

    final double[] xy;
    final int size;
    final int[] owner;
    private final double minX;
    private final double minY;
    private final double cell;
    private final int nx;
    private final int ny;
    private final int[] cellStart;
    private final int[] cellItems;
    private final int[] stamp;
    private int epoch;

    EdgeGrid(double[] xy, int[] owner, int count, double cellHint) {
        this(xy, owner, count, cellHint, 0);
    }

    /** {@code extraPad} — запас вокруг отрезков сверх одной клетки (для запроса с допуском). */
    EdgeGrid(double[] xy, int[] owner, int count, double cellHint, double extraPad) {
        this.xy = xy;
        this.owner = owner;
        this.size = count;
        this.stamp = new int[Math.max(1, count)];
        double x0 = Double.POSITIVE_INFINITY;
        double y0 = Double.POSITIVE_INFINITY;
        double x1 = Double.NEGATIVE_INFINITY;
        double y1 = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < count; i++) {
            int o = i * 4;
            x0 = Math.min(x0, Math.min(xy[o], xy[o + 2]));
            y0 = Math.min(y0, Math.min(xy[o + 1], xy[o + 3]));
            x1 = Math.max(x1, Math.max(xy[o], xy[o + 2]));
            y1 = Math.max(y1, Math.max(xy[o + 1], xy[o + 3]));
        }
        if (count == 0) {
            x0 = y0 = 0;
            x1 = y1 = 1;
        }
        double w = Math.max(1, x1 - x0);
        double h = Math.max(1, y1 - y0);
        double c = Math.max(cellHint, Math.sqrt(w * h / 4_000_000.0));
        this.cell = c;
        double pad = c + Math.max(0, extraPad);
        this.minX = x0 - pad;
        this.minY = y0 - pad;
        this.nx = (int) Math.ceil((w + 2 * pad) / c) + 1;
        this.ny = (int) Math.ceil((h + 2 * pad) / c) + 1;
        int cells = nx * ny;
        int[] counts = new int[cells + 1];
        for (int i = 0; i < count; i++) {
            forEachCell(i, counts, null);
        }
        cellStart = new int[cells + 1];
        for (int k = 0; k < cells; k++) {
            cellStart[k + 1] = cellStart[k] + counts[k];
        }
        cellItems = new int[cellStart[cells]];
        int[] fill = Arrays.copyOf(cellStart, cells);
        for (int i = 0; i < count; i++) {
            forEachCell(i, null, fill);
        }
    }

    private void forEachCell(int edge, int[] counts, int[] fill) {
        int o = edge * 4;
        int ax = cx(Math.min(xy[o], xy[o + 2]));
        int bx = cx(Math.max(xy[o], xy[o + 2]));
        int ay = cy(Math.min(xy[o + 1], xy[o + 3]));
        int by = cy(Math.max(xy[o + 1], xy[o + 3]));
        for (int j = ay; j <= by; j++) {
            for (int i = ax; i <= bx; i++) {
                if (!boxTouches(edge, i, j)) {
                    continue;
                }
                int k = j * nx + i;
                if (counts != null) {
                    counts[k]++;
                } else {
                    cellItems[fill[k]++] = edge;
                }
            }
        }
    }

    private boolean boxTouches(int edge, int i, int j) {
        int o = edge * 4;
        double x0 = minX + i * cell - 1e-7;
        double y0 = minY + j * cell - 1e-7;
        double x1 = x0 + cell + 2e-7;
        double y1 = y0 + cell + 2e-7;
        return Geo.segmentTouchesBox(xy[o], xy[o + 1], xy[o + 2], xy[o + 3], x0, y0, x1, y1);
    }

    private int cx(double x) {
        return Math.max(0, Math.min(nx - 1, (int) ((x - minX) / cell)));
    }

    private int cy(double y) {
        return Math.max(0, Math.min(ny - 1, (int) ((y - minY) / cell)));
    }

    interface Visitor {
        /** @return {@code true}, чтобы остановить обход. */
        boolean visit(int edge);
    }

    /** Все отрезки в клетках, через которые проходит отрезок ab; обход прекращается, если visitor вернул true. */
    boolean walk(double ax, double ay, double bx, double by, Visitor visitor) {
        if (size == 0) {
            return false;
        }
        int e = ++epoch;
        if (e == Integer.MAX_VALUE) {
            Arrays.fill(stamp, 0);
            epoch = 1;
            e = 1;
        }
        double dx = bx - ax;
        double dy = by - ay;
        int i = cx(ax);
        int j = cy(ay);
        int ti = cx(bx);
        int tj = cy(by);
        int stepX = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
        int stepY = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
        double tMaxX = Double.POSITIVE_INFINITY;
        double tMaxY = Double.POSITIVE_INFINITY;
        double tDeltaX = Double.POSITIVE_INFINITY;
        double tDeltaY = Double.POSITIVE_INFINITY;
        if (stepX != 0) {
            double nextX = minX + (i + (stepX > 0 ? 1 : 0)) * cell;
            tMaxX = (nextX - ax) / dx;
            tDeltaX = cell / Math.abs(dx);
        }
        if (stepY != 0) {
            double nextY = minY + (j + (stepY > 0 ? 1 : 0)) * cell;
            tMaxY = (nextY - ay) / dy;
            tDeltaY = cell / Math.abs(dy);
        }
        int guard = nx + ny + 4;
        while (true) {
            int k = j * nx + i;
            for (int p = cellStart[k]; p < cellStart[k + 1]; p++) {
                int edge = cellItems[p];
                if (stamp[edge] == e) {
                    continue;
                }
                stamp[edge] = e;
                if (visitor.visit(edge)) {
                    return true;
                }
            }
            if ((i == ti && j == tj) || guard-- <= 0) {
                return false;
            }
            if (tMaxX < tMaxY) {
                if (tMaxX > 1.0) {
                    return false;
                }
                i += stepX;
                tMaxX += tDeltaX;
            } else {
                if (tMaxY > 1.0) {
                    return false;
                }
                j += stepY;
                tMaxY += tDeltaY;
            }
            if (i < 0 || j < 0 || i >= nx || j >= ny) {
                return false;
            }
        }
    }

    /** Центр клетки внутри контуров (для {@link #covers}); null — не построено. */
    private boolean[] centerIn;
    /** Штамп клеток коридора: каждая клетка запроса проверяется один раз. */
    private int[] cellStamp;
    private int[] cellQueue;
    private int cellEpoch;
    private final double[] clip = new double[4];
    private final double[] clipP = new double[4];
    private final double[] clipQ = new double[4];

    /** Отрезок, обрезанный прямоугольником сетки. false — весь отрезок снаружи. */
    private boolean clipSegment(double ax, double ay, double bx, double by) {
        double dx = bx - ax;
        double dy = by - ay;
        double t0 = 0;
        double t1 = 1;
        double maxX = minX + nx * cell;
        double maxY = minY + ny * cell;
        clipP[0] = -dx;
        clipP[1] = dx;
        clipP[2] = -dy;
        clipP[3] = dy;
        clipQ[0] = ax - minX;
        clipQ[1] = maxX - ax;
        clipQ[2] = ay - minY;
        clipQ[3] = maxY - ay;
        for (int s = 0; s < 4; s++) {
            double ps = clipP[s];
            double qs = clipQ[s];
            if (Math.abs(ps) < 1e-15) {
                if (qs < 0) {
                    return false;
                }
            } else {
                double t = qs / ps;
                if (ps < 0) {
                    if (t > t1) {
                        return false;
                    }
                    if (t > t0) {
                        t0 = t;
                    }
                } else if (t < t0) {
                    return false;
                } else if (t < t1) {
                    t1 = t;
                }
            }
        }
        clip[0] = ax + t0 * dx;
        clip[1] = ay + t0 * dy;
        clip[2] = ax + t1 * dx;
        clip[3] = ay + t1 * dy;
        return true;
    }

    /**
     * Отрезки ближе {@code margin} к отрезку ab. Клетки луча и соседние в радиусе
     * {@code ceil(margin / cell) + 1}: точка на расстоянии margin лежит не дальше этого числа клеток
     * от клетки луча. Каждый отрезок и каждая клетка — один раз за запрос.
     */
    boolean corridor(double ax, double ay, double bx, double by, double margin, Visitor visitor) {
        if (size == 0) {
            return false;
        }
        int cells = nx * ny;
        if (cellStamp == null) {
            cellStamp = new int[cells];
            cellQueue = new int[cells];
        }
        if (!clipSegment(ax, ay, bx, by)) {
            return false;
        }
        ax = clip[0];
        ay = clip[1];
        bx = clip[2];
        by = clip[3];
        int e = nextEpoch();
        int ce = ++cellEpoch;
        if (ce == Integer.MAX_VALUE) {
            Arrays.fill(cellStamp, 0);
            cellEpoch = 1;
            ce = 1;
        }
        int r = (int) Math.ceil(margin / cell) + 1;
        int nq = 0;
        double dx = bx - ax;
        double dy = by - ay;
        int i = cx(ax);
        int j = cy(ay);
        int ti = cx(bx);
        int tj = cy(by);
        int stepX = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
        int stepY = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
        double tMaxX = Double.POSITIVE_INFINITY;
        double tMaxY = Double.POSITIVE_INFINITY;
        double tDeltaX = Double.POSITIVE_INFINITY;
        double tDeltaY = Double.POSITIVE_INFINITY;
        if (stepX != 0) {
            double nextX = minX + (i + (stepX > 0 ? 1 : 0)) * cell;
            tMaxX = (nextX - ax) / dx;
            tDeltaX = cell / Math.abs(dx);
        }
        if (stepY != 0) {
            double nextY = minY + (j + (stepY > 0 ? 1 : 0)) * cell;
            tMaxY = (nextY - ay) / dy;
            tDeltaY = cell / Math.abs(dy);
        }
        int guard = nx + ny + 4;
        int prevI = i;
        int prevJ = j;
        boolean first = true;
        while (true) {
            if (first) {
                nq = enqueue(i - r, i + r, j - r, j + r, ce, nq);
                first = false;
            } else if (i != prevI) {
                int col = i + (i > prevI ? r : -r);
                nq = enqueue(col, col, j - r, j + r, ce, nq);
            } else if (j != prevJ) {
                int row = j + (j > prevJ ? r : -r);
                nq = enqueue(i - r, i + r, row, row, ce, nq);
            }
            prevI = i;
            prevJ = j;
            if ((i == ti && j == tj) || guard-- <= 0) {
                break;
            }
            if (tMaxX < tMaxY) {
                if (tMaxX > 1.0) {
                    break;
                }
                i += stepX;
                tMaxX += tDeltaX;
            } else {
                if (tMaxY > 1.0) {
                    break;
                }
                j += stepY;
                tMaxY += tDeltaY;
            }
            if (i < 0 || j < 0 || i >= nx || j >= ny) {
                break;
            }
        }
        for (int q = 0; q < nq; q++) {
            int k = cellQueue[q];
            for (int p = cellStart[k]; p < cellStart[k + 1]; p++) {
                int edge = cellItems[p];
                if (stamp[edge] == e) {
                    continue;
                }
                stamp[edge] = e;
                if (visitor.visit(edge)) {
                    return true;
                }
            }
        }
        return false;
    }

    /** Клетки прямоугольника в очередь, каждая один раз за эпоху ce. */
    private int enqueue(int i0, int i1, int j0, int j1, int ce, int nq) {
        i0 = Math.max(0, i0);
        i1 = Math.min(nx - 1, i1);
        j0 = Math.max(0, j0);
        j1 = Math.min(ny - 1, j1);
        for (int j = j0; j <= j1; j++) {
            int row = j * nx;
            for (int i = i0; i <= i1; i++) {
                int k = row + i;
                if (cellStamp[k] != ce) {
                    cellStamp[k] = ce;
                    cellQueue[nq++] = k;
                }
            }
        }
        return nq;
    }

    /**
     * Для отрезков, образующих замкнутые контуры: состояние «внутри» центров клеток по чётности
     * пересечений горизонтали через центры. O(C + E log E) один раз.
     */
    void indexInside() {
        centerIn = new boolean[nx * ny];
        double[] xs = new double[64];
        for (int j = 0; j < ny; j++) {
            double y = minY + (j + 0.5) * cell;
            int e = nextEpoch();
            int m = 0;
            for (int i = 0; i < nx; i++) {
                int k = j * nx + i;
                for (int p = cellStart[k]; p < cellStart[k + 1]; p++) {
                    int edge = cellItems[p];
                    if (stamp[edge] == e) {
                        continue;
                    }
                    stamp[edge] = e;
                    int o = edge * 4;
                    double y1 = xy[o + 1];
                    double y2 = xy[o + 3];
                    if ((y1 > y) != (y2 > y)) {
                        if (m == xs.length) {
                            xs = Arrays.copyOf(xs, m * 2);
                        }
                        xs[m++] = xy[o] + (y - y1) * (xy[o + 2] - xy[o]) / (y2 - y1);
                    }
                }
            }
            Arrays.sort(xs, 0, m);
            int c = 0;
            for (int i = 0; i < nx; i++) {
                double x = minX + (i + 0.5) * cell;
                while (c < m && xs[c] < x) {
                    c++;
                }
                centerIn[j * nx + i] = (c & 1) == 1;
            }
        }
    }

    /** Точка внутри контуров: состояние центра клетки и чётность пересечений отрезка центр → точка. */
    boolean covers(double x, double y) {
        if (size == 0 || x < minX || y < minY || x >= minX + nx * cell || y >= minY + ny * cell) {
            return false;
        }
        int i = cx(x);
        int j = cy(y);
        int k = j * nx + i;
        double cx0 = minX + (i + 0.5) * cell;
        double cy0 = minY + (j + 0.5) * cell;
        boolean in = centerIn[k];
        for (int p = cellStart[k]; p < cellStart[k + 1]; p++) {
            int o = cellItems[p] * 4;
            double ax = xy[o];
            double ay = xy[o + 1];
            double bx = xy[o + 2];
            double by = xy[o + 3];
            boolean sa = Geo.cross(cx0, cy0, x, y, ax, ay) > 0;
            boolean sb = Geo.cross(cx0, cy0, x, y, bx, by) > 0;
            if (sa != sb && (Geo.cross(ax, ay, bx, by, cx0, cy0) > 0) != (Geo.cross(ax, ay, bx, by, x, y) > 0)) {
                in = !in;
            }
        }
        return in;
    }

    private int nextEpoch() {
        int e = ++epoch;
        if (e == Integer.MAX_VALUE) {
            Arrays.fill(stamp, 0);
            epoch = 1;
            e = 1;
        }
        return e;
    }

    /** Все отрезки в клетках, пересекающих прямоугольник. */
    void box(double x0, double y0, double x1, double y1, Visitor visitor) {
        if (size == 0) {
            return;
        }
        int e = ++epoch;
        if (e == Integer.MAX_VALUE) {
            Arrays.fill(stamp, 0);
            epoch = 1;
            e = 1;
        }
        int ai = cx(x0);
        int bi = cx(x1);
        int aj = cy(y0);
        int bj = cy(y1);
        for (int j = aj; j <= bj; j++) {
            for (int i = ai; i <= bi; i++) {
                int k = j * nx + i;
                for (int p = cellStart[k]; p < cellStart[k + 1]; p++) {
                    int edge = cellItems[p];
                    if (stamp[edge] == e) {
                        continue;
                    }
                    stamp[edge] = e;
                    if (visitor.visit(edge)) {
                        return;
                    }
                }
            }
        }
    }
}
