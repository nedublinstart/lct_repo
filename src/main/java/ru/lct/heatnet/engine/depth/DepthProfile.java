package ru.lct.heatnet.engine.depth;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Профиль глубины по разделу 5 приложения.
 * <p>
 * Глубина — до верхней границы габарита. Обычная 3,0 м, минимум 0,7 м, уклон не круче 0,10.
 * На пересечении глубина фиксируется, к обычной возвращается спуском или подъёмом.
 * Между близкими пересечениями глубина остаётся изменённой, если уклон не успевает вернуться.
 */
public final class DepthProfile {

    static final double INF = 1.0e12;

    private DepthProfile() {
    }

    /** Kгл: 1 при h ≤ обычной, иначе 1 + 0,10·(h − 3). */
    public static double kGl(double depth, double ordinary, double factor) {
        if (depth <= ordinary + 1e-9) {
            return 1.0;
        }
        return 1.0 + factor * (depth - ordinary);
    }

    /** Среднее Kгл на равномерном уклоне. */
    public static double kGlMean(double start, double end, double ordinary, double factor) {
        return 0.5 * (kGl(start, ordinary, factor) + kGl(end, ordinary, factor));
    }

    /** Вертикальное окно одной коммуникации: выше (h ≤ aboveMax) или ниже (h ≥ belowMin). */
    public static final class Band {
        public final double aboveMax;
        public final double belowMin;

        public Band(double aboveMax, double belowMin) {
            this.aboveMax = aboveMax;
            this.belowMin = belowMin;
        }
    }

    /** Выбранная глубина и допустимый отрезок, внутри которого её можно сдвигать. */
    public static final class Choice {
        public final double depth;
        public final double low;
        public final double high;

        Choice(double depth, double low, double high) {
            this.depth = depth;
            this.low = low;
            this.high = high;
        }
    }

    /**
     * Самый дешёвый допустимый верх габарита.
     * При равном Kгл берётся глубина ближе к обычной: проход сверху дешевле прохода снизу.
     */
    public static Choice choose(double ordinary, double floor, List<Band> bands, double factor) {
        double base = Math.max(0.0, floor);
        if (bands == null || bands.isEmpty()) {
            double h = Math.max(ordinary, base);
            return new Choice(h, base, INF);
        }
        int n = Math.min(bands.size(), 16);
        double bestH = Double.NaN;
        double bestLow = base;
        double bestHigh = INF;
        double bestK = Double.POSITIVE_INFINITY;
        double bestDist = Double.POSITIVE_INFINITY;
        int masks = 1 << n;
        for (int mask = 0; mask < masks; mask++) {
            double low = base;
            double high = INF;
            for (int i = 0; i < n; i++) {
                Band band = bands.get(i);
                if ((mask & (1 << i)) == 0) {
                    high = Math.min(high, band.aboveMax);
                } else {
                    low = Math.max(low, band.belowMin);
                }
            }
            if (low > high + 1e-9) {
                continue;
            }
            double h = clamp(ordinary, low, high);
            double k = kGl(h, ordinary, factor);
            double dist = Math.abs(h - ordinary);
            if (k < bestK - 1e-12 || (Math.abs(k - bestK) <= 1e-12 && dist < bestDist - 1e-12)) {
                bestK = k;
                bestDist = dist;
                bestH = h;
                bestLow = low;
                bestHigh = high;
            }
        }
        if (Double.isNaN(bestH)) {
            double h = base;
            for (int i = 0; i < n; i++) {
                h = Math.max(h, bands.get(i).belowMin);
            }
            return new Choice(h, h, INF);
        }
        return new Choice(bestH, bestLow, bestHigh);
    }

    public static final class Edge {
        public final String fromId;
        public final String toId;
        public final double length;

        public Edge(String fromId, String toId, double length) {
            this.fromId = fromId;
            this.toId = toId;
            this.length = length;
        }
    }

    /** Участок, где верх габарита обязан быть выбранной глубиной. */
    public static final class Pin {
        public final int edge;
        public double s0;
        public double s1;
        public final List<Band> bands = new ArrayList<>();
        public double floor;
        public double depth;
        public double low;
        public double high;

        public Pin(int edge, double s0, double s1) {
            this.edge = edge;
            this.s0 = s0;
            this.s1 = s1;
        }
    }

    /** Минимальная глубина на интервале (дорога, трамвай). Не задаёт точную отметку. */
    public static final class Cover {
        public final int edge;
        public final double s0;
        public final double s1;
        public final double minDepth;

        public Cover(int edge, double s0, double s1, double minDepth) {
            this.edge = edge;
            this.s0 = Math.min(s0, s1);
            this.s1 = Math.max(s0, s1);
            this.minDepth = minDepth;
        }
    }

    public static final class Knot {
        public final double station;
        public final double depth;

        public Knot(double station, double depth) {
            this.station = station;
            this.depth = depth;
        }
    }

    /**
     * Кусочно-линейный профиль по каждому ребру.
     * Станция 0 — конец {@code fromId}, станция длины — конец {@code toId}.
     */
    public static List<List<Knot>> solve(List<Edge> edges, List<Pin> pins, List<Cover> covers,
                                          double ordinary, double minDepth, double slope, double factor) {
        int m = edges.size();
        List<List<Knot>> out = new ArrayList<>();
        if (m == 0) {
            return out;
        }
        double rise = slope > 1e-9 ? slope : 0.10;
        List<Pin> live = pins == null ? new ArrayList<>() : pins;
        List<Cover> roofs = covers == null ? new ArrayList<>() : covers;
        for (Pin pin : live) {
            Edge edge = edges.get(pin.edge);
            double length = Math.max(0.0, edge.length);
            pin.s0 = clamp(pin.s0, 0.0, length);
            pin.s1 = clamp(pin.s1, 0.0, length);
            if (pin.s1 < pin.s0) {
                double swap = pin.s0;
                pin.s0 = pin.s1;
                pin.s1 = swap;
            }
            if (pin.floor < minDepth) {
                pin.floor = minDepth;
            }
        }
        int[] from = new int[m];
        int[] to = new int[m];
        double[][] nodeDist = distances(edges, from, to);
        resolve(edges, live, roofs, from, to, nodeDist, ordinary, minDepth, rise, factor);
        for (int e = 0; e < m; e++) {
            out.add(knots(e, edges, live, roofs, from, to, nodeDist, ordinary, minDepth, rise));
        }
        return out;
    }

    private static void resolve(List<Edge> edges, List<Pin> pins, List<Cover> covers,
                                int[] from, int[] to, double[][] nodeDist,
                                double ordinary, double minDepth, double slope, double factor) {
        int guard = Math.max(4, pins.size() * 3 + 4);
        for (int step = 0; step < guard; step++) {
            boolean changed = raiseForCovers(edges, pins, covers, from, to, nodeDist, ordinary, minDepth, slope, factor);
            changed = mergeConflicts(edges, pins, from, to, nodeDist, ordinary, slope, factor) || changed;
            if (!changed) {
                return;
            }
        }
    }

    private static boolean raiseForCovers(List<Edge> edges, List<Pin> pins, List<Cover> covers,
                                          int[] from, int[] to, double[][] nodeDist,
                                          double ordinary, double minDepth, double slope, double factor) {
        boolean changed = false;
        for (Pin pin : pins) {
            for (Cover cover : covers) {
                if (cover.minDepth <= minDepth + 1e-12) {
                    continue;
                }
                double dist = spanDistance(pin.edge, pin.s0, pin.s1, cover.edge, cover.s0, cover.s1,
                        edges, from, to, nodeDist);
                double need = cover.minDepth - slope * dist;
                if (need < minDepth) {
                    need = minDepth;
                }
                if (pin.depth + 1e-7 >= need) {
                    continue;
                }
                if (need <= pin.high + 1e-7) {
                    double low = Math.max(pin.low, need);
                    double h = clamp(ordinary, low, pin.high);
                    if (h > pin.depth + 1e-9) {
                        pin.depth = h;
                        pin.low = low;
                        changed = true;
                    }
                } else {
                    pin.floor = Math.max(pin.floor, need);
                    Choice choice = choose(ordinary, pin.floor, pin.bands, factor);
                    if (Math.abs(choice.depth - pin.depth) > 1e-9) {
                        pin.depth = choice.depth;
                        pin.low = choice.low;
                        pin.high = choice.high;
                        changed = true;
                    }
                }
            }
        }
        return changed;
    }

    private static boolean mergeConflicts(List<Edge> edges, List<Pin> pins,
                                          int[] from, int[] to, double[][] nodeDist,
                                          double ordinary, double slope, double factor) {
        int n = pins.size();
        if (n < 2) {
            return false;
        }
        int[] parent = new int[n];
        for (int i = 0; i < n; i++) {
            parent[i] = i;
        }
        boolean any = false;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                Pin a = pins.get(i);
                Pin b = pins.get(j);
                double dist = spanDistance(a.edge, a.s0, a.s1, b.edge, b.s0, b.s1, edges, from, to, nodeDist);
                if (Math.abs(a.depth - b.depth) > slope * dist + 1e-6) {
                    union(parent, i, j);
                    any = true;
                }
            }
        }
        if (!any) {
            return false;
        }
        boolean changed = false;
        boolean[] seen = new boolean[n];
        for (int i = 0; i < n; i++) {
            int root = find(parent, i);
            if (seen[root]) {
                continue;
            }
            seen[root] = true;
            List<Band> bands = new ArrayList<>();
            double floor = 0.0;
            int count = 0;
            for (int j = 0; j < n; j++) {
                if (find(parent, j) != root) {
                    continue;
                }
                count++;
                floor = Math.max(floor, pins.get(j).floor);
                for (Band band : pins.get(j).bands) {
                    if (!containsBand(bands, band)) {
                        bands.add(band);
                    }
                }
            }
            if (count < 2) {
                continue;
            }
            Choice choice = choose(ordinary, floor, bands, factor);
            for (int j = 0; j < n; j++) {
                if (find(parent, j) != root) {
                    continue;
                }
                Pin pin = pins.get(j);
                if (Math.abs(pin.depth - choice.depth) > 1e-9) {
                    changed = true;
                }
                pin.depth = choice.depth;
                pin.low = choice.low;
                pin.high = choice.high;
                pin.floor = floor;
            }
        }
        return changed;
    }

    private static List<Knot> knots(int edgeIndex, List<Edge> edges, List<Pin> pins, List<Cover> covers,
                                    int[] from, int[] to, double[][] nodeDist,
                                    double ordinary, double minDepth, double slope) {
        Edge edge = edges.get(edgeIndex);
        double length = Math.max(0.0, edge.length);
        if (length < 1e-6) {
            double h = ordinary;
            for (Pin pin : pins) {
                if (pin.edge == edgeIndex) {
                    h = pin.depth;
                }
            }
            List<Knot> one = new ArrayList<>();
            one.add(new Knot(0.0, h));
            return one;
        }
        List<Linear> lower = new ArrayList<>();
        List<Linear> upper = new ArrayList<>();
        lower.add(new Linear(0.0, length, minDepth, 0.0));
        for (Cover cover : covers) {
            if (cover.edge != edgeIndex || cover.s1 - cover.s0 < 1e-9) {
                continue;
            }
            double s0 = clamp(cover.s0, 0.0, length);
            double s1 = clamp(cover.s1, 0.0, length);
            if (s1 > s0) {
                lower.add(new Linear(s0, s1, cover.minDepth, 0.0));
            }
        }
        for (Pin pin : pins) {
            addCones(lower, upper, edgeIndex, edges, pin, from, to, nodeDist, slope);
        }
        List<Double> cuts = new ArrayList<>();
        cuts.add(0.0);
        cuts.add(length);
        for (Linear piece : lower) {
            cuts.add(piece.s0);
            cuts.add(piece.s1);
            addRoot(cuts, piece, ordinary);
            addRoot(cuts, piece, minDepth);
        }
        for (Linear piece : upper) {
            cuts.add(piece.s0);
            cuts.add(piece.s1);
            addRoot(cuts, piece, ordinary);
        }
        addCrossings(cuts, lower);
        addCrossings(cuts, upper);
        List<Double> stations = unique(cuts, length);
        List<Knot> raw = new ArrayList<>();
        for (double station : stations) {
            raw.add(new Knot(station, depthAt(station, lower, upper, ordinary, minDepth)));
        }
        return simplify(raw, ordinary);
    }

    private static void addCones(List<Linear> lower, List<Linear> upper, int host, List<Edge> edges, Pin pin,
                                 int[] from, int[] to, double[][] nodeDist, double slope) {
        double length = Math.max(0.0, edges.get(host).length);
        if (pin.edge == host) {
            addSide(lower, upper, 0.0, pin.s0, pin.depth, slope, pin.s0, -1.0);
            addSide(lower, upper, pin.s0, pin.s1, pin.depth, slope, 0.0, 0.0);
            addSide(lower, upper, pin.s1, length, pin.depth, slope, -pin.s1, 1.0);
            return;
        }
        double pinLength = Math.max(0.0, edges.get(pin.edge).length);
        double d0 = distNodeToSpan(from[host], pin, pinLength, from, to, nodeDist);
        double d1 = distNodeToSpan(to[host], pin, pinLength, from, to, nodeDist);
        if (d0 >= INF / 2.0 && d1 >= INF / 2.0) {
            return;
        }
        double apex = (d1 + length - d0) / 2.0;
        double mid = clamp(apex, 0.0, length);
        if (mid > 1e-9) {
            addSide(lower, upper, 0.0, mid, pin.depth, slope, d0, 1.0);
        }
        if (length - mid > 1e-9) {
            addSide(lower, upper, mid, length, pin.depth, slope, d1 + length, -1.0);
        }
    }

    /** Кратчайшее расстояние от узла сети до интервала пина, м. */
    private static double distNodeToSpan(int node, Pin pin, double pinLength, int[] from, int[] to, double[][] nodeDist) {
        double viaFrom = nodeDist[node][from[pin.edge]] + pin.s0;
        double viaTo = nodeDist[node][to[pin.edge]] + (pinLength - pin.s1);
        return Math.min(viaFrom, viaTo);
    }

    private static void addSide(List<Linear> lower, List<Linear> upper, double s0, double s1,
                                double depth, double slope, double distA, double distB) {
        if (s1 - s0 < 1e-9) {
            return;
        }
        // dist = distA + distB * s; lb = h - slope * dist; ub = h + slope * dist.
        lower.add(new Linear(s0, s1, depth - slope * distA, -slope * distB));
        upper.add(new Linear(s0, s1, depth + slope * distA, slope * distB));
    }

    private static double depthAt(double station, List<Linear> lower, List<Linear> upper,
                                  double ordinary, double minDepth) {
        double lb = minDepth;
        double ub = INF;
        for (Linear piece : lower) {
            if (piece.covers(station)) {
                lb = Math.max(lb, piece.at(station));
            }
        }
        for (Linear piece : upper) {
            if (piece.covers(station)) {
                ub = Math.min(ub, piece.at(station));
            }
        }
        if (lb > ub + 1e-6) {
            return lb;
        }
        return clamp(ordinary, lb, ub);
    }

    private static List<Knot> simplify(List<Knot> raw, double ordinary) {
        if (raw.size() <= 2) {
            return raw;
        }
        List<Knot> out = new ArrayList<>();
        out.add(raw.get(0));
        for (int i = 1; i < raw.size() - 1; i++) {
            Knot prev = out.get(out.size() - 1);
            Knot cur = raw.get(i);
            Knot next = raw.get(i + 1);
            boolean bend = Math.abs(grade(prev, cur) - grade(cur, next)) > 1e-5;
            boolean crosses = (prev.depth - ordinary) * (next.depth - ordinary) < -1e-8
                    && Math.abs(cur.depth - ordinary) <= 1e-3;
            if (bend || crosses) {
                out.add(cur);
            }
        }
        out.add(raw.get(raw.size() - 1));
        return out;
    }

    private static double grade(Knot a, Knot b) {
        double ds = b.station - a.station;
        if (Math.abs(ds) < 1e-12) {
            return 0.0;
        }
        return (b.depth - a.depth) / ds;
    }

    private static void addRoot(List<Double> cuts, Linear piece, double level) {
        if (Math.abs(piece.b) < 1e-12) {
            return;
        }
        double s = (level - piece.a) / piece.b;
        if (s >= piece.s0 - 1e-8 && s <= piece.s1 + 1e-8) {
            cuts.add(s);
        }
    }

    private static void addCrossings(List<Double> cuts, List<Linear> pieces) {
        for (int i = 0; i < pieces.size(); i++) {
            Linear left = pieces.get(i);
            for (int j = i + 1; j < pieces.size(); j++) {
                Linear right = pieces.get(j);
                if (Math.abs(left.b - right.b) < 1e-12) {
                    continue;
                }
                double s = (right.a - left.a) / (left.b - right.b);
                if (left.covers(s) && right.covers(s)) {
                    cuts.add(s);
                }
            }
        }
    }

    private static List<Double> unique(List<Double> cuts, double length) {
        Collections.sort(cuts);
        List<Double> out = new ArrayList<>();
        for (double station : cuts) {
            if (station < -1e-4 || station > length + 1e-4) {
                continue;
            }
            double s = clamp(station, 0.0, length);
            if (out.isEmpty() || s - out.get(out.size() - 1) > 1e-5) {
                out.add(s);
            }
        }
        if (out.isEmpty() || out.get(0) > 1e-8) {
            out.add(0, 0.0);
        }
        if (out.get(out.size() - 1) < length - 1e-8) {
            out.add(length);
        }
        return out;
    }

    private static double[][] distances(List<Edge> edges, int[] from, int[] to) {
        List<String> names = new ArrayList<>();
        for (int i = 0; i < edges.size(); i++) {
            Edge edge = edges.get(i);
            from[i] = index(names, edge.fromId == null ? "f" + i : edge.fromId);
            to[i] = index(names, edge.toId == null ? "t" + i : edge.toId);
        }
        int n = names.size();
        double[][] dist = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                dist[i][j] = i == j ? 0.0 : INF;
            }
        }
        for (int i = 0; i < edges.size(); i++) {
            double length = Math.max(0.0, edges.get(i).length);
            if (length < dist[from[i]][to[i]]) {
                dist[from[i]][to[i]] = length;
                dist[to[i]][from[i]] = length;
            }
        }
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) {
                if (dist[i][k] >= INF / 2.0) {
                    continue;
                }
                for (int j = 0; j < n; j++) {
                    double through = dist[i][k] + dist[k][j];
                    if (through < dist[i][j]) {
                        dist[i][j] = through;
                    }
                }
            }
        }
        return dist;
    }

    private static int index(List<String> names, String id) {
        for (int i = 0; i < names.size(); i++) {
            if (names.get(i).equals(id)) {
                return i;
            }
        }
        names.add(id);
        return names.size() - 1;
    }

    static double spanDistance(int edgeA, double a0, double a1, int edgeB, double b0, double b1,
                               List<Edge> edges, int[] from, int[] to, double[][] nodeDist) {
        if (edgeA == edgeB) {
            if (a1 < b0) {
                return b0 - a1;
            }
            if (b1 < a0) {
                return a0 - b1;
            }
            return 0.0;
        }
        double tailA = edges.get(edgeA).length - a1;
        double tailB = edges.get(edgeB).length - b1;
        return Math.min(
                Math.min(nodeDist[from[edgeA]][from[edgeB]] + a0 + b0,
                        nodeDist[from[edgeA]][to[edgeB]] + a0 + tailB),
                Math.min(nodeDist[to[edgeA]][from[edgeB]] + tailA + b0,
                        nodeDist[to[edgeA]][to[edgeB]] + tailA + tailB));
    }

    private static boolean containsBand(List<Band> bands, Band band) {
        for (Band have : bands) {
            if (Math.abs(have.aboveMax - band.aboveMax) < 1e-9 && Math.abs(have.belowMin - band.belowMin) < 1e-9) {
                return true;
            }
        }
        return false;
    }

    private static int find(int[] parent, int i) {
        int root = i;
        while (parent[root] != root) {
            root = parent[root];
        }
        while (parent[i] != root) {
            int next = parent[i];
            parent[i] = root;
            i = next;
        }
        return root;
    }

    private static void union(int[] parent, int a, int b) {
        int ra = find(parent, a);
        int rb = find(parent, b);
        if (ra != rb) {
            parent[rb] = ra;
        }
    }

    private static double clamp(double value, double low, double high) {
        if (value < low) {
            return low;
        }
        if (value > high) {
            return high;
        }
        return value;
    }

    /** value = a + b · s на отрезке станций. */
    private static final class Linear {
        final double s0;
        final double s1;
        final double a;
        final double b;

        Linear(double s0, double s1, double a, double b) {
            this.s0 = s0;
            this.s1 = s1;
            this.a = a;
            this.b = b;
        }

        boolean covers(double station) {
            return station >= s0 - 1e-7 && station <= s1 + 1e-7;
        }

        double at(double station) {
            return a + b * station;
        }
    }
}
