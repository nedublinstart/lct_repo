package ru.lct.heatnet.engine.steiner;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.PathSmoother;
import ru.lct.heatnet.engine.greedy.PipeEmitter;
import ru.lct.heatnet.geo.GeoJsonGeometries;

/**
 * Алгоритм Курсора (Cursor Union Steiner).
 * <p>
 * Кластеры Mehlhorn сначала рисуют свои деревья на одном уличном рельсе.
 * Этот шаг склеивает их в <b>одно</b> дерево: оверлей с snap, ортогональная
 * сшивка, Kruskal-MST, обрезка листьев, одна врезка на компоненту.
 * Параллельные нитки и повторные проходы по той же улице исчезают,
 * остаётся иерархия ствол → ветка → ввод (двоичное/Стейнер-дерево по метрике улиц).
 */
public final class SteinerForestUnifier {

    static final double SNAP_M = 9.0;

    private SteinerForestUnifier() {
    }

    public static void unify(Variant variant, ObstacleIndex obstacles, AtomicInteger ids,
                             Set<String> oksIds, Map<String, Double> oksFlow) {
        if (variant == null || variant.segments.size() < 2 || oksIds == null || oksIds.isEmpty()) {
            return;
        }
        Overlay overlay = new Overlay();
        for (NewSegment seg : variant.segments) {
            if (seg == null || seg.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = seg.geometryMeters.getCoordinates();
            if (pts == null || pts.length < 2) {
                continue;
            }
            overlay.addPath(pts);
            if (seg.fromId != null && oksIds.contains(seg.fromId)) {
                overlay.markOks(pts[0], seg.fromId);
            }
        }
        for (TapPoint tap : variant.taps) {
            if (tap != null && tap.geometryMeters != null) {
                overlay.markTap(tap.geometryMeters.getCoordinate(), tap);
            }
        }
        overlay.stitch(obstacles);
        List<Branch> branches = overlay.extract(oksFlow);
        if (branches == null || branches.isEmpty()) {
            return;
        }
        Set<String> keepTaps = new HashSet<>();
        for (Branch b : branches) {
            if (b.tapId != null) {
                keepTaps.add(b.tapId);
            }
        }
        List<TapPoint> taps = new ArrayList<>();
        for (TapPoint t : variant.taps) {
            if (t != null && keepTaps.contains(t.id)) {
                taps.add(t);
            }
        }
        if (taps.isEmpty()) {
            return;
        }
        variant.segments.clear();
        variant.technicalNodes.clear();
        variant.chambers.clear();
        variant.taps.clear();
        variant.taps.addAll(taps);
        Map<String, String> nodeAt = new HashMap<>();
        for (TapPoint t : taps) {
            String node = t.nodeId != null ? t.nodeId : t.id;
            t.nodeId = node;
            if (t.geometryMeters != null) {
                nodeAt.put(key(t.geometryMeters.getCoordinate()), node);
            }
        }
        for (Branch b : branches) {
            List<Coordinate> path = b.oksId != null
                    ? PathSmoother.collapseKeepStub(b.path, obstacles)
                    : PathSmoother.collapseColinear(b.path, obstacles);
            if (path == null || path.size() < 2) {
                continue;
            }
            String from = b.oksId != null ? b.oksId : chamber(variant, ids, path.get(0), nodeAt, false);
            String to = b.tapId != null ? nodeOf(taps, b.tapId) : chamber(variant, ids, path.get(path.size() - 1), nodeAt, false);
            if (from == null || to == null || from.equals(to)) {
                continue;
            }
            PipeEmitter.emit(variant, obstacles, ids, from, to, Math.max(0.01, b.flow), path);
        }
    }

    private static String nodeOf(List<TapPoint> taps, String tapId) {
        for (TapPoint t : taps) {
            if (tapId.equals(t.id)) {
                return t.nodeId != null ? t.nodeId : t.id;
            }
        }
        return tapId;
    }

    private static String chamber(Variant variant, AtomicInteger ids, Coordinate c,
                                  Map<String, String> nodeAt, boolean atTap) {
        String k = key(c);
        String existing = nodeAt.get(k);
        if (existing != null) {
            return existing;
        }
        NewChamber ch = new NewChamber();
        ch.id = "CH-" + ids.getAndIncrement();
        ch.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(c));
        ch.atTap = atTap;
        variant.chambers.add(ch);
        nodeAt.put(k, ch.id);
        return ch.id;
    }

    static String key(Coordinate c) {
        return Math.round(c.x / SNAP_M) + ":" + Math.round(c.y / SNAP_M);
    }

    private static final class Branch {
        List<Coordinate> path;
        String oksId;
        String tapId;
        double flow;
    }

    private static final class Overlay {
        private final List<Coordinate> coords = new ArrayList<>();
        private final List<String> oksAt = new ArrayList<>();
        private final List<TapPoint> tapAt = new ArrayList<>();
        private final Map<String, Integer> index = new HashMap<>();
        private final List<int[]> edges = new ArrayList<>();
        private final List<Double> weights = new ArrayList<>();

        void addPath(Coordinate[] path) {
            int prev = -1;
            for (Coordinate raw : path) {
                if (raw == null) {
                    continue;
                }
                int id = addNode(raw);
                if (prev >= 0 && prev != id) {
                    double w = coords.get(prev).distance(coords.get(id));
                    if (w > 0.2) {
                        edges.add(new int[]{prev, id});
                        weights.add(w);
                    }
                }
                prev = id;
            }
        }

        void markOks(Coordinate c, String id) {
            int n = addNode(c);
            if (oksAt.get(n) == null) {
                oksAt.set(n, id);
            }
        }

        void markTap(Coordinate c, TapPoint tap) {
            int n = addNode(c);
            if (tapAt.get(n) == null) {
                tapAt.set(n, tap);
            }
        }

        void stitch(ObstacleIndex obstacles) {
            int n = coords.size();
            for (int i = 0; i < n; i++) {
                Coordinate a = coords.get(i);
                for (int j = i + 1; j < n; j++) {
                    Coordinate b = coords.get(j);
                    double d = a.distance(b);
                    if (d < 0.8 || d > 16) {
                        continue;
                    }
                    if (obstacles != null && obstacles.segmentHitsAvoid(a, b, 0, true)) {
                        continue;
                    }
                    if (obstacles != null && obstacles.special() != null) {
                        ru.lct.heatnet.engine.greedy.SpecialLayer.Corridor c =
                                obstacles.special().nearestCorridor(
                                        new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5), 24);
                        if (c != null && c.axis != null) {
                            double ang = ru.lct.heatnet.engine.greedy.SpecialLayer.crossingAngleDeg(a, b, c.axis);
                            if (ang > 16 && ang < 74) {
                                continue;
                            }
                        }
                    }
                    edges.add(new int[]{i, j});
                    weights.add(d);
                }
            }
        }

        List<Branch> extract(Map<String, Double> oksFlow) {
            int n = coords.size();
            if (n < 2 || edges.isEmpty()) {
                return null;
            }
            List<int[]> mst = kruskal(n, edges, weights);
            List<List<int[]>> adj = new ArrayList<>();
            for (int i = 0; i < n; i++) {
                adj.add(new ArrayList<>());
            }
            for (int[] e : mst) {
                adj.get(e[0]).add(e);
                adj.get(e[1]).add(e);
            }
            boolean[] terminal = new boolean[n];
            List<Integer> oksNodes = new ArrayList<>();
            List<Integer> tapNodes = new ArrayList<>();
            for (int i = 0; i < n; i++) {
                if (oksAt.get(i) != null) {
                    terminal[i] = true;
                    oksNodes.add(i);
                }
                if (tapAt.get(i) != null) {
                    terminal[i] = true;
                    tapNodes.add(i);
                }
            }
            if (oksNodes.isEmpty() || tapNodes.isEmpty()) {
                return null;
            }
            prune(adj, terminal);
            int[] parent = bfsFromTaps(adj, tapNodes);
            for (int o : oksNodes) {
                if (parent[o] < 0) {
                    return null;
                }
            }
            int root = pickRoot(tapNodes, oksNodes);
            if (root < 0) {
                return null;
            }
            for (int t : tapNodes) {
                if (t != root && oksAt.get(t) == null && sameRoot(parent, t, root)) {
                    terminal[t] = false;
                }
            }
            prune(adj, terminal);
            parent = bfsParent(adj, root);
            boolean[] keep = new boolean[n];
            keep[root] = true;
            for (int o : oksNodes) {
                keep[o] = true;
            }
            for (int i = 0; i < n; i++) {
                if (adj.get(i).size() >= 3) {
                    keep[i] = true;
                }
            }
            Map<Integer, Double> flow = new HashMap<>();
            for (int o : oksNodes) {
                if (parent[o] < 0 && o != root) {
                    continue;
                }
                double f = oksFlow == null ? 0.01 : oksFlow.getOrDefault(oksAt.get(o), 0.01);
                int cur = o;
                int guard = 0;
                while (cur != root && guard++ < n + 2) {
                    flow.merge(cur, f, Double::sum);
                    cur = parent[cur];
                    if (cur < 0) {
                        break;
                    }
                }
            }
            List<Branch> out = new ArrayList<>();
            for (int i = 0; i < n; i++) {
                if (!keep[i] || i == root) {
                    continue;
                }
                List<Coordinate> path = new ArrayList<>();
                path.add(new Coordinate(coords.get(i)));
                int cur = i;
                int nextKeep = -1;
                int guard = 0;
                while (cur != root && guard++ < n + 2) {
                    int par = parent[cur];
                    if (par < 0) {
                        nextKeep = -1;
                        break;
                    }
                    path.add(new Coordinate(coords.get(par)));
                    cur = par;
                    if (keep[cur]) {
                        nextKeep = cur;
                        break;
                    }
                }
                if (nextKeep < 0 || path.size() < 2) {
                    continue;
                }
                Branch b = new Branch();
                b.path = path;
                b.oksId = oksAt.get(i);
                b.tapId = nextKeep == root ? tapAt.get(root).id : null;
                b.flow = Math.max(0.01, flow.getOrDefault(i, 0.01));
                out.add(b);
            }
            return out;
        }

        private int pickRoot(List<Integer> tapNodes, List<Integer> oksNodes) {
            int best = -1;
            double bestScore = Double.POSITIVE_INFINITY;
            for (int t : tapNodes) {
                double s = 0;
                for (int o : oksNodes) {
                    s += coords.get(t).distance(coords.get(o));
                }
                if (s < bestScore) {
                    bestScore = s;
                    best = t;
                }
            }
            return best;
        }

        private int addNode(Coordinate c) {
            int near = nearest(c, SNAP_M);
            if (near >= 0) {
                return near;
            }
            String k = Math.round(c.x / 3.0) + ":" + Math.round(c.y / 3.0);
            Integer existing = index.get(k);
            if (existing != null) {
                return existing;
            }
            int id = coords.size();
            coords.add(new Coordinate(c));
            oksAt.add(null);
            tapAt.add(null);
            index.put(k, id);
            return id;
        }

        private int nearest(Coordinate c, double snapM) {
            int best = -1;
            double bestD = snapM;
            for (int i = 0; i < coords.size(); i++) {
                double d = coords.get(i).distance(c);
                if (d <= bestD) {
                    bestD = d;
                    best = i;
                }
            }
            return best;
        }
    }

    private static List<int[]> kruskal(int n, List<int[]> edges, List<Double> weights) {
        Integer[] order = new Integer[edges.size()];
        for (int i = 0; i < order.length; i++) {
            order[i] = i;
        }
        Arrays.sort(order, Comparator.comparingDouble(weights::get));
        int[] p = new int[n];
        int[] r = new int[n];
        for (int i = 0; i < n; i++) {
            p[i] = i;
        }
        List<int[]> mst = new ArrayList<>();
        for (int i : order) {
            int[] e = edges.get(i);
            int a = find(p, e[0]);
            int b = find(p, e[1]);
            if (a == b) {
                continue;
            }
            if (r[a] < r[b]) {
                p[a] = b;
            } else if (r[a] > r[b]) {
                p[b] = a;
            } else {
                p[b] = a;
                r[a]++;
            }
            mst.add(e);
        }
        return mst;
    }

    private static int find(int[] p, int x) {
        while (p[x] != x) {
            p[x] = p[p[x]];
            x = p[x];
        }
        return x;
    }

    private static void prune(List<List<int[]>> adj, boolean[] terminal) {
        boolean changed = true;
        while (changed) {
            changed = false;
            for (int i = 0; i < adj.size(); i++) {
                if (terminal[i] || adj.get(i).size() != 1) {
                    continue;
                }
                int[] e = adj.get(i).get(0);
                int other = e[0] == i ? e[1] : e[0];
                adj.get(i).clear();
                adj.get(other).remove(e);
                changed = true;
            }
        }
    }

    private static int[] bfsFromTaps(List<List<int[]>> adj, List<Integer> tapNodes) {
        int[] parent = new int[adj.size()];
        Arrays.fill(parent, -1);
        ArrayDeque<Integer> q = new ArrayDeque<>();
        for (int t : tapNodes) {
            parent[t] = t;
            q.add(t);
        }
        while (!q.isEmpty()) {
            int u = q.removeFirst();
            for (int[] e : adj.get(u)) {
                int v = e[0] == u ? e[1] : e[0];
                if (parent[v] < 0) {
                    parent[v] = u;
                    q.add(v);
                }
            }
        }
        return parent;
    }

    private static boolean sameRoot(int[] parent, int a, int root) {
        int guard = 0;
        int cur = a;
        while (cur >= 0 && parent[cur] != cur && guard++ < parent.length + 2) {
            cur = parent[cur];
        }
        return cur == root;
    }

    private static int[] bfsParent(List<List<int[]>> adj, int root) {
        int[] parent = new int[adj.size()];
        Arrays.fill(parent, -1);
        ArrayDeque<Integer> q = new ArrayDeque<>();
        q.add(root);
        parent[root] = root;
        while (!q.isEmpty()) {
            int u = q.removeFirst();
            for (int[] e : adj.get(u)) {
                int v = e[0] == u ? e[1] : e[0];
                if (parent[v] < 0) {
                    parent[v] = u;
                    q.add(v);
                }
            }
        }
        return parent;
    }
}
