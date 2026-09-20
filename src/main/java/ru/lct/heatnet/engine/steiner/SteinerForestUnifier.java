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
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.PathSmoother;
import ru.lct.heatnet.engine.greedy.PipeEmitter;
import ru.lct.heatnet.geo.GeoJsonGeometries;

/**
 * Алгоритм Курсора (Cursor Union Steiner).
 * <p>
 * Кластеры Mehlhorn сначала рисуют деревья на скелете улиц.
 * Этот шаг склеивает их в <b>одно</b> дерево: оверлей с snap, ортогональная
 * сшивка до 22 м, Kruskal-MST, обрезка листьев, одна врезка на компоненту.
 * Параллельные нитки одной стороны улицы исчезают,
 * остаётся иерархия ствол → ветка → ввод.
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
            if (seg.toId != null && oksIds.contains(seg.toId)) {
                overlay.markOks(pts[pts.length - 1], seg.toId);
            }
        }
        for (TapPoint tap : variant.taps) {
            if (tap != null && tap.geometryMeters != null) {
                overlay.markTap(tap.geometryMeters.getCoordinate(), tap);
            }
        }
        overlay.stitch(obstacles);
        List<Branch> branches = overlay.extract(oksFlow, null);
        if (branches == null || branches.isEmpty()) {
            return;
        }
        List<NewSegment> savedSegs = new ArrayList<>(variant.segments);
        List<TapPoint> savedTaps = new ArrayList<>(variant.taps);
        List<NewChamber> savedChambers = new ArrayList<>(variant.chambers);
        List<TechnicalNode> savedNodes = new ArrayList<>(variant.technicalNodes);
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
            String from = b.oksId != null ? b.oksId : junction(variant, ids, path.get(0), nodeAt);
            String to = b.tapId != null ? nodeOf(taps, b.tapId) : junction(variant, ids, path.get(path.size() - 1), nodeAt);
            if (from == null || to == null || from.equals(to)) {
                continue;
            }
            PipeEmitter.emit(variant, obstacles, ids, from, to, Math.max(0.01, b.flow), path);
        }
        ensureTapChambers(variant);
        graftMissing(variant, savedSegs, savedTaps, savedChambers, savedNodes, oksIds, oksFlow);
    }

    private static String nodeOf(List<TapPoint> taps, String tapId) {
        for (TapPoint t : taps) {
            if (tapId.equals(t.id)) {
                return t.nodeId != null ? t.nodeId : t.id;
            }
        }
        return tapId;
    }

    private static String junction(Variant variant, AtomicInteger ids, Coordinate c,
                                  Map<String, String> nodeAt) {
        String k = key(c);
        String existing = nodeAt.get(k);
        if (existing != null) {
            return existing;
        }
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-" + ids.getAndIncrement();
        node.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(c));
        node.reason = "steiner_branch";
        variant.technicalNodes.add(node);
        nodeAt.put(k, node.id);
        return node.id;
    }

    static String key(Coordinate c) {
        return Math.round(c.x / SNAP_M) + ":" + Math.round(c.y / SNAP_M);
    }

    private static void ensureTapChambers(Variant variant) {
        Set<String> have = new HashSet<>();
        for (NewChamber ch : variant.chambers) {
            if (ch.id != null) {
                have.add(ch.id);
            }
        }
        for (TapPoint t : variant.taps) {
            if (t == null || t.nodeId == null || !t.nodeId.startsWith("CH-") || have.contains(t.nodeId)) {
                continue;
            }
            NewChamber ch = new NewChamber();
            ch.id = t.nodeId;
            ch.atTap = true;
            ch.geometryMeters = t.geometryMeters;
            variant.chambers.add(ch);
            have.add(ch.id);
        }
    }

    private static void graftMissing(Variant variant, List<NewSegment> savedSegs, List<TapPoint> savedTaps,
                                     List<NewChamber> savedChambers, List<TechnicalNode> savedNodes,
                                     Set<String> oksIds, Map<String, Double> oksFlow) {
        Set<String> have = new HashSet<>();
        Set<String> presentNodes = new HashSet<>();
        for (NewSegment s : variant.segments) {
            if (s.fromId != null) {
                have.add(s.fromId);
                presentNodes.add(s.fromId);
            }
            if (s.toId != null) {
                presentNodes.add(s.toId);
            }
        }
        for (TapPoint t : variant.taps) {
            if (t.id != null) {
                presentNodes.add(t.id);
            }
            if (t.nodeId != null) {
                presentNodes.add(t.nodeId);
            }
        }
        Map<String, List<NewSegment>> outgoing = new HashMap<>();
        for (NewSegment s : savedSegs) {
            if (s.fromId != null) {
                outgoing.computeIfAbsent(s.fromId, k -> new ArrayList<>()).add(s);
            }
        }
        Set<String> tapIds = new HashSet<>();
        for (TapPoint t : savedTaps) {
            if (t.id != null) {
                tapIds.add(t.id);
            }
            if (t.nodeId != null) {
                tapIds.add(t.nodeId);
            }
        }
        for (String oks : oksIds) {
            if (have.contains(oks)) {
                continue;
            }
            Coordinate origin = originOf(savedSegs, oks);
            Attach near = nearestAttach(variant, origin);
            if (origin != null && near != null && near.dist <= 48 && near.nodeId != null) {
                NewSegment stub = new NewSegment();
                stub.fromId = oks;
                stub.toId = near.nodeId;
                stub.flowTph = oksFlow == null ? 0.01 : Math.max(0.01, oksFlow.getOrDefault(oks, 0.01));
                stub.geometryMeters = GeoJsonGeometries.GF.createLineString(new Coordinate[]{
                        new Coordinate(origin), new Coordinate(near.at)
                });
                stub.lengthM = stub.geometryMeters.getLength();
                stub.layingMethod = "base";
                stub.kSpec = 1.0;
                variant.segments.add(stub);
                have.add(oks);
                continue;
            }
            List<NewSegment> chain = new ArrayList<>();
            Set<String> seen = new HashSet<>();
            ArrayDeque<String> q = new ArrayDeque<>();
            q.add(oks);
            seen.add(oks);
            while (!q.isEmpty()) {
                String node = q.removeFirst();
                if (tapIds.contains(node) || (presentNodes.contains(node) && !node.equals(oks))) {
                    continue;
                }
                for (NewSegment s : outgoing.getOrDefault(node, List.of())) {
                    chain.add(s);
                    String next = s.toId;
                    if (next != null && seen.add(next) && !tapIds.contains(next) && !presentNodes.contains(next)) {
                        q.add(next);
                    }
                }
            }
            if (chain.isEmpty()) {
                continue;
            }
            Set<String> used = new HashSet<>();
            for (NewSegment s : chain) {
                variant.segments.add(s);
                if (s.fromId != null) {
                    used.add(s.fromId);
                    have.add(s.fromId);
                }
                if (s.toId != null) {
                    used.add(s.toId);
                }
            }
            addTapIfMissing(variant, savedTaps, used);
            addChamberIfMissing(variant, savedChambers, used);
            addNodeIfMissing(variant, savedNodes, used);
        }
    }

    private static Coordinate originOf(List<NewSegment> segs, String oks) {
        for (NewSegment s : segs) {
            if (oks.equals(s.fromId) && s.geometryMeters != null && s.geometryMeters.getNumPoints() > 0) {
                return s.geometryMeters.getCoordinateN(0);
            }
        }
        return null;
    }

    private static final class Attach {
        String nodeId;
        Coordinate at;
        double dist;
    }

    private static Attach nearestAttach(Variant variant, Coordinate origin) {
        if (origin == null || variant.segments.isEmpty()) {
            return null;
        }
        Attach best = null;
        for (NewSegment s : variant.segments) {
            if (s.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            for (int i = 0; i < pts.length; i++) {
                double d = origin.distance(pts[i]);
                if (best == null || d < best.dist) {
                    Attach a = new Attach();
                    a.at = pts[i];
                    a.dist = d;
                    if (i == 0 && s.fromId != null) {
                        a.nodeId = s.fromId;
                    } else if (i == pts.length - 1 && s.toId != null) {
                        a.nodeId = s.toId;
                    } else {
                        a.nodeId = s.toId != null ? s.toId : s.fromId;
                    }
                    best = a;
                }
            }
        }
        for (TapPoint t : variant.taps) {
            if (t.geometryMeters == null) {
                continue;
            }
            Coordinate c = t.geometryMeters.getCoordinate();
            double d = origin.distance(c);
            if (best == null || d < best.dist) {
                Attach a = new Attach();
                a.at = c;
                a.dist = d;
                a.nodeId = t.nodeId != null ? t.nodeId : t.id;
                best = a;
            }
        }
        return best;
    }

    private static void addTapIfMissing(Variant variant, List<TapPoint> saved, Set<String> used) {
        for (TapPoint t : saved) {
            if (!used.contains(t.id) && (t.nodeId == null || !used.contains(t.nodeId))) {
                continue;
            }
            boolean exists = false;
            for (TapPoint cur : variant.taps) {
                if (t.id.equals(cur.id)) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                variant.taps.add(t);
            }
        }
    }

    private static void addChamberIfMissing(Variant variant, List<NewChamber> saved, Set<String> used) {
        for (NewChamber ch : saved) {
            if (ch.id == null || !used.contains(ch.id)) {
                continue;
            }
            boolean exists = false;
            for (NewChamber cur : variant.chambers) {
                if (ch.id.equals(cur.id)) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                variant.chambers.add(ch);
            }
        }
    }

    private static void addNodeIfMissing(Variant variant, List<TechnicalNode> saved, Set<String> used) {
        for (TechnicalNode n : saved) {
            if (n.id == null || !used.contains(n.id)) {
                continue;
            }
            boolean exists = false;
            for (TechnicalNode cur : variant.technicalNodes) {
                if (n.id.equals(cur.id)) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                variant.technicalNodes.add(n);
            }
        }
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
        private final Map<String, Integer> oksIndex = new HashMap<>();
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
            if (id == null) {
                return;
            }
            Integer existing = oksIndex.get(id);
            int n = addNode(c);
            if (existing != null) {
                if (existing != n) {
                    double w = coords.get(existing).distance(coords.get(n));
                    if (w > 0.2) {
                        edges.add(new int[]{existing, n});
                        weights.add(w);
                    }
                }
                return;
            }
            if (oksAt.get(n) != null && !id.equals(oksAt.get(n))) {
                int extra = coords.size();
                Coordinate base = coords.get(n);
                coords.add(new Coordinate(base.x + 0.6, base.y));
                oksAt.add(id);
                tapAt.add(null);
                oksIndex.put(id, extra);
                edges.add(new int[]{n, extra});
                weights.add(0.6);
                return;
            }
            if (oksAt.get(n) == null) {
                oksAt.set(n, id);
            }
            oksIndex.put(id, n);
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
                    if (d < 0.8 || d > 22) {
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

        List<Branch> extract(Map<String, Double> oksFlow, Set<String> requiredOks) {
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
            Set<String> markedOks = new HashSet<>();
            for (int i = 0; i < n; i++) {
                if (oksAt.get(i) != null) {
                    terminal[i] = true;
                    oksNodes.add(i);
                    markedOks.add(oksAt.get(i));
                }
                if (tapAt.get(i) != null) {
                    terminal[i] = true;
                    tapNodes.add(i);
                }
            }
            if (oksNodes.isEmpty() || tapNodes.isEmpty()) {
                return null;
            }
            if (requiredOks != null && !markedOks.containsAll(requiredOks)) {
                return null;
            }
            prune(adj, terminal);
            splitFarTaps(adj, tapNodes, 90);
            int[] cc = undirectedComponents(adj);
            Map<Integer, List<Integer>> tapsBy = new HashMap<>();
            Map<Integer, List<Integer>> oksBy = new HashMap<>();
            for (int t : tapNodes) {
                if (cc[t] >= 0) {
                    tapsBy.computeIfAbsent(cc[t], k -> new ArrayList<>()).add(t);
                }
            }
            List<Integer> reachableOks = new ArrayList<>();
            for (int o : oksNodes) {
                if (cc[o] < 0) {
                    continue;
                }
                oksBy.computeIfAbsent(cc[o], k -> new ArrayList<>()).add(o);
                if (!tapsBy.getOrDefault(cc[o], List.of()).isEmpty()) {
                    reachableOks.add(o);
                }
            }
            if (reachableOks.isEmpty()) {
                return null;
            }
            List<Integer> roots = new ArrayList<>();
            for (Map.Entry<Integer, List<Integer>> e : tapsBy.entrySet()) {
                List<Integer> oksHere = oksBy.getOrDefault(e.getKey(), List.of());
                if (oksHere.isEmpty()) {
                    continue;
                }
                int root = pickRoot(e.getValue(), oksHere);
                if (root < 0) {
                    return null;
                }
                roots.add(root);
                for (int t : e.getValue()) {
                    if (t != root && oksAt.get(t) == null) {
                        terminal[t] = false;
                    }
                }
            }
            if (roots.isEmpty()) {
                return null;
            }
            prune(adj, terminal);
            boolean[] keep = new boolean[n];
            for (int r : roots) {
                keep[r] = true;
            }
            for (int o : reachableOks) {
                keep[o] = true;
            }
            for (int i = 0; i < n; i++) {
                if (adj.get(i).size() >= 3) {
                    keep[i] = true;
                }
            }
            List<Branch> out = new ArrayList<>();
            Set<String> got = new HashSet<>();
            for (int root : roots) {
                extractComponent(adj, keep, root, reachableOks, oksFlow, out, got);
            }
            if (requiredOks != null && !got.containsAll(requiredOks)) {
                return null;
            }
            return out;
        }

        private void extractComponent(List<List<int[]>> adj, boolean[] keep, int root,
                                      List<Integer> oksNodes, Map<String, Double> oksFlow,
                                      List<Branch> out, Set<String> got) {
            int n = adj.size();
            int[] parent = bfsParent(adj, root);
            Map<Integer, Double> flow = new HashMap<>();
            Set<String> counted = new HashSet<>();
            for (int o : oksNodes) {
                if (parent[o] < 0 && o != root) {
                    continue;
                }
                String oksId = oksAt.get(o);
                if (oksId == null || !counted.add(oksId)) {
                    continue;
                }
                got.add(oksId);
                double f = oksFlow == null ? 0.01 : oksFlow.getOrDefault(oksId, 0.01);
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
            if (oksAt.get(root) != null) {
                got.add(oksAt.get(root));
            }
            for (int i = 0; i < n; i++) {
                if (!keep[i] || i == root || parent[i] < 0) {
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
                b.tapId = nextKeep == root && tapAt.get(root) != null ? tapAt.get(root).id : null;
                b.flow = Math.max(0.01, flow.getOrDefault(i, 0.01));
                out.add(b);
            }
        }

        private void splitFarTaps(List<List<int[]>> adj, List<Integer> tapNodes, double mergeM) {
            if (tapNodes == null || tapNodes.size() < 2) {
                return;
            }
            int n = adj.size();
            int[] region = new int[n];
            Arrays.fill(region, -1);
            Map<Integer, Integer> tapRegion = new HashMap<>();
            boolean[] used = new boolean[tapNodes.size()];
            for (int i = 0; i < tapNodes.size(); i++) {
                if (used[i]) {
                    continue;
                }
                int rep = tapNodes.get(i);
                used[i] = true;
                tapRegion.put(rep, rep);
                for (int j = i + 1; j < tapNodes.size(); j++) {
                    if (used[j]) {
                        continue;
                    }
                    if (coords.get(rep).distance(coords.get(tapNodes.get(j))) <= mergeM) {
                        used[j] = true;
                        tapRegion.put(tapNodes.get(j), rep);
                    }
                }
            }
            ArrayDeque<Integer> q = new ArrayDeque<>();
            for (Map.Entry<Integer, Integer> e : tapRegion.entrySet()) {
                region[e.getKey()] = e.getValue();
                q.add(e.getKey());
            }
            while (!q.isEmpty()) {
                int u = q.removeFirst();
                for (int[] e : adj.get(u)) {
                    int v = e[0] == u ? e[1] : e[0];
                    if (region[v] < 0) {
                        region[v] = region[u];
                        q.add(v);
                    }
                }
            }
            for (int u = 0; u < n; u++) {
                List<int[]> keepE = new ArrayList<>();
                for (int[] e : adj.get(u)) {
                    int v = e[0] == u ? e[1] : e[0];
                    if (region[u] >= 0 && region[u] == region[v]) {
                        keepE.add(e);
                    }
                }
                adj.set(u, keepE);
            }
        }

        private int[] undirectedComponents(List<List<int[]>> adj) {
            int n = adj.size();
            int[] id = new int[n];
            Arrays.fill(id, -1);
            int c = 0;
            for (int i = 0; i < n; i++) {
                if (id[i] >= 0) {
                    continue;
                }
                ArrayDeque<Integer> q = new ArrayDeque<>();
                q.add(i);
                id[i] = c;
                while (!q.isEmpty()) {
                    int u = q.removeFirst();
                    for (int[] e : adj.get(u)) {
                        int v = e[0] == u ? e[1] : e[0];
                        if (id[v] < 0) {
                            id[v] = c;
                            q.add(v);
                        }
                    }
                }
                c++;
            }
            return id;
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
