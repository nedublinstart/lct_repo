package ru.lct.heatnet.engine;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.greedy.GridPathfinder;
import ru.lct.heatnet.engine.greedy.NetworkSnapper;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.PathSmoother;
import ru.lct.heatnet.engine.greedy.PipeEmitter;
import ru.lct.heatnet.engine.greedy.VisibilityPathfinder;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;

/**
 * Совместное подключение двоичным деревом камер (степень ≤ 4) к лучшей врезке.
 * Точки ИТП, лежащие внутри зданий, выводятся на улицу, трасса идёт по свободному коридору.
 */
@Component
public class SmartRoutingEngine implements RoutingEngine {

    private static final Logger log = LoggerFactory.getLogger(SmartRoutingEngine.class);

    @Override
    public List<Variant> route(Scene scene, AppendixModel appendix, CalculationMode mode, ProgressListener progress) {
        progress.progress(18, "Индексирую препятствия");
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);
        Map<String, Coordinate> ports = ports(scene, obstacles);
        progress.progress(28, "Строю поисковую сетку");
        GridPathfinder grid = GridPathfinder.build(scene, appendix, obstacles);
        VisibilityPathfinder visibility = VisibilityPathfinder.build(obstacles);
        PathCache cache = new PathCache(grid, visibility, obstacles, appendix.getRouting().turnKeepDeg, scene.envelopeMeters);
        List<TapCandidate> candidates = candidates(scene, appendix, obstacles);
        progress.progress(40, "Выбираю точки врезки");

        List<ProspectiveOks> oks = new ArrayList<>();
        for (ProspectiveOks o : scene.oks) {
            if (o.connection != null && o.flowTph > 0) {
                oks.add(o);
            }
        }

        AtomicInteger ids = new AtomicInteger(1);
        List<Variant> variants = new ArrayList<>();
        progress.progress(55, "Дерево к лучшей врезке");
        Variant joint = treeVariant(scene, appendix, cache, obstacles, ports, oks, candidates, ids, false,
                "joint", "Совместное дерево",
                "ОКС собираются двоичным деревом камер (не больше 4 примыканий) и одной врезкой садятся на сеть.");
        add(variants, joint);
        progress.progress(70, "Дерево к существующей камере");
        Variant chamber = treeVariant(scene, appendix, cache, obstacles, ports, oks, candidates, ids, true,
                "chamber", "Врезка в существующую камеру",
                "То же дерево, но врезка в существующую тепловую камеру (правило 10 м и 4 примыкания).");
        if (chamber != null && !sameTaps(joint, chamber)) {
            add(variants, chamber);
        }
        progress.progress(82, "Два куста");
        List<List<ProspectiveOks>> groups = splitTwo(oks);
        if (groups.size() == 2) {
            add(variants, twoTaps(scene, appendix, cache, obstacles, ports, groups, candidates, ids));
        }
        if (variants.size() < 3) {
            NetworkSnapper snapper = new NetworkSnapper(scene, appendix);
            add(variants, independent(scene, appendix, cache, obstacles, ports, snapper, oks, ids));
        }
        log.info("Построено вариантов: {}, ОКС: {}", variants.size(), oks.size());
        return variants;
    }

    private static void add(List<Variant> variants, Variant v) {
        if (v != null) {
            variants.add(v);
        }
    }

    private Map<String, Coordinate> ports(Scene scene, ObstacleIndex obstacles) {
        Map<String, Coordinate> ports = new HashMap<>();
        for (ProspectiveOks o : scene.oks) {
            if (o.connection == null) {
                continue;
            }
            Coordinate origin = o.connection.getCoordinate();
            Coordinate target = nearestNetwork(scene, origin);
            Coordinate at = obstacles.exitToStreet(origin, target, 1.6);
            ports.put(o.id, at);
        }
        return ports;
    }

    private static Coordinate nearestNetwork(Scene scene, Coordinate from) {
        Coordinate best = null;
        double bestD = Double.POSITIVE_INFINITY;
        for (ExistingSegment seg : scene.segments) {
            org.locationtech.jts.operation.distance.DistanceOp op =
                    new org.locationtech.jts.operation.distance.DistanceOp(seg.line, GeoJsonGeometries.GF.createPoint(from));
            Coordinate[] pts = op.nearestPoints();
            double d = from.distance(pts[0]);
            if (d < bestD) {
                bestD = d;
                best = pts[0];
            }
        }
        for (Chamber ch : scene.chambers) {
            double d = from.distance(ch.point.getCoordinate());
            if (d < bestD) {
                bestD = d;
                best = ch.point.getCoordinate();
            }
        }
        return best;
    }

    private Variant treeVariant(Scene scene, AppendixModel appendix, PathCache cache, ObstacleIndex obstacles,
                                Map<String, Coordinate> ports, List<ProspectiveOks> oks, List<TapCandidate> taps,
                                AtomicInteger ids, boolean chambersOnly, String code, String title, String description) {
        if (oks.isEmpty()) {
            return null;
        }
        Builder b = new Builder(appendix, cache, obstacles, null, ids);
        List<Cluster> clusters = new ArrayList<>();
        for (ProspectiveOks o : oks) {
            clusters.add(Cluster.leaf(o, ports.getOrDefault(o.id, o.connection.getCoordinate())));
        }
        agglomerate(clusters, b);
        List<TapCandidate> ranked = rankTaps(taps, clusters, appendix, chambersOnly);
        if (ranked.isEmpty()) {
            for (Cluster c : clusters) {
                b.unconnectedCluster(c);
            }
            return b.finish(code, title, description);
        }
        for (Cluster root : new ArrayList<>(clusters)) {
            boolean attached = false;
            for (TapCandidate tap : ranked) {
                if (b.cache.find(root.at, tap.coordinate) == null) {
                    continue;
                }
                b.tap = tap;
                String node = b.attachTap(root.flow);
                if (b.connect(root, node, tap.coordinate)) {
                    attached = true;
                    break;
                }
            }
            if (!attached) {
                b.unconnectedCluster(root);
            }
        }
        return b.finish(code, title, description);
    }

    private Variant twoTaps(Scene scene, AppendixModel appendix, PathCache cache, ObstacleIndex obstacles,
                            Map<String, Coordinate> ports, List<List<ProspectiveOks>> groups,
                            List<TapCandidate> candidates, AtomicInteger ids) {
        Variant v = null;
        for (List<ProspectiveOks> group : groups) {
            Variant part = treeVariant(scene, appendix, cache, obstacles, ports, group, candidates, ids, false,
                    "twotap", "Два независимых куста",
                    "ОКС разбиты на два пространственных куста, у каждого своя врезка.");
            if (part == null) {
                continue;
            }
            if (v == null) {
                v = part;
            } else {
                v.segments.addAll(part.segments);
                v.chambers.addAll(part.chambers);
                v.taps.addAll(part.taps);
                v.unconnectedOks.addAll(part.unconnectedOks);
                v.unconnectedFlows.putAll(part.unconnectedFlows);
                v.notes.addAll(part.notes);
            }
        }
        if (v != null) {
            v.code = "twotap";
            v.title = "Два независимых куста";
            v.description = "ОКС разбиты на два пространственных куста, у каждого своя врезка.";
        }
        return v;
    }

    private Variant independent(Scene scene, AppendixModel appendix, PathCache cache, ObstacleIndex obstacles,
                                Map<String, Coordinate> ports, NetworkSnapper snapper,
                                List<ProspectiveOks> oks, AtomicInteger ids) {
        Builder b = new Builder(appendix, cache, obstacles, null, ids);
        for (ProspectiveOks o : oks) {
            NetworkSnapper.Snap snap = snapper.snap(o.connection, false);
            if (snap == null) {
                b.unconnected(o);
                continue;
            }
            TapCandidate tap = TapCandidate.of(snap);
            b.tap = tap;
            Coordinate port = ports.getOrDefault(o.id, o.connection.getCoordinate());
            Cluster leaf = Cluster.leaf(o, port);
            String node = b.attachTap(o.flowTph);
            if (!b.connect(leaf, node, tap.coordinate)) {
                b.unconnected(o);
            }
        }
        return b.finish("independent", "Раздельные врезки",
                "Каждый ОКС идёт к ближайшей точке существующей сети своей трассой.");
    }

    private boolean agglomerate(List<Cluster> clusters, Builder b) {
        while (clusters.size() > 1) {
            List<int[]> order = new ArrayList<>();
            for (int i = 0; i < clusters.size(); i++) {
                for (int j = i + 1; j < clusters.size(); j++) {
                    order.add(new int[]{i, j});
                }
            }
            order.sort(Comparator.comparingDouble(ij -> clusters.get(ij[0]).at.distance(clusters.get(ij[1]).at)));
            Cluster left = null;
            Cluster right = null;
            Coordinate mid = null;
            for (int[] ij : order) {
                Cluster a = clusters.get(ij[0]);
                Cluster bCl = clusters.get(ij[1]);
                List<Coordinate> path = b.cache.find(a.at, bCl.at);
                if (path == null) {
                    continue;
                }
                mid = midpoint(path, b.obstacles);
                left = a;
                right = bCl;
                break;
            }
            if (left == null || mid == null) {
                return clusters.size() == 1;
            }
            String chamberId = b.chamber(mid, false);
            if (!b.connect(left, chamberId, mid) || !b.connect(right, chamberId, mid)) {
                return clusters.size() == 1;
            }
            Cluster merged = new Cluster();
            merged.id = chamberId;
            merged.at = mid;
            merged.flow = left.flow + right.flow;
            merged.chamber = true;
            merged.members.addAll(left.members);
            merged.members.addAll(right.members);
            clusters.remove(right);
            clusters.remove(left);
            clusters.add(merged);
        }
        return true;
    }

    private List<TapCandidate> candidates(Scene scene, AppendixModel appendix, ObstacleIndex obstacles) {
        List<TapCandidate> list = new ArrayList<>();
        int maxDeg = appendix.getRouting().maxChamberDegree;
        for (Chamber ch : scene.chambers) {
            if (ch.incidentCount < maxDeg) {
                TapCandidate t = TapCandidate.chamber(ch);
                if (!obstacles.blocked(t.coordinate)) {
                    list.add(t);
                }
            }
        }
        double step = Math.max(20, appendix.getRouting().candidateStepM);
        for (ExistingSegment seg : scene.segments) {
            double len = seg.line.getLength();
            int n = Math.max(1, (int) Math.round(len / step));
            LengthIndexedLine lil = new LengthIndexedLine(seg.line);
            for (int i = 0; i <= n; i++) {
                Coordinate c = lil.extractPoint(len * i / n);
                boolean nearChamber = false;
                for (Chamber ch : scene.chambers) {
                    if (ch.point.getCoordinate().distance(c) <= appendix.getRouting().chamberSnapM) {
                        nearChamber = true;
                        break;
                    }
                }
                if (!nearChamber && !obstacles.blocked(c)) {
                    list.add(TapCandidate.segment(seg, c));
                }
            }
        }
        return list;
    }

    private List<TapCandidate> rankTaps(List<TapCandidate> candidates, List<Cluster> clusters,
                                        AppendixModel appendix, boolean chambersOnly) {
        Coordinate mean = new Coordinate(0, 0);
        double flow = 0;
        int n = 0;
        for (Cluster c : clusters) {
            mean.x += c.at.x;
            mean.y += c.at.y;
            flow += c.flow;
            n++;
        }
        if (n == 0) {
            return List.of();
        }
        mean.x /= n;
        mean.y /= n;
        List<TapCandidate> ranked = new ArrayList<>();
        for (TapCandidate tap : candidates) {
            if (chambersOnly && !tap.chamber) {
                continue;
            }
            ranked.add(tap);
        }
        double totalFlow = flow;
        ranked.sort(Comparator.comparingDouble(tap -> {
            double s = tap.coordinate.distance(mean);
            if (tap.chamber) {
                s *= 0.90;
            }
            double cap = capacity(tap.existingDn, appendix);
            if (cap + 1e-9 < totalFlow) {
                s += 700;
            } else {
                s -= Math.min(80, tap.existingDn * 0.08);
            }
            return s;
        }));
        return ranked;
    }

    private static boolean sameTaps(Variant a, Variant b) {
        if (a == null || b == null || a.taps.isEmpty() || b.taps.isEmpty()) {
            return false;
        }
        return a.taps.get(0).existingObjectId.equals(b.taps.get(0).existingObjectId)
                && a.taps.size() == b.taps.size();
    }

    private TapCandidate pickTap(List<TapCandidate> candidates, List<ProspectiveOks> oks,
                                 Map<String, Coordinate> ports, AppendixModel appendix,
                                 double totalFlow, boolean chambersOnly, String excludeKey) {
        TapCandidate best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        for (TapCandidate tap : candidates) {
            if (chambersOnly && !tap.chamber) {
                continue;
            }
            if (excludeKey != null && excludeKey.equals(tap.key())) {
                continue;
            }
            double s = 0;
            for (ProspectiveOks o : oks) {
                Coordinate at = ports.getOrDefault(o.id, o.connection.getCoordinate());
                s += at.distance(tap.coordinate);
            }
            if (tap.chamber) {
                s *= 0.90;
            }
            double cap = capacity(tap.existingDn, appendix);
            if (cap + 1e-9 < totalFlow) {
                s += 700;
            } else {
                s -= Math.min(80, tap.existingDn * 0.08);
            }
            if (s < bestScore) {
                bestScore = s;
                best = tap;
            }
        }
        return best;
    }

    private static double capacity(int dn, AppendixModel appendix) {
        if (dn <= 0) {
            return 0;
        }
        AppendixModel.DiameterSpec spec = appendix.diameter(dn);
        return spec == null ? 0 : spec.capacityTph;
    }

    private List<List<ProspectiveOks>> splitTwo(List<ProspectiveOks> oks) {
        if (oks.size() < 4) {
            return List.of();
        }
        Coordinate mean = new Coordinate(0, 0);
        for (ProspectiveOks o : oks) {
            mean.x += o.connection.getX();
            mean.y += o.connection.getY();
        }
        mean.x /= oks.size();
        mean.y /= oks.size();
        ProspectiveOks far = oks.get(0);
        double farD = -1;
        for (ProspectiveOks o : oks) {
            double d = o.connection.getCoordinate().distance(mean);
            if (d > farD) {
                farD = d;
                far = o;
            }
        }
        double ax = far.connection.getX() - mean.x;
        double ay = far.connection.getY() - mean.y;
        List<ProspectiveOks> a = new ArrayList<>();
        List<ProspectiveOks> b = new ArrayList<>();
        for (ProspectiveOks o : oks) {
            double dx = o.connection.getX() - mean.x;
            double dy = o.connection.getY() - mean.y;
            if (dx * ax + dy * ay >= 0) {
                a.add(o);
            } else {
                b.add(o);
            }
        }
        if (a.isEmpty() || b.isEmpty()) {
            return List.of();
        }
        return List.of(a, b);
    }

    private static Coordinate midpoint(List<Coordinate> path, ObstacleIndex obstacles) {
        double total = length(path);
        double acc = 0;
        Coordinate fallback = path.get(path.size() / 2);
        Coordinate mid = null;
        for (int i = 1; i < path.size(); i++) {
            double d = path.get(i - 1).distance(path.get(i));
            if (acc + d >= total / 2) {
                double t = d < 1e-6 ? 0 : (total / 2 - acc) / d;
                mid = new Coordinate(
                        path.get(i - 1).x + t * (path.get(i).x - path.get(i - 1).x),
                        path.get(i - 1).y + t * (path.get(i).y - path.get(i - 1).y));
                break;
            }
            acc += d;
        }
        if (mid == null) {
            mid = fallback;
        }
        Coordinate sidewalk = snapOffRoad(path, obstacles, mid);
        if (sidewalk != null) {
            return sidewalk;
        }
        if (!obstacles.blocked(mid)) {
            return mid;
        }
        for (Coordinate c : path) {
            if (!obstacles.blocked(c) && !obstacles.inRoad(c)) {
                return c;
            }
        }
        for (Coordinate c : path) {
            if (!obstacles.blocked(c)) {
                return c;
            }
        }
        return fallback;
    }

    private static Coordinate snapOffRoad(List<Coordinate> path, ObstacleIndex obstacles, Coordinate mid) {
        Coordinate best = null;
        double bestD = Double.POSITIVE_INFINITY;
        for (Coordinate c : path) {
            if (obstacles.blocked(c) || obstacles.inRoad(c)) {
                continue;
            }
            double d = c.distance(mid);
            if (d < bestD) {
                bestD = d;
                best = c;
            }
        }
        if (best != null && bestD < 80) {
            return best;
        }
        if (!obstacles.blocked(mid) && !obstacles.inRoad(mid)) {
            return mid;
        }
        return best;
    }

    private static double length(List<Coordinate> path) {
        double s = 0;
        for (int i = 1; i < path.size(); i++) {
            s += path.get(i - 1).distance(path.get(i));
        }
        return s;
    }

    private static final class Cluster {
        String id;
        Coordinate at;
        Coordinate origin;
        double flow;
        boolean chamber;
        final List<ProspectiveOks> members = new ArrayList<>();

        static Cluster leaf(ProspectiveOks o, Coordinate port) {
            Cluster c = new Cluster();
            c.id = o.id;
            c.at = port == null ? o.connection.getCoordinate() : port;
            c.origin = o.connection.getCoordinate();
            c.flow = o.flowTph;
            c.members.add(o);
            return c;
        }
    }

    static final class TapCandidate {
        final String id;
        final Coordinate coordinate;
        final boolean chamber;
        final String existingId;
        final String existingKind;
        final int existingDn;

        private TapCandidate(String id, Coordinate coordinate, boolean chamber, String existingId, String existingKind, int existingDn) {
            this.id = id;
            this.coordinate = coordinate;
            this.chamber = chamber;
            this.existingId = existingId;
            this.existingKind = existingKind;
            this.existingDn = existingDn;
        }

        static TapCandidate chamber(Chamber ch) {
            return new TapCandidate("TAP-" + ch.id, ch.point.getCoordinate(), true, ch.id, "heat_chamber", ch.dn);
        }

        static TapCandidate segment(ExistingSegment seg, Coordinate c) {
            return new TapCandidate("TAP-" + seg.id + "-" + Math.round(c.x) + "-" + Math.round(c.y),
                    c, false, seg.id, "heat_network", seg.dn);
        }

        static TapCandidate of(NetworkSnapper.Snap snap) {
            boolean ch = "chamber".equals(snap.kind) || "source".equals(snap.kind);
            String kind = ch ? "heat_chamber" : "heat_network";
            return new TapCandidate("TAP-" + snap.objectId, snap.coordinate, ch, snap.objectId, kind, snap.dn);
        }

        String key() {
            return existingKind + ":" + existingId + ":" + Math.round(coordinate.x) + ":" + Math.round(coordinate.y);
        }
    }

    private static final class PathCache {
        private final GridPathfinder grid;
        private final VisibilityPathfinder visibility;
        private final ObstacleIndex obstacles;
        private final double keepDeg;
        private final Envelope env;
        private final Map<String, List<Coordinate>> cache = new HashMap<>();

        PathCache(GridPathfinder grid, VisibilityPathfinder visibility, ObstacleIndex obstacles, double keepDeg, Envelope env) {
            this.grid = grid;
            this.visibility = visibility;
            this.obstacles = obstacles;
            this.keepDeg = keepDeg;
            this.env = env;
        }

        List<Coordinate> find(Coordinate a, Coordinate b) {
            String k = key(a) + ">" + key(b);
            if (cache.containsKey(k)) {
                return cache.get(k);
            }
            String kr = key(b) + ">" + key(a);
            if (cache.containsKey(kr) && cache.get(kr) != null) {
                List<Coordinate> rev = new ArrayList<>(cache.get(kr));
                java.util.Collections.reverse(rev);
                cache.put(k, rev);
                return rev;
            }
            List<Coordinate> path = visibility.find(a, b);
            if (path == null) {
                path = grid.find(a, b);
            }
            if (path != null) {
                path = PathSmoother.smooth(path, obstacles, keepDeg);
                if (obstacles.pathHitsAvoid(path, 0)) {
                    path = visibility.find(a, b);
                    if (path == null) {
                        path = grid.find(a, b);
                    }
                }
            } else {
                path = via(a, b);
            }
            cache.put(k, path);
            return path;
        }

        private List<Coordinate> via(Coordinate a, Coordinate b) {
            if (env == null || env.isNull()) {
                return null;
            }
            double minX = env.getMinX() + 8;
            double maxX = env.getMaxX() - 8;
            double minY = env.getMinY() + 8;
            double maxY = env.getMaxY() - 8;
            double cx = (minX + maxX) / 2;
            double cy = (minY + maxY) / 2;
            Coordinate[] wps = {
                    new Coordinate(cx, minY),
                    new Coordinate(cx, maxY),
                    new Coordinate(minX, cy),
                    new Coordinate(maxX, cy)
            };
            List<Coordinate> best = null;
            double bestLen = Double.POSITIVE_INFINITY;
            for (Coordinate wp : wps) {
                List<Coordinate> p1 = grid.find(a, wp);
                List<Coordinate> p2 = grid.find(wp, b);
                if (p1 == null || p2 == null) {
                    continue;
                }
                List<Coordinate> joined = new ArrayList<>(p1);
                joined.addAll(p2.subList(1, p2.size()));
                joined = PathSmoother.smooth(joined, obstacles, keepDeg);
                double len = 0;
                for (int i = 1; i < joined.size(); i++) {
                    len += joined.get(i - 1).distance(joined.get(i));
                }
                if (len < bestLen) {
                    bestLen = len;
                    best = joined;
                }
            }
            return best;
        }

        private static String key(Coordinate c) {
            return Math.round(c.x * 2) + ":" + Math.round(c.y * 2);
        }
    }

    private static final class Builder {
        final AppendixModel appendix;
        final PathCache cache;
        final ObstacleIndex obstacles;
        TapCandidate tap;
        final AtomicInteger ids;
        final Variant variant = new Variant();
        String lastTapNode;

        Builder(AppendixModel appendix, PathCache cache, ObstacleIndex obstacles, TapCandidate tap, AtomicInteger ids) {
            this.appendix = appendix;
            this.cache = cache;
            this.obstacles = obstacles;
            this.tap = tap;
            this.ids = ids;
        }

        void unconnected(ProspectiveOks o) {
            variant.unconnectedOks.add(o.id);
            variant.unconnectedFlows.put(o.id, o.flowTph);
            variant.notes.add("Маршрут не найден для ОКС " + o.id);
        }

        void unconnectedCluster(Cluster cluster) {
            for (ProspectiveOks o : cluster.members) {
                unconnected(o);
            }
        }

        void unconnectedAll(List<ProspectiveOks> oks) {
            for (ProspectiveOks o : oks) {
                unconnected(o);
            }
        }

        String chamber(Coordinate c, boolean atTap) {
            NewChamber ch = new NewChamber();
            ch.id = "CH-" + ids.getAndIncrement();
            ch.geometryMeters = GeoJsonGeometries.GF.createPoint(c);
            ch.atTap = atTap;
            variant.chambers.add(ch);
            return ch.id;
        }

        boolean connect(Cluster from, String toId, Coordinate to) {
            List<Coordinate> path = cache.find(from.at, to);
            if (path == null) {
                return false;
            }
            if (from.origin != null && from.origin.distance(from.at) > 0.4) {
                List<Coordinate> full = new ArrayList<>();
                full.add(new Coordinate(from.origin));
                full.addAll(path);
                path = full;
            }
            pipe(from.id, toId, from.flow, path);
            return true;
        }

        void pipe(String from, String to, double flow, List<Coordinate> path) {
            PipeEmitter.emit(variant, obstacles, ids, from, to, flow, path);
        }

        String attachTap(double flow) {
            if (tap == null) {
                return lastTapNode;
            }
            for (TapPoint t : variant.taps) {
                if (t.existingObjectId.equals(tap.existingId)
                        && t.geometryMeters.getCoordinate().distance(tap.coordinate) < 1.0) {
                    t.extraFlowTph += flow;
                    lastTapNode = t.nodeId != null ? t.nodeId : t.id;
                    return lastTapNode;
                }
            }
            String nodeId;
            if (tap.chamber) {
                nodeId = tap.existingId;
            } else {
                nodeId = chamber(tap.coordinate, true);
            }
            TapPoint t = new TapPoint();
            t.id = "TI-" + ids.getAndIncrement();
            t.nodeId = nodeId;
            t.geometryMeters = GeoJsonGeometries.GF.createPoint(tap.coordinate);
            t.existingObjectId = tap.existingId;
            t.existingObjectKind = tap.existingKind;
            t.existingDiameter = tap.existingDn;
            t.extraFlowTph = flow;
            t.cost = appendix.getCosts().tapInPipe;
            variant.taps.add(t);
            lastTapNode = nodeId;
            return nodeId;
        }

        Variant finish(String code, String title, String description) {
            variant.code = code;
            variant.title = title;
            variant.description = description;
            variant.segments.sort(Comparator.comparing(s -> s.id));
            return variant;
        }
    }
}
