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
import org.locationtech.jts.operation.distance.DistanceOp;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.OrthoPaths;
import ru.lct.heatnet.engine.greedy.PathSmoother;
import ru.lct.heatnet.engine.greedy.PipeEmitter;
import ru.lct.heatnet.engine.greedy.StreetFrame;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;

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
            if (b.path == null || b.path.size() < 2) {
                continue;
            }
            String to = b.tapId != null ? nodeOf(taps, b.tapId)
                    : junction(variant, ids, b.path.get(b.path.size() - 1), nodeAt);
            if (b.oksId != null) {
                ItpSnapper.Cut cut = ItpSnapper.cutStub(obstacles, b.path.get(0), b.path);
                List<Coordinate> stub = PathSmoother.collapseKeepStub(cut.stub, obstacles);
                double stubFlow = oksFlow != null ? oksFlow.getOrDefault(b.oksId, b.flow) : b.flow;
                if (stub == null || stub.size() < 2) {
                    continue;
                }
                if (cut.rest.size() < 2) {
                    if (to == null || b.oksId.equals(to)) {
                        continue;
                    }
                    PipeEmitter.emit(variant, obstacles, ids, b.oksId, to, Math.max(0.01, stubFlow), stub);
                    continue;
                }
                String hub = junction(variant, ids, cut.joinAt, nodeAt);
                if (hub == null || b.oksId.equals(hub)) {
                    continue;
                }
                PipeEmitter.emit(variant, obstacles, ids, b.oksId, hub, Math.max(0.01, stubFlow), stub);
                if (hub.equals(to) || to == null) {
                    continue;
                }
                List<Coordinate> rest = keepPath(cut.rest, obstacles);
                if (rest != null && rest.size() >= 2) {
                    PipeEmitter.emit(variant, obstacles, ids, hub, to, Math.max(0.01, b.flow), rest);
                }
                continue;
            }
            List<Coordinate> path = PathSmoother.refine(b.path, obstacles);
            if (path == null || path.size() < 2) {
                continue;
            }
            String from = junction(variant, ids, path.get(0), nodeAt);
            if (from == null || to == null || from.equals(to)) {
                continue;
            }
            PipeEmitter.emit(variant, obstacles, ids, from, to, Math.max(0.01, b.flow), path);
        }
        ensureTapChambers(variant);
        graftMissing(variant, savedSegs, savedTaps, savedChambers, savedNodes, oksIds, oksFlow);
        graftOrphans(variant, savedSegs, savedTaps, oksIds);
        dropDuplicateTaps(variant);
    }

    static List<Coordinate> keepPath(List<Coordinate> raw, ObstacleIndex obstacles) {
        if (raw == null || raw.size() < 2) {
            return raw;
        }
        List<Coordinate> refined = PathSmoother.refine(raw, obstacles);
        if (refined != null && refined.size() >= 2) {
            return refined;
        }
        List<Coordinate> polished = PathSmoother.emitPolish(raw, obstacles);
        if (polished != null && polished.size() >= 2) {
            return polished;
        }
        List<Coordinate> copy = new ArrayList<>();
        for (Coordinate c : raw) {
            if (c != null) {
                copy.add(new Coordinate(c));
            }
        }
        return copy.size() >= 2 ? copy : raw;
    }

    private static final double WIRE_M = 8.0;
    private static final double EXIST_WIRE_M = 3.5;
    private static final double BRIDGE_NEAR_M = 128.0;
    private static final double BRIDGE_TAP_M = 48.0;

    /**
     * Компонента без врезки стыкуется коротким пролётом к дереву, у которого
     * врезка уже есть, либо к геометрии этой врезки. Третью врезку не ставим.
     */
    public static void stitchToExisting(Variant variant, Scene scene,
                                        ObstacleIndex obstacles, StreetFrame frame,
                                        AtomicInteger ids, List<OksPort> ports) {
        if (variant == null || variant.segments.isEmpty()) {
            return;
        }
        Map<String, OksPort> byId = new HashMap<>();
        if (ports != null) {
            for (OksPort p : ports) {
                if (p != null && p.id() != null) {
                    byId.put(p.id(), p);
                }
            }
        }
        for (int round = 0; round < 8; round++) {
            boolean changed = false;
            Map<String, Integer> compOf = nodeComponents(variant);
            Map<Integer, Comp> comps = groupComps(variant, compOf, byId);
            attachTapsByGeometry(variant, comps, 4.5);
            List<Comp> rooted = new ArrayList<>();
            List<Comp> islands = new ArrayList<>();
            for (Comp c : comps.values()) {
                if (c.oks.isEmpty()) {
                    continue;
                }
                if (c.tap != null && hitsTapId(c, c.tap)) {
                    rooted.add(c);
                } else {
                    islands.add(c);
                }
            }
            for (Comp island : islands) {
                if (wireOntoAnyTap(variant, scene, obstacles, ids, island)) {
                    changed = true;
                    continue;
                }
                if (wireOntoExisting(variant, scene, obstacles, ids, island)) {
                    changed = true;
                    continue;
                }
                if (bridgeToRooted(variant, obstacles, frame, ids, island, rooted, BRIDGE_NEAR_M)) {
                    changed = true;
                    continue;
                }
                if (bridgeToKnownTap(variant, obstacles, frame, ids, island, BRIDGE_TAP_M)) {
                    changed = true;
                    continue;
                }
                if (variant.taps.size() < 2 && scene != null
                        && bridgeNewTap(variant, scene, obstacles, frame, ids, island, BRIDGE_NEAR_M)) {
                    changed = true;
                }
            }
            if (!changed) {
                break;
            }
        }
        stitchLostPorts(variant, scene, obstacles, frame, ids, ports);
        dropDuplicateTaps(variant);
    }

    /**
     * Две врезки в одной компоненте — лишняя. Оставляем ту, к которой
     * реально приходят трубы.
     */
    public static void dropDuplicateTaps(Variant variant) {
        if (variant == null || variant.taps.size() < 2) {
            return;
        }
        Map<String, Integer> compOf = nodeComponents(variant);
        Map<Integer, List<TapPoint>> groups = new HashMap<>();
        for (TapPoint t : variant.taps) {
            if (t == null) {
                continue;
            }
            Integer c = t.nodeId != null ? compOf.get(t.nodeId) : null;
            if (c == null && t.id != null) {
                c = compOf.get(t.id);
            }
            if (c == null) {
                continue;
            }
            groups.computeIfAbsent(c, k -> new ArrayList<>()).add(t);
        }
        Set<String> drop = new HashSet<>();
        for (List<TapPoint> g : groups.values()) {
            if (g.size() < 2) {
                continue;
            }
            TapPoint keep = g.get(0);
            int best = tapDegree(variant, keep);
            for (TapPoint t : g) {
                int d = tapDegree(variant, t);
                if (d > best) {
                    best = d;
                    keep = t;
                }
            }
            for (TapPoint t : g) {
                if (t != keep && t.id != null) {
                    drop.add(t.id);
                }
            }
        }
        if (drop.isEmpty()) {
            return;
        }
        variant.taps.removeIf(t -> t == null || (t.id != null && drop.contains(t.id)));
        Set<String> used = new HashSet<>();
        for (NewSegment s : variant.segments) {
            if (s.fromId != null) {
                used.add(s.fromId);
            }
            if (s.toId != null) {
                used.add(s.toId);
            }
        }
        for (TapPoint t : variant.taps) {
            if (t.nodeId != null) {
                used.add(t.nodeId);
            }
        }
        variant.chambers.removeIf(c -> c != null && c.atTap && c.id != null && !used.contains(c.id));
    }

    private static int tapDegree(Variant variant, TapPoint t) {
        int n = 0;
        for (NewSegment s : variant.segments) {
            if (named(t, s.fromId) || named(t, s.toId)) {
                n++;
            }
        }
        return n;
    }

    private static void stitchLostPorts(Variant variant, Scene scene, ObstacleIndex obstacles,
                                        StreetFrame frame, AtomicInteger ids, List<OksPort> ports) {
        if (ports == null || ports.isEmpty()) {
            return;
        }
        for (int round = 0; round < 4; round++) {
            Set<String> tapNow = tapIds(variant);
            Set<String> rooted = nodesReaching(variant, tapNow);
            boolean changed = false;
            for (OksPort p : ports) {
                if (p == null || p.id() == null || rooted.contains(p.id())) {
                    continue;
                }
                if (attachLostPort(variant, scene, obstacles, frame, ids, p, rooted)) {
                    changed = true;
                    tapNow = tapIds(variant);
                    rooted = nodesReaching(variant, tapNow);
                }
            }
            if (!changed) {
                break;
            }
        }
    }

    private static boolean attachLostPort(Variant variant, Scene scene, ObstacleIndex obstacles,
                                          StreetFrame frame, AtomicInteger ids, OksPort port,
                                          Set<String> rooted) {
        String fromId = port.id();
        Coordinate from = port.origin != null ? port.origin : port.at;
        double stubBest = Double.POSITIVE_INFINITY;
        for (NewSegment s : variant.segments) {
            if (s.geometryMeters == null || s.fromId == null) {
                continue;
            }
            if (!port.id().equals(s.fromId) && !port.id().equals(s.toId)) {
                continue;
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            Coordinate far = port.id().equals(s.fromId) ? pts[pts.length - 1] : pts[0];
            String farId = port.id().equals(s.fromId) ? s.toId : s.fromId;
            if (far == null || farId == null || rooted.contains(farId)) {
                continue;
            }
            double score = far.distance(from == null ? far : from);
            if (score < stubBest) {
                stubBest = score;
                from = far;
                fromId = farId;
            }
        }
        if (from == null || fromId == null) {
            return false;
        }
        Coordinate goal = null;
        String toId = null;
        double bestD = Double.POSITIVE_INFINITY;
        for (TapPoint t : variant.taps) {
            if (t == null || t.geometryMeters == null) {
                continue;
            }
            Coordinate at = t.geometryMeters.getCoordinate();
            double d = from.distance(at);
            if (d < bestD) {
                bestD = d;
                goal = at;
                toId = t.nodeId != null ? t.nodeId : t.id;
            }
        }
        for (NewSegment s : variant.segments) {
            if (s.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            if (rooted.contains(s.fromId) && pts[0].distance(from) < bestD) {
                bestD = pts[0].distance(from);
                goal = pts[0];
                toId = s.fromId;
            }
            if (rooted.contains(s.toId) && pts[pts.length - 1].distance(from) < bestD) {
                bestD = pts[pts.length - 1].distance(from);
                goal = pts[pts.length - 1];
                toId = s.toId;
            }
        }
        if (goal == null || toId == null || fromId.equals(toId)) {
            return false;
        }
        if (bestD > BRIDGE_TAP_M + 16) {
            if (wireOntoExisting(variant, scene, obstacles, ids, fromId, from, 0.01)) {
                return true;
            }
            return false;
        }
        Coordinate start = from;
        if (obstacles != null && obstacles.blocked(from)) {
            Coordinate exit = obstacles.exitToStreet(from, goal, 1.2);
            if (exit != null) {
                start = exit;
            }
        }
        List<Coordinate> path = shortPath(obstacles, frame, start, goal, BRIDGE_TAP_M);
        if (path == null || path.size() < 2) {
            return wireOntoExisting(variant, scene, obstacles, ids, fromId, from, 0.01);
        }
        PipeEmitter.emit(variant, obstacles, ids, fromId, toId, Math.max(0.01, port.flow()), path);
        return true;
    }

    private static boolean hitsTapId(Comp c, TapPoint tap) {
        if (c == null || tap == null) {
            return false;
        }
        for (NewSegment s : c.segs) {
            if (named(tap, s.fromId) || named(tap, s.toId)) {
                return true;
            }
        }
        return false;
    }

    private static boolean named(TapPoint tap, String id) {
        return id != null && (id.equals(tap.id) || id.equals(tap.nodeId) || id.equals(tap.existingObjectId));
    }

    private static boolean wireOntoExisting(Variant variant, Scene scene, ObstacleIndex obstacles,
                                            AtomicInteger ids, Comp island) {
        if (island == null) {
            return false;
        }
        for (NewSegment s : island.segs) {
            if (s.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            double flow = islandFlow(island);
            if (s.fromId != null
                    && wireOntoExisting(variant, scene, obstacles, ids, s.fromId, pts[0], flow)) {
                return true;
            }
            if (s.toId != null
                    && wireOntoExisting(variant, scene, obstacles, ids, s.toId, pts[pts.length - 1], flow)) {
                return true;
            }
        }
        return false;
    }

    private static boolean wireOntoExisting(Variant variant, Scene scene, ObstacleIndex obstacles,
                                            AtomicInteger ids, String fromId, Coordinate from, double flow) {
        if (scene == null || variant == null || fromId == null || from == null || variant.taps.isEmpty()) {
            return false;
        }
        Coordinate exist = nearestExisting(scene, from);
        if (exist == null || from.distance(exist) > EXIST_WIRE_M) {
            return false;
        }
        String existId = existingIdAt(scene, exist);
        String toId = null;
        double best = Double.POSITIVE_INFINITY;
        for (TapPoint t : variant.taps) {
            if (t == null) {
                continue;
            }
            String id = t.nodeId != null ? t.nodeId : t.id;
            if (id == null || id.equals(fromId)) {
                continue;
            }
            if (existId != null && existId.equals(t.existingObjectId)) {
                toId = id;
                break;
            }
            if (t.geometryMeters == null) {
                continue;
            }
            double d = from.distance(t.geometryMeters.getCoordinate());
            if (d < best) {
                best = d;
                toId = id;
            }
        }
        if (toId == null) {
            return false;
        }
        List<Coordinate> path = new ArrayList<>();
        path.add(new Coordinate(from));
        if (from.distance(exist) >= 0.3) {
            path.add(new Coordinate(exist));
        } else {
            path.add(new Coordinate(exist.x + 0.4, exist.y));
        }
        PipeEmitter.emit(variant, obstacles, ids, fromId, toId, Math.max(0.01, flow), path);
        return true;
    }

    private static void attachTapsByGeometry(Variant variant, Map<Integer, Comp> comps, double m) {
        for (TapPoint t : variant.taps) {
            if (t == null || t.geometryMeters == null) {
                continue;
            }
            boolean already = false;
            for (Comp c : comps.values()) {
                if (c.tap == t) {
                    already = true;
                    break;
                }
            }
            if (already) {
                continue;
            }
            Coordinate at = t.geometryMeters.getCoordinate();
            Comp best = null;
            double bestD = m;
            for (Comp c : comps.values()) {
                if (c.oks.isEmpty()) {
                    continue;
                }
                for (NewSegment s : c.segs) {
                    if (s.geometryMeters == null) {
                        continue;
                    }
                    for (Coordinate p : s.geometryMeters.getCoordinates()) {
                        if (p != null && p.distance(at) < bestD) {
                            bestD = p.distance(at);
                            best = c;
                        }
                    }
                }
            }
            if (best != null && best.tap == null) {
                best.tap = t;
            }
        }
    }

    private static boolean wireOntoAnyTap(Variant variant, Scene scene, ObstacleIndex obstacles,
                                          AtomicInteger ids, Comp island) {
        for (TapPoint t : variant.taps) {
            if (wireOntoTap(variant, scene, obstacles, ids, island, t)) {
                return true;
            }
        }
        return false;
    }

    private static boolean wireOntoTap(Variant variant, Scene scene, ObstacleIndex obstacles,
                                       AtomicInteger ids, Comp island, TapPoint tap) {
        if (tap == null) {
            return false;
        }
        Coordinate from = null;
        String fromId = null;
        Coordinate at = null;
        double bestD = WIRE_M;
        for (NewSegment s : island.segs) {
            if (s.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            Coordinate[] ends = {pts[0], pts[pts.length - 1]};
            String[] idsAt = {s.fromId, s.toId};
            for (int i = 0; i < ends.length; i++) {
                Coordinate goal = pointOnTapObject(scene, tap, ends[i]);
                if (goal == null) {
                    continue;
                }
                double d = ends[i].distance(goal);
                if (d < bestD && idsAt[i] != null) {
                    bestD = d;
                    from = ends[i];
                    fromId = idsAt[i];
                    at = goal;
                }
            }
        }
        String toId = tap.nodeId != null ? tap.nodeId : tap.id;
        if (from == null || fromId == null || toId == null || fromId.equals(toId) || at == null) {
            return false;
        }
        List<Coordinate> path = new ArrayList<>();
        path.add(new Coordinate(from));
        if (from.distance(at) >= 0.3) {
            path.add(new Coordinate(at));
        } else {
            Coordinate nudge = new Coordinate(at.x + 0.4, at.y);
            path.add(nudge);
        }
        PipeEmitter.emit(variant, obstacles, ids, fromId, toId, islandFlow(island), path);
        return true;
    }

    private static Coordinate pointOnTapObject(Scene scene, TapPoint tap, Coordinate from) {
        if (tap == null || from == null) {
            return null;
        }
        Coordinate best = null;
        double bestD = WIRE_M;
        if (tap.geometryMeters != null) {
            Coordinate c = tap.geometryMeters.getCoordinate();
            if (from.distance(c) < bestD) {
                bestD = from.distance(c);
                best = c;
            }
        }
        if (scene == null || tap.existingObjectId == null) {
            return best;
        }
        for (Chamber ch : scene.chambers) {
            if (ch.point == null || !tap.existingObjectId.equals(ch.id)) {
                continue;
            }
            Coordinate c = ch.point.getCoordinate();
            if (from.distance(c) < bestD) {
                bestD = from.distance(c);
                best = c;
            }
        }
        for (ExistingSegment seg : scene.segments) {
            if (seg.line == null || !tap.existingObjectId.equals(seg.id)) {
                continue;
            }
            DistanceOp op = new DistanceOp(seg.line, GeoJsonGeometries.GF.createPoint(from));
            Coordinate[] pts = op.nearestPoints();
            if (pts.length > 0 && from.distance(pts[0]) < bestD) {
                bestD = from.distance(pts[0]);
                best = pts[0];
            }
        }
        return best;
    }

    private static double islandFlow(Comp c) {
        double f = 0;
        Set<String> seen = new HashSet<>();
        if (c != null) {
            for (OksPort p : c.oks) {
                if (p == null || p.id() == null || !seen.add(p.id())) {
                    continue;
                }
                f += Math.max(0.0, p.flow());
            }
            if (f < 0.02) {
                for (NewSegment s : c.segs) {
                    if (s == null || !looksLikeOks(s.fromId) || !seen.add(s.fromId)) {
                        continue;
                    }
                    f += Math.max(0.01, s.flowTph);
                }
            }
        }
        return Math.max(0.01, f);
    }

    private static boolean hasOks(Comp g, String id) {
        if (g == null || id == null) {
            return false;
        }
        for (OksPort p : g.oks) {
            if (p != null && id.equals(p.id())) {
                return true;
            }
        }
        return false;
    }

    private static OksPort syntheticPort(String id, double flow, Coordinate at) {
        ProspectiveOks oks = new ProspectiveOks();
        oks.id = id;
        oks.flowTph = Math.max(0.01, flow);
        Coordinate c = at == null ? new Coordinate() : new Coordinate(at);
        return new OksPort(oks, c, c);
    }

    private static boolean looksLikeOks(String id) {
        if (id == null || id.isBlank()) {
            return false;
        }
        if (id.startsWith("TN-") || id.startsWith("CH-") || id.startsWith("TI-") || id.startsWith("NS-")) {
            return false;
        }
        if (id.startsWith("OKS") || id.startsWith("oks")) {
            return true;
        }
        try {
            int n = Integer.parseInt(id);
            return n > 0 && n < 100;
        } catch (NumberFormatException e) {
            return true;
        }
    }

    private static boolean bridgeToRooted(Variant variant, ObstacleIndex obstacles, StreetFrame frame,
                                          AtomicInteger ids, Comp island, List<Comp> rooted, double capM) {
        List<Coordinate> bestPath = null;
        double bestLen = 220;
        String bestFrom = null;
        String bestTo = null;
        for (Comp r : rooted) {
            List<Pair> pairs = candidatePairs(island, r, capM + 24);
            int tried = 0;
            for (Pair p : pairs) {
                if (tried++ >= 24) {
                    break;
                }
                List<Coordinate> path = shortPath(obstacles, frame, p.a, p.b, capM);
                if (path == null || path.size() < 2) {
                    continue;
                }
                String from = nearestNodeId(variant, p.a, 80);
                String to = nearestNodeId(variant, p.b, 80);
                if (from == null || to == null || from.equals(to)) {
                    continue;
                }
                double len = OrthoPaths.length(path);
                if (len + 8 < p.dist) {
                    continue;
                }
                if (len < bestLen) {
                    bestLen = len;
                    bestPath = path;
                    bestFrom = from;
                    bestTo = to;
                }
            }
        }
        if (bestPath == null || bestFrom == null || bestTo == null) {
            return false;
        }
        if (rewireIsland(variant, obstacles, frame, ids, island, rooted, bestPath, bestFrom, bestTo, bestLen)) {
            return true;
        }
        PipeEmitter.emit(variant, obstacles, ids, bestFrom, bestTo, islandFlow(island), bestPath);
        return true;
    }

    /**
     * Остров не наращиваем мостом поверх длинного внутреннего дерева:
     * MST вводов + стык к уже врезанному дереву, если это короче.
     */
    private static boolean rewireIsland(Variant variant, ObstacleIndex obstacles, StreetFrame frame,
                                        AtomicInteger ids, Comp island, List<Comp> rooted,
                                        List<Coordinate> bridge, String bridgeFrom, String bridgeTo,
                                        double bridgeLen) {
        if (island == null || island.oks.size() < 2 || rooted == null || rooted.isEmpty()) {
            return false;
        }
        List<Hub> hubs = new ArrayList<>();
        Set<NewSegment> stubs = new HashSet<>();
        Set<String> seenOks = new HashSet<>();
        double stubLen = 0;
        double islandLen = 0;
        for (NewSegment s : island.segs) {
            if (s == null) {
                continue;
            }
            islandLen += s.lengthM > 0 ? s.lengthM : 0;
            if (s.fromId == null || !looksLikeOks(s.fromId) || !seenOks.add(s.fromId)) {
                continue;
            }
            stubs.add(s);
            stubLen += s.lengthM > 0 ? s.lengthM : 0;
            Coordinate at = s.geometryMeters == null ? null
                    : s.geometryMeters.getCoordinateN(s.geometryMeters.getNumPoints() - 1);
            String node = s.toId != null ? s.toId : nearestNodeId(variant, at, 8);
            if (at == null || node == null) {
                continue;
            }
            hubs.add(new Hub(s.fromId, node, at, oksFlowOf(island, s.fromId)));
        }
        if (hubs.size() < 2) {
            return false;
        }
        int n = hubs.size();
        int root = n;
        List<MstEdge> edges = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                List<Coordinate> path = shortPath(obstacles, frame, hubs.get(i).at, hubs.get(j).at, BRIDGE_NEAR_M);
                if (path == null || path.size() < 2) {
                    continue;
                }
                double len = OrthoPaths.length(path);
                if (len > 220 || len + 8 < hubs.get(i).at.distance(hubs.get(j).at)) {
                    continue;
                }
                edges.add(new MstEdge(i, j, path, len, hubs.get(i).nodeId, hubs.get(j).nodeId));
            }
            List<Coordinate> toRoot = null;
            double toLen = 221;
            String toId = null;
            String fromId = hubs.get(i).nodeId;
            for (Comp r : rooted) {
                List<Pair> pairs = candidatePairs(hubPoint(island, hubs.get(i)), r, BRIDGE_NEAR_M + 24);
                int tried = 0;
                for (Pair p : pairs) {
                    if (tried++ >= 12) {
                        break;
                    }
                    List<Coordinate> path = shortPath(obstacles, frame, hubs.get(i).at, p.b, BRIDGE_NEAR_M);
                    if (path == null || path.size() < 2) {
                        continue;
                    }
                    String to = nearestNodeId(variant, p.b, 80);
                    if (to == null || to.equals(fromId)) {
                        continue;
                    }
                    double len = OrthoPaths.length(path);
                    if (len > 220 || len + 8 < hubs.get(i).at.distance(p.b)) {
                        continue;
                    }
                    if (len < toLen) {
                        toLen = len;
                        toRoot = path;
                        toId = to;
                    }
                }
            }
            if (toRoot != null && toId != null) {
                edges.add(new MstEdge(i, root, toRoot, toLen, fromId, toId));
            }
        }
        List<MstEdge> mst = kruskalTerms(n + 1, edges);
        boolean[] seen = new boolean[n + 1];
        List<List<MstEdge>> adj = new ArrayList<>();
        for (int i = 0; i <= n; i++) {
            adj.add(new ArrayList<>());
        }
        double mstLen = 0;
        for (MstEdge e : mst) {
            mstLen += e.len;
            adj.get(e.a).add(e);
            adj.get(e.b).add(e);
        }
        ArrayDeque<Integer> q = new ArrayDeque<>();
        q.add(root);
        seen[root] = true;
        int reached = 1;
        while (!q.isEmpty()) {
            int u = q.removeFirst();
            for (MstEdge e : adj.get(u)) {
                int v = e.a == u ? e.b : e.a;
                if (!seen[v]) {
                    seen[v] = true;
                    reached++;
                    q.add(v);
                }
            }
        }
        if (reached < n + 1) {
            return false;
        }
        double keep = islandLen + bridgeLen;
        double neu = stubLen + mstLen;
        if (neu + 1 >= keep) {
            return false;
        }
        variant.segments.removeIf(s -> island.segs.contains(s) && !stubs.contains(s));
        dropUnusedNodes(variant);
        double[] sub = new double[n + 1];
        for (int i = 0; i < n; i++) {
            sub[i] = Math.max(0.01, hubs.get(i).flow);
        }
        int[] parent = new int[n + 1];
        MstEdge[] via = new MstEdge[n + 1];
        Arrays.fill(parent, -1);
        q.add(root);
        parent[root] = root;
        while (!q.isEmpty()) {
            int u = q.removeFirst();
            for (MstEdge e : adj.get(u)) {
                int v = e.a == u ? e.b : e.a;
                if (parent[v] < 0) {
                    parent[v] = u;
                    via[v] = e;
                    q.add(v);
                }
            }
        }
        for (int i = 0; i < n; i++) {
            int cur = i;
            int guard = 0;
            while (cur != root && guard++ <= n) {
                int p = parent[cur];
                if (p < 0) {
                    break;
                }
                if (p != root) {
                    sub[p] += hubs.get(i).flow;
                }
                cur = p;
            }
        }
        for (int i = 0; i < n; i++) {
            MstEdge e = via[i];
            if (e == null || e.path == null || e.path.size() < 2) {
                continue;
            }
            double flow = Math.max(0.01, sub[i]);
            PipeEmitter.emit(variant, obstacles, ids, e.fromId, e.toId, flow, e.path);
        }
        return true;
    }

    private static Comp hubPoint(Comp island, Hub hub) {
        Comp c = new Comp();
        NewSegment s = new NewSegment();
        s.fromId = hub.oksId;
        s.toId = hub.nodeId;
        s.geometryMeters = GeoJsonGeometries.GF.createLineString(new Coordinate[]{
                new Coordinate(hub.at), new Coordinate(hub.at.x + 0.4, hub.at.y)
        });
        c.segs.add(s);
        if (island != null) {
            c.oks.addAll(island.oks);
        }
        return c;
    }

    private static double oksFlowOf(Comp island, String id) {
        if (island == null || id == null) {
            return 0.01;
        }
        for (OksPort p : island.oks) {
            if (p != null && id.equals(p.id())) {
                return Math.max(0.01, p.flow());
            }
        }
        return 0.01;
    }

    private static List<MstEdge> kruskalTerms(int n, List<MstEdge> edges) {
        List<MstEdge> order = new ArrayList<>(edges);
        order.sort(Comparator.comparingDouble(e -> e.len));
        int[] p = new int[n];
        for (int i = 0; i < n; i++) {
            p[i] = i;
        }
        List<MstEdge> mst = new ArrayList<>();
        for (MstEdge e : order) {
            int a = find(p, e.a);
            int b = find(p, e.b);
            if (a == b) {
                continue;
            }
            p[b] = a;
            mst.add(e);
        }
        return mst;
    }

    private static void dropUnusedNodes(Variant variant) {
        if (variant == null) {
            return;
        }
        Set<String> used = new HashSet<>();
        for (NewSegment s : variant.segments) {
            if (s.fromId != null) {
                used.add(s.fromId);
            }
            if (s.toId != null) {
                used.add(s.toId);
            }
        }
        variant.technicalNodes.removeIf(n -> n == null || n.id == null || !used.contains(n.id));
    }

    private static final class Hub {
        final String oksId;
        final String nodeId;
        final Coordinate at;
        final double flow;

        Hub(String oksId, String nodeId, Coordinate at, double flow) {
            this.oksId = oksId;
            this.nodeId = nodeId;
            this.at = at;
            this.flow = flow;
        }
    }

    private static final class MstEdge {
        final int a;
        final int b;
        final List<Coordinate> path;
        final double len;
        final String fromId;
        final String toId;

        MstEdge(int a, int b, List<Coordinate> path, double len, String fromId, String toId) {
            this.a = a;
            this.b = b;
            this.path = path;
            this.len = len;
            this.fromId = fromId;
            this.toId = toId;
        }
    }

    private static boolean bridgeToKnownTap(Variant variant, ObstacleIndex obstacles, StreetFrame frame,
                                            AtomicInteger ids, Comp island, double capM) {
        TapPoint bestTap = null;
        Coordinate from = null;
        String fromId = null;
        double bestD = Double.POSITIVE_INFINITY;
        for (TapPoint t : variant.taps) {
            if (t == null || t.geometryMeters == null) {
                continue;
            }
            Coordinate at = t.geometryMeters.getCoordinate();
            for (NewSegment s : island.segs) {
                if (s.geometryMeters == null) {
                    continue;
                }
                Coordinate[] pts = s.geometryMeters.getCoordinates();
                if (pts[0].distance(at) < bestD && s.fromId != null) {
                    bestD = pts[0].distance(at);
                    from = pts[0];
                    fromId = s.fromId;
                    bestTap = t;
                }
                if (pts[pts.length - 1].distance(at) < bestD && s.toId != null) {
                    bestD = pts[pts.length - 1].distance(at);
                    from = pts[pts.length - 1];
                    fromId = s.toId;
                    bestTap = t;
                }
            }
        }
        if (bestTap == null || from == null || fromId == null || bestD > capM + 8) {
            return false;
        }
        Coordinate at = bestTap.geometryMeters.getCoordinate();
        String toId = bestTap.nodeId != null ? bestTap.nodeId : bestTap.id;
        if (toId == null || fromId.equals(toId)) {
            return false;
        }
        List<Coordinate> path = shortPath(obstacles, frame, from, at, capM);
        if (path == null || path.size() < 2) {
            return false;
        }
        PipeEmitter.emit(variant, obstacles, ids, fromId, toId, islandFlow(island), path);
        return true;
    }

    private static boolean bridgeNewTap(Variant variant, Scene scene, ObstacleIndex obstacles,
                                        StreetFrame frame, AtomicInteger ids, Comp island, double capM) {
        Coordinate exist = nearestExisting(scene, island);
        String existId = existingIdAt(scene, exist);
        Coordinate from = island.sample();
        if (from == null && !island.oks.isEmpty()) {
            from = island.oks.get(0).origin;
        }
        if (exist == null || existId == null || from == null || from.distance(exist) > capM + 8) {
            return false;
        }
        String fromId = nearestNodeId(variant, from);
        if (fromId == null && !island.oks.isEmpty()) {
            fromId = island.oks.get(0).id();
        }
        if (fromId == null || fromId.equals(existId)) {
            return false;
        }
        List<Coordinate> path = shortPath(obstacles, frame, from, exist, capM);
        if (path == null || path.size() < 2) {
            return false;
        }
        addTap(variant, ids, exist, existId);
        PipeEmitter.emit(variant, obstacles, ids, fromId, existId, islandFlow(island), path);
        return true;
    }

    private static List<Coordinate> shortPath(ObstacleIndex obstacles, StreetFrame frame,
                                              Coordinate from, Coordinate to, double capM) {
        if (from == null || to == null) {
            return null;
        }
        double d = from.distance(to);
        if (d < 0.35) {
            List<Coordinate> tiny = new ArrayList<>();
            tiny.add(new Coordinate(from));
            tiny.add(new Coordinate(to.x + 0.4, to.y));
            return tiny;
        }
        List<Coordinate> best = null;
        boolean interiorHit = obstacles != null && obstacles.segmentHitsAvoid(from, to, 0, true);
        boolean grazeHit = obstacles != null && obstacles.segmentHitsAvoid(from, to, 0, false);
        boolean along = obstacles != null && obstacles.alongAvoid(from, to, OrthoPaths.FACADE_M);
        boolean axis = OrthoPaths.nearlyAxis(from, to);
        if (d <= capM && !interiorHit && (!grazeHit || along || axis || d <= 36)) {
            List<Coordinate> direct = new ArrayList<>();
            direct.add(new Coordinate(from));
            direct.add(new Coordinate(to));
            best = shorter(best, direct, 220);
        }
        if (obstacles != null) {
            best = shorter(best, obstacles.hugAround(from, to, false), 220);
            best = shorter(best, obstacles.hugAround(from, to), 220);
            best = shorter(best, OrthoPaths.streetElbow(obstacles, from, to), 220);
            best = shorter(best, OrthoPaths.usefulElbow(obstacles, from, to), 220);
            best = shorter(best, OrthoPaths.bestElbow(obstacles, from, to), 220);
        }
        if (frame != null && d <= capM + 24) {
            Coordinate sa = frame.attach(from);
            Coordinate sb = frame.attach(to);
            best = shorter(best, frame.find(sa != null ? sa : from, sb != null ? sb : to), 220);
        }
        if (best != null && obstacles != null) {
            List<Coordinate> polished = PathSmoother.emitPolish(best, obstacles);
            if (polished != null && polished.size() >= 2
                    && OrthoPaths.length(polished) <= Math.min(220, OrthoPaths.length(best) + 1)) {
                best = polished;
            }
        }
        return best;
    }

    private static List<Coordinate> shorter(List<Coordinate> best, List<Coordinate> cand, double cap) {
        if (cand == null || cand.size() < 2) {
            return best;
        }
        double len = OrthoPaths.length(cand);
        if (len > cap + 1e-6) {
            return best;
        }
        if (best == null || len + 0.4 < OrthoPaths.length(best)) {
            return cand;
        }
        return best;
    }

    private static void addTap(Variant variant, AtomicInteger ids, Coordinate at, String existingId) {
        TapPoint t = new TapPoint();
        t.id = "TI-" + ids.getAndIncrement();
        t.nodeId = existingId;
        t.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(at));
        t.existingObjectId = existingId;
        t.existingObjectKind = existingId != null && existingId.startsWith("S") ? "heat_network" : "heat_chamber";
        variant.taps.add(t);
    }

    private static Map<String, Integer> nodeComponents(Variant variant) {
        Map<String, List<String>> adj = new HashMap<>();
        Set<String> nodes = new HashSet<>();
        for (NewSegment s : variant.segments) {
            if (s.fromId == null || s.toId == null) {
                continue;
            }
            nodes.add(s.fromId);
            nodes.add(s.toId);
            adj.computeIfAbsent(s.fromId, k -> new ArrayList<>()).add(s.toId);
            adj.computeIfAbsent(s.toId, k -> new ArrayList<>()).add(s.fromId);
        }
        Map<String, Integer> comp = new HashMap<>();
        int n = 0;
        for (String start : nodes) {
            if (comp.containsKey(start)) {
                continue;
            }
            ArrayDeque<String> q = new ArrayDeque<>();
            q.add(start);
            comp.put(start, n);
            while (!q.isEmpty()) {
                String u = q.removeFirst();
                for (String v : adj.getOrDefault(u, List.of())) {
                    if (comp.putIfAbsent(v, n) == null) {
                        q.add(v);
                    }
                }
            }
            n++;
        }
        return comp;
    }

    private static Map<Integer, Comp> groupComps(Variant variant, Map<String, Integer> compOf,
                                                 Map<String, OksPort> byId) {
        Map<Integer, Comp> out = new HashMap<>();
        for (NewSegment s : variant.segments) {
            Integer c = s.fromId == null ? null : compOf.get(s.fromId);
            if (c == null) {
                continue;
            }
            Comp g = out.computeIfAbsent(c, k -> new Comp());
            g.segs.add(s);
            if (s.fromId != null && byId.containsKey(s.fromId)) {
                OksPort p = byId.get(s.fromId);
                if (!g.oks.contains(p)) {
                    g.oks.add(p);
                }
            } else if (looksLikeOks(s.fromId) && !hasOks(g, s.fromId)) {
                g.oks.add(syntheticPort(s.fromId, s.flowTph, s.geometryMeters == null ? null
                        : s.geometryMeters.getCoordinateN(0)));
            }
            if (s.toId != null && byId.containsKey(s.toId)) {
                OksPort p = byId.get(s.toId);
                if (!g.oks.contains(p)) {
                    g.oks.add(p);
                }
            }
        }
        for (TapPoint t : variant.taps) {
            if (t == null) {
                continue;
            }
            Integer c = t.nodeId != null ? compOf.get(t.nodeId) : null;
            if (c == null && t.id != null) {
                c = compOf.get(t.id);
            }
            if (c != null) {
                out.computeIfAbsent(c, k -> new Comp()).tap = t;
            }
        }
        return out;
    }

    private static Coordinate nearestExisting(Scene scene, Comp c) {
        Coordinate from = c.sample();
        if (from == null && !c.oks.isEmpty() && c.oks.get(0).origin != null) {
            from = c.oks.get(0).origin;
        }
        return nearestExisting(scene, from);
    }

    private static Coordinate nearestExisting(Scene scene, Coordinate from) {
        if (from == null || scene == null) {
            return null;
        }
        Coordinate best = null;
        double bestD = Double.POSITIVE_INFINITY;
        for (Chamber ch : scene.chambers) {
            if (ch.point == null) {
                continue;
            }
            double d = from.distance(ch.point.getCoordinate());
            if (d < bestD) {
                bestD = d;
                best = ch.point.getCoordinate();
            }
        }
        for (ExistingSegment seg : scene.segments) {
            if (seg.line == null) {
                continue;
            }
            DistanceOp op = new DistanceOp(seg.line, GeoJsonGeometries.GF.createPoint(from));
            Coordinate[] pts = op.nearestPoints();
            double d = from.distance(pts[0]);
            if (d < bestD) {
                bestD = d;
                best = pts[0];
            }
        }
        return best;
    }

    private static String existingIdAt(Scene scene, Coordinate at) {
        if (at == null || scene == null) {
            return null;
        }
        String best = null;
        double bestD = 3.5;
        for (Chamber ch : scene.chambers) {
            if (ch.point == null || ch.id == null) {
                continue;
            }
            double d = at.distance(ch.point.getCoordinate());
            if (d < bestD) {
                bestD = d;
                best = ch.id;
            }
        }
        for (ExistingSegment seg : scene.segments) {
            if (seg.line == null || seg.id == null) {
                continue;
            }
            DistanceOp op = new DistanceOp(seg.line, GeoJsonGeometries.GF.createPoint(at));
            double d = at.distance(op.nearestPoints()[0]);
            if (d < bestD) {
                bestD = d;
                best = seg.id;
            }
        }
        return best;
    }

    private static String nearestNodeId(Variant variant, Coordinate at) {
        return nearestNodeId(variant, at, 8);
    }

    private static String nearestNodeId(Variant variant, Coordinate at, double cap) {
        String best = null;
        double bestD = cap;
        if (at == null) {
            return null;
        }
        for (NewSegment s : variant.segments) {
            if (s.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            if (pts[0].distance(at) < bestD && s.fromId != null) {
                bestD = pts[0].distance(at);
                best = s.fromId;
            }
            if (pts[pts.length - 1].distance(at) < bestD && s.toId != null) {
                bestD = pts[pts.length - 1].distance(at);
                best = s.toId;
            }
        }
        for (TapPoint t : variant.taps) {
            if (t.geometryMeters == null) {
                continue;
            }
            double d = t.geometryMeters.getCoordinate().distance(at);
            if (d < bestD) {
                bestD = d;
                best = t.nodeId != null ? t.nodeId : t.id;
            }
        }
        return best;
    }

    private static List<Pair> candidatePairs(Comp a, Comp b, double cap) {
        List<Pair> out = new ArrayList<>();
        if (a == null || b == null) {
            return out;
        }
        for (NewSegment s : a.segs) {
            List<Coordinate> pa = samples(s, 8);
            if (pa.isEmpty()) {
                continue;
            }
            for (NewSegment t : b.segs) {
                List<Coordinate> pb = samples(t, 8);
                if (pb.isEmpty()) {
                    continue;
                }
                for (Coordinate u : pa) {
                    for (Coordinate v : pb) {
                        double d = u.distance(v);
                        if (d <= cap) {
                            out.add(new Pair(u, v, d));
                        }
                    }
                }
            }
        }
        out.sort(Comparator.comparingDouble(p -> p.dist));
        if (out.size() > 32) {
            return new ArrayList<>(out.subList(0, 32));
        }
        return out;
    }

    private static Pair nearestPair(Comp a, Comp b) {
        Pair best = null;
        for (NewSegment s : a.segs) {
            List<Coordinate> pa = samples(s, 4);
            if (pa.isEmpty()) {
                continue;
            }
            for (NewSegment t : b.segs) {
                List<Coordinate> pb = samples(t, 4);
                if (pb.isEmpty()) {
                    continue;
                }
                for (Coordinate u : pa) {
                    for (Coordinate v : pb) {
                        double d = u.distance(v);
                        if (best == null || d < best.dist) {
                            best = new Pair(u, v, d);
                        }
                    }
                }
            }
        }
        return best;
    }

    private static List<Coordinate> samples(NewSegment s, double step) {
        List<Coordinate> out = new ArrayList<>();
        if (s == null || s.geometryMeters == null) {
            return out;
        }
        Coordinate[] pts = s.geometryMeters.getCoordinates();
        for (int i = 0; i < pts.length - 1; i++) {
            out.add(pts[i]);
            double d = pts[i].distance(pts[i + 1]);
            int n = Math.max(1, (int) Math.floor(d / step));
            for (int k = 1; k < n; k++) {
                double t = k / (double) n;
                out.add(new Coordinate(
                        pts[i].x + t * (pts[i + 1].x - pts[i].x),
                        pts[i].y + t * (pts[i + 1].y - pts[i].y)));
            }
        }
        if (pts.length > 0) {
            out.add(pts[pts.length - 1]);
        }
        return out;
    }

    private static final class Comp {
        final List<NewSegment> segs = new ArrayList<>();
        final List<OksPort> oks = new ArrayList<>();
        TapPoint tap;

        Coordinate sample() {
            for (NewSegment s : segs) {
                if (s.geometryMeters != null) {
                    return s.geometryMeters.getCoordinateN(s.geometryMeters.getNumPoints() - 1);
                }
            }
            return null;
        }
    }

    private static final class Pair {
        final Coordinate a;
        final Coordinate b;
        final double dist;

        Pair(Coordinate a, Coordinate b, double dist) {
            this.a = a;
            this.b = b;
            this.dist = dist;
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

    private static void graftOrphans(Variant variant, List<NewSegment> savedSegs, List<TapPoint> savedTaps,
                                     Set<String> oksIds) {
        if (variant == null || savedSegs == null || oksIds == null || oksIds.isEmpty()) {
            return;
        }
        Set<String> tapNow = tapIds(variant);
        Set<String> rooted = nodesReaching(variant, tapNow);
        Map<String, List<NewSegment>> incident = incidentMap(savedSegs);
        for (String oks : oksIds) {
            if (rooted.contains(oks)) {
                continue;
            }
            Map<String, String> parent = new HashMap<>();
            Map<String, NewSegment> via = new HashMap<>();
            Set<String> seen = new HashSet<>();
            ArrayDeque<String> q = new ArrayDeque<>();
            q.add(oks);
            seen.add(oks);
            String hit = null;
            boolean hitSavedTap = false;
            while (!q.isEmpty()) {
                String u = q.removeFirst();
                boolean savedTap = isTapId(savedTaps, u);
                if (!u.equals(oks) && (rooted.contains(u) || tapNow.contains(u) || savedTap)) {
                    hit = u;
                    hitSavedTap = savedTap && !rooted.contains(u) && !tapNow.contains(u);
                    break;
                }
                for (NewSegment s : incident.getOrDefault(u, List.of())) {
                    String v = u.equals(s.fromId) ? s.toId : s.fromId;
                    if (v != null && seen.add(v)) {
                        parent.put(v, u);
                        via.put(v, s);
                        q.add(v);
                    }
                }
            }
            if (hit == null) {
                continue;
            }
            if (hitSavedTap && variant.taps.size() >= 2) {
                continue;
            }
            Set<String> used = new HashSet<>();
            String cur = hit;
            while (parent.containsKey(cur)) {
                NewSegment s = via.get(cur);
                if (s != null && !hasEdge(variant, s.fromId, s.toId)) {
                    variant.segments.add(s);
                }
                if (s != null) {
                    if (s.fromId != null) {
                        used.add(s.fromId);
                    }
                    if (s.toId != null) {
                        used.add(s.toId);
                    }
                }
                cur = parent.get(cur);
            }
            if (!hitSavedTap || variant.taps.size() < 2) {
                addTapIfMissing(variant, savedTaps, used);
            }
            tapNow = tapIds(variant);
            rooted = nodesReaching(variant, tapNow);
        }
    }

    private static Set<String> tapIds(Variant variant) {
        Set<String> ids = new HashSet<>();
        if (variant == null) {
            return ids;
        }
        for (TapPoint t : variant.taps) {
            if (t == null) {
                continue;
            }
            if (t.id != null) {
                ids.add(t.id);
            }
            if (t.nodeId != null) {
                ids.add(t.nodeId);
            }
            if (t.existingObjectId != null) {
                ids.add(t.existingObjectId);
            }
        }
        return ids;
    }

    private static Set<String> nodesReaching(Variant variant, Set<String> taps) {
        Map<String, List<String>> adj = new HashMap<>();
        for (NewSegment s : variant.segments) {
            if (s.fromId == null || s.toId == null) {
                continue;
            }
            adj.computeIfAbsent(s.fromId, k -> new ArrayList<>()).add(s.toId);
            adj.computeIfAbsent(s.toId, k -> new ArrayList<>()).add(s.fromId);
        }
        Set<String> seen = new HashSet<>();
        ArrayDeque<String> q = new ArrayDeque<>();
        for (String t : taps) {
            if (t != null && seen.add(t)) {
                q.add(t);
            }
        }
        while (!q.isEmpty()) {
            String u = q.removeFirst();
            for (String v : adj.getOrDefault(u, List.of())) {
                if (seen.add(v)) {
                    q.add(v);
                }
            }
        }
        return seen;
    }

    private static boolean isTapId(List<TapPoint> taps, String id) {
        if (id == null || taps == null) {
            return false;
        }
        for (TapPoint t : taps) {
            if (t != null && (id.equals(t.id) || id.equals(t.nodeId) || id.equals(t.existingObjectId))) {
                return true;
            }
        }
        return false;
    }

    private static boolean hasEdge(Variant variant, String a, String b) {
        if (a == null || b == null) {
            return false;
        }
        for (NewSegment s : variant.segments) {
            if ((a.equals(s.fromId) && b.equals(s.toId)) || (b.equals(s.fromId) && a.equals(s.toId))) {
                return true;
            }
        }
        return false;
    }

    private static Map<String, List<NewSegment>> incidentMap(List<NewSegment> segs) {
        Map<String, List<NewSegment>> out = new HashMap<>();
        for (NewSegment s : segs) {
            if (s.fromId != null) {
                out.computeIfAbsent(s.fromId, k -> new ArrayList<>()).add(s);
            }
            if (s.toId != null) {
                out.computeIfAbsent(s.toId, k -> new ArrayList<>()).add(s);
            }
        }
        return out;
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
                    boolean oksStub = oksAt.get(i) != null || oksAt.get(j) != null;
                    boolean along = obstacles != null && obstacles.alongAvoid(a, b, 8);
                    boolean street = false;
                    if (obstacles != null && obstacles.special() != null) {
                        ru.lct.heatnet.engine.greedy.SpecialLayer.Corridor c =
                                obstacles.special().nearestCorridor(
                                        new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5), 28);
                        if (c != null && c.axis != null) {
                            double ang = ru.lct.heatnet.engine.greedy.SpecialLayer.crossingAngleDeg(a, b, c.axis);
                            street = ang <= 16 || ang >= 74;
                        }
                    }
                    double cap = along || street ? 96 : (oksStub ? 36 : 22);
                    if (d < 0.8 || d > cap) {
                        continue;
                    }
                    if (obstacles != null && obstacles.segmentHitsAvoid(a, b, 0, true)) {
                        if (!oksStub) {
                            continue;
                        }
                        Coordinate from = oksAt.get(i) != null ? a : b;
                        Coordinate to = oksAt.get(i) != null ? b : a;
                        if (!ItpSnapper.stubLegal(obstacles, from, to)) {
                            continue;
                        }
                    }
                    if (obstacles != null && obstacles.special() != null) {
                        ru.lct.heatnet.engine.greedy.SpecialLayer.Corridor c =
                                obstacles.special().nearestCorridor(
                                        new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5), 24);
                        if (c != null && c.axis != null) {
                            double ang = ru.lct.heatnet.engine.greedy.SpecialLayer.crossingAngleDeg(a, b, c.axis);
                            if (ang > 16 && ang < 74 && d > 8) {
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
            leafifyOks(adj);
            boolean[] term = new boolean[coords.size()];
            oksNodes.clear();
            markedOks.clear();
            for (int i = 0; i < coords.size(); i++) {
                if (oksAt.get(i) != null) {
                    term[i] = true;
                    oksNodes.add(i);
                    markedOks.add(oksAt.get(i));
                }
                if (tapAt.get(i) != null) {
                    term[i] = true;
                }
            }
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
                        term[t] = false;
                    }
                }
            }
            if (roots.isEmpty()) {
                return null;
            }
            prune(adj, term);
            boolean[] keep = new boolean[adj.size()];
            for (int r : roots) {
                keep[r] = true;
            }
            for (int o : reachableOks) {
                keep[o] = true;
            }
            for (int i = 0; i < adj.size(); i++) {
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

        private void leafifyOks(List<List<int[]>> adj) {
            int n = adj.size();
            for (int i = 0; i < n; i++) {
                if (oksAt.get(i) == null || adj.get(i).size() <= 1) {
                    continue;
                }
                int extra = coords.size();
                Coordinate base = coords.get(i);
                coords.add(new Coordinate(base.x + 0.45, base.y));
                String id = oksAt.get(i);
                oksAt.set(i, null);
                oksAt.add(id);
                oksIndex.put(id, extra);
                tapAt.add(null);
                int[] e = new int[]{i, extra};
                adj.add(new ArrayList<>());
                adj.get(extra).add(e);
                adj.get(i).add(e);
            }
        }

        private int addNode(Coordinate c) {
            int near = nearest(c, SNAP_M);
            if (near >= 0 && (oksAt.get(near) == null || coords.get(near).distance(c) <= 0.55)) {
                return near;
            }
            String k = Math.round(c.x / 3.0) + ":" + Math.round(c.y / 3.0);
            Integer existing = index.get(k);
            if (existing != null && (oksAt.get(existing) == null || coords.get(existing).distance(c) <= 0.55)) {
                return existing;
            }
            int id = coords.size();
            coords.add(new Coordinate(c));
            oksAt.add(null);
            tapAt.add(null);
            if (existing == null) {
                index.put(k, id);
            }
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
                if ((i < terminal.length && terminal[i]) || adj.get(i).size() != 1) {
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
