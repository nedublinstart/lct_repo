package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.OrthoPaths;
import ru.lct.heatnet.engine.greedy.PipeEmitter;
import ru.lct.heatnet.engine.greedy.SpecialLayer;
import ru.lct.heatnet.engine.greedy.StreetFrame;
import ru.lct.heatnet.geo.GeoJsonGeometries;

/**
 * Ввод ИТП — прямой перпендикуляр к уже построенному дереву, не объезд угла дома.
 */
public final class ItpSnapper {

    private static final double SNAP_REACH_M = 96.0;
    private static final double TAP_SNAP_M = 160.0;
    private static final double GRAFT_REACH_M = 220.0;

    private ItpSnapper() {
    }

    public static void snap(Variant variant, ObstacleIndex obstacles, AtomicInteger ids,
                            List<OksPort> ports) {
        snap(variant, obstacles, ids, ports, null);
    }

    public static void snap(Variant variant, ObstacleIndex obstacles, AtomicInteger ids,
                            List<OksPort> ports, StreetFrame frame) {
        if (variant == null || variant.segments.isEmpty() || ports == null || ports.isEmpty()) {
            return;
        }
        for (OksPort p : ports) {
            snapOne(variant, obstacles, ids, p, frame);
        }
        graftMissing(variant, obstacles, ids, ports, frame);
    }

    static void graftMissing(Variant variant, ObstacleIndex obstacles, AtomicInteger ids,
                             List<OksPort> ports, StreetFrame frame) {
        if (variant == null || variant.segments.isEmpty() || ports == null) {
            return;
        }
        Set<String> have = new HashSet<>();
        for (NewSegment s : variant.segments) {
            if (s.fromId != null) {
                have.add(s.fromId);
            }
        }
        for (OksPort p : ports) {
            if (p == null || p.origin == null || have.contains(p.id())) {
                continue;
            }
            Coordinate origin = new Coordinate(p.origin);
            Hit towardHit = nearestGeom(variant, origin, GRAFT_REACH_M);
            Coordinate from = obstacles.exitToStreet(origin, towardHit == null ? null : towardHit.at, 1.2);
            if (from == null) {
                from = origin;
            }
            Hit best = nearestTree(variant, obstacles, origin, List.of(), SNAP_REACH_M);
            if (best == null) {
                best = nearestTree(variant, obstacles, origin, List.of(), GRAFT_REACH_M);
            }
            if (best == null && frame != null) {
                best = nearestTree(variant, obstacles, from, List.of(), GRAFT_REACH_M);
            }
            List<Coordinate> via = null;
            if (best != null && stubReachable(obstacles, origin, best.at)) {
                via = stubPath(obstacles, origin, best.at);
            }
            if (via == null && best != null && frame != null) {
                Coordinate snap = frame.attach(best.at);
                via = frame.find(from, snap != null ? snap : best.at);
            }
            if (via == null && frame != null) {
                via = pathMeetTree(variant, frame, from);
            }
            if (via == null && frame != null) {
                via = pathToConnectedPort(frame, from, ports, have);
            }
            if (via == null && frame != null) {
                via = nearestFramePath(variant, frame, from);
            }
            if (via == null && best != null) {
                List<Coordinate> hug = obstacles.hugAround(from, best.at);
                if (hug != null && hug.size() >= 2) {
                    via = hug;
                }
            }
            if (via == null || via.size() < 2) {
                continue;
            }
            Coordinate joinAt = via.get(via.size() - 1);
            Hit attach = new Hit();
            attach.at = new Coordinate(joinAt);
            attach.dist = origin.distance(joinAt);
            attach.seg = segmentAt(variant, joinAt);
            attach.nodeId = idAt(variant, joinAt);
            String node = ensureNode(variant, obstacles, ids, attach);
            if (node == null || node.equals(p.id())) {
                continue;
            }
            List<Coordinate> path = join(obstacles, origin, via);
            PipeEmitter.emit(variant, obstacles, ids, p.id(), node, Math.max(0.01, p.flow()), path);
            have.add(p.id());
        }
    }

    private static List<Coordinate> pathToConnectedPort(StreetFrame frame, Coordinate from,
                                                        List<OksPort> ports, Set<String> have) {
        if (frame == null || from == null || ports == null) {
            return null;
        }
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        for (OksPort q : ports) {
            if (q == null || q.at == null || !have.contains(q.id())) {
                continue;
            }
            List<Coordinate> path = frame.find(from, q.at);
            if (path == null || path.size() < 2) {
                continue;
            }
            double len = OrthoPaths.length(path);
            if (len < bestLen) {
                bestLen = len;
                best = path;
            }
        }
        return best;
    }

    private static List<Coordinate> nearestFramePath(Variant variant, StreetFrame frame, Coordinate from) {
        if (from == null || frame == null) {
            return null;
        }
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        for (NewSegment s : variant.segments) {
            if (s.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            List<Coordinate> samples = new ArrayList<>();
            samples.add(pts[0]);
            samples.add(pts[pts.length - 1]);
            for (int i = 0; i < pts.length - 1; i++) {
                double span = pts[i].distance(pts[i + 1]);
                int n = Math.max(1, (int) Math.floor(span / 24.0));
                for (int k = 1; k < n; k++) {
                    double t = k / (double) n;
                    samples.add(new Coordinate(
                            pts[i].x + t * (pts[i + 1].x - pts[i].x),
                            pts[i].y + t * (pts[i + 1].y - pts[i].y)));
                }
            }
            for (Coordinate t : samples) {
                Coordinate goal = t;
                if (frame != null) {
                    Coordinate snap = frame.attach(t);
                    if (snap != null) {
                        goal = snap;
                    }
                }
                List<Coordinate> path = frame.find(from, goal);
                if (path == null || path.size() < 2) {
                    continue;
                }
                double len = OrthoPaths.length(path);
                if (len < bestLen) {
                    bestLen = len;
                    best = path;
                }
            }
        }
        return best;
    }

    private static List<Coordinate> pathMeetTree(Variant variant, StreetFrame frame, Coordinate from) {
        if (variant == null || variant.taps.isEmpty() || frame == null || from == null) {
            return null;
        }
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        for (TapPoint t : variant.taps) {
            if (t == null || t.geometryMeters == null) {
                continue;
            }
            List<Coordinate> toTap = frame.find(from, t.geometryMeters.getCoordinate());
            if (toTap == null || toTap.size() < 2) {
                continue;
            }
            Hit meet = nearestGeomOnPath(variant, toTap, 18);
            if (meet == null) {
                continue;
            }
            List<Coordinate> cut = new ArrayList<>();
            for (Coordinate c : toTap) {
                cut.add(new Coordinate(c));
                if (c.distance(meet.at) <= 1.2) {
                    break;
                }
            }
            if (cut.get(cut.size() - 1).distance(meet.at) > 0.45) {
                cut.add(new Coordinate(meet.at));
            }
            double len = OrthoPaths.length(cut);
            if (len < bestLen) {
                bestLen = len;
                best = cut;
            }
        }
        return best;
    }

    private static Hit nearestGeom(Variant variant, Coordinate origin, double reach) {
        if (origin == null || variant == null) {
            return null;
        }
        Hit best = null;
        for (NewSegment s : variant.segments) {
            if (s.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            for (int i = 0; i < pts.length; i++) {
                double d = origin.distance(pts[i]);
                if (d < 0.4 || d > reach) {
                    continue;
                }
                if (best == null || d < best.dist) {
                    Hit h = new Hit();
                    h.at = new Coordinate(pts[i]);
                    h.dist = d;
                    h.score = d;
                    h.seg = s;
                    h.nodeId = i == 0 ? s.fromId : (i == pts.length - 1 ? s.toId : null);
                    best = h;
                }
            }
        }
        return best;
    }

    private static Hit nearestGeomOnPath(Variant variant, List<Coordinate> path, double reach) {
        Hit best = null;
        if (path == null) {
            return null;
        }
        for (Coordinate p : path) {
            Hit hit = nearestGeom(variant, p, reach);
            if (hit == null) {
                continue;
            }
            if (best == null || hit.dist < best.dist) {
                best = hit;
            }
        }
        return best;
    }

    private static List<Coordinate> stubPath(ObstacleIndex obstacles, Coordinate origin, Coordinate at) {
        List<Coordinate> path = new ArrayList<>();
        path.add(new Coordinate(origin));
        appendStub(path, obstacles, origin, at);
        if (path.get(path.size() - 1).distance(at) > 0.45) {
            path.add(new Coordinate(at));
        }
        return path;
    }

    private static String idAt(Variant variant, Coordinate at) {
        if (at == null) {
            return null;
        }
        for (NewSegment s : variant.segments) {
            if (s.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            if (pts[0].distance(at) <= 1.2 && s.fromId != null) {
                return s.fromId;
            }
            if (pts[pts.length - 1].distance(at) <= 1.2 && s.toId != null) {
                return s.toId;
            }
        }
        for (TechnicalNode n : variant.technicalNodes) {
            if (n.geometryMeters != null && n.geometryMeters.getCoordinate().distance(at) <= 1.2) {
                return n.id;
            }
        }
        for (TapPoint t : variant.taps) {
            if (t.geometryMeters != null && t.geometryMeters.getCoordinate().distance(at) <= 1.2) {
                return t.nodeId != null ? t.nodeId : t.id;
            }
        }
        return null;
    }

    private static NewSegment segmentAt(Variant variant, Coordinate at) {
        NewSegment best = null;
        double bestD = 4;
        for (NewSegment s : variant.segments) {
            if (s.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            for (Coordinate p : pts) {
                double d = at.distance(p);
                if (d < bestD) {
                    bestD = d;
                    best = s;
                    if (d < 0.5) {
                        return s;
                    }
                }
            }
        }
        return best;
    }

    /**
     * Стык ИТП с уже найденным уличным путём: к ближайшей точке на пути,
     * без обхода угла здания.
     */
    public static List<Coordinate> join(ObstacleIndex obstacles, Coordinate origin, List<Coordinate> path) {
        Cut cut = cutStub(obstacles, origin, path);
        List<Coordinate> out = new ArrayList<>(cut.stub);
        for (int i = 1; i < cut.rest.size(); i++) {
            Coordinate q = cut.rest.get(i);
            if (q == null || out.get(out.size() - 1).distance(q) < 0.45) {
                continue;
            }
            out.add(new Coordinate(q));
        }
        if (out.size() < 2 && path != null && !path.isEmpty()) {
            out.add(new Coordinate(path.get(path.size() - 1)));
        }
        return out;
    }

    /**
     * Ввод ИТП отдельно от ствола: лист с собственным расходом, не сквозная нитка.
     */
    public static Cut cutStub(ObstacleIndex obstacles, Coordinate origin, List<Coordinate> path) {
        List<Coordinate> stub = new ArrayList<>();
        if (origin == null) {
            List<Coordinate> rest = copy(path);
            Coordinate joinAt = rest.isEmpty() ? null : rest.get(0);
            return new Cut(rest, List.of(), joinAt);
        }
        stub.add(new Coordinate(origin));
        if (path == null || path.isEmpty()) {
            return new Cut(stub, List.of(), origin);
        }
        Hit best = nearestOnPath(obstacles, origin, path, SNAP_REACH_M);
        Coordinate joinAt = best == null ? path.get(0) : best.at;
        int fromIdx = best == null ? 0 : Math.max(0, best.pathIndex);
        if (origin.distance(joinAt) > 0.45) {
            appendStub(stub, obstacles, origin, joinAt);
        }
        if (stub.get(stub.size() - 1).distance(joinAt) > 0.45) {
            stub.add(new Coordinate(joinAt));
        }
        List<Coordinate> rest = new ArrayList<>();
        rest.add(new Coordinate(joinAt));
        for (int i = fromIdx; i < path.size(); i++) {
            Coordinate q = path.get(i);
            if (q == null || rest.get(rest.size() - 1).distance(q) < 0.45) {
                continue;
            }
            rest.add(new Coordinate(q));
        }
        if (rest.size() < 2) {
            rest = List.of();
        }
        if (stub.size() < 2 && !path.isEmpty()) {
            stub.add(new Coordinate(joinAt));
        }
        return new Cut(stub, rest, joinAt);
    }

    private static void snapOne(Variant variant, ObstacleIndex obstacles, AtomicInteger ids,
                                OksPort port, StreetFrame frame) {
        if (port == null || port.origin == null) {
            return;
        }
        List<NewSegment> spur = exclusiveSpur(variant, port.id());
        if (spur.isEmpty()) {
            return;
        }
        Coordinate origin = new Coordinate(port.origin);
        double old = 0;
        for (NewSegment s : spur) {
            old += s.lengthM;
        }
        Coordinate now = spur.get(0).geometryMeters.getCoordinateN(
                spur.get(0).geometryMeters.getNumPoints() - 1);
        Peel peel = bestPeel(variant, obstacles, frame, origin, spur, old);
        if (peel == null || peel.path == null || peel.path.size() < 2) {
            return;
        }
        boolean oldOrtho = ortho(obstacles, origin, now);
        if (oldOrtho && old <= peel.len + 2.5) {
            return;
        }
        if (peel.len + 0.8 >= old && oldOrtho) {
            return;
        }
        if (oldOrtho && peel.len > old * 0.85 && peel.len + 4 >= old) {
            return;
        }
        String node = peel.nodeId;
        if (node == null && peel.hit != null) {
            node = ensureNode(variant, obstacles, ids, peel.hit);
        }
        if (node == null || node.equals(port.id())) {
            return;
        }
        variant.segments.removeAll(spur);
        PipeEmitter.emit(variant, obstacles, ids, port.id(), node, Math.max(0.01, port.flow()), peel.path);
    }

    private static Peel bestPeel(Variant variant, ObstacleIndex obstacles, StreetFrame frame,
                                 Coordinate origin, List<NewSegment> spur, double old) {
        Peel best = null;
        Hit tree = nearestTree(variant, obstacles, origin, spur, SNAP_REACH_M);
        if (tree == null) {
            tree = nearestTree(variant, obstacles, origin, spur, GRAFT_REACH_M);
        }
        if (tree != null) {
            List<Coordinate> path = buildStub(obstacles, frame, origin, tree.at);
            best = Peel.of(tree.nodeId, tree, path);
        }
        for (TapPoint t : variant.taps) {
            if (t == null || t.geometryMeters == null) {
                continue;
            }
            Coordinate at = t.geometryMeters.getCoordinate();
            double d = origin.distance(at);
            if (d < 0.6 || d > TAP_SNAP_M) {
                continue;
            }
            List<Coordinate> path = buildStub(obstacles, frame, origin, at);
            if (path == null || path.size() < 2) {
                continue;
            }
            String nid = t.nodeId != null ? t.nodeId : t.id;
            Hit hit = new Hit();
            hit.at = new Coordinate(at);
            hit.dist = d;
            hit.score = OrthoPaths.length(path);
            hit.nodeId = nid;
            Peel cand = Peel.of(nid, hit, path);
            if (best == null || cand.len + 0.4 < best.len || (old > 80 && cand.len + 12 < old && cand.len <= best.len + 8)) {
                best = cand;
            }
        }
        return best;
    }

    private static List<Coordinate> buildStub(ObstacleIndex obstacles, StreetFrame frame,
                                              Coordinate origin, Coordinate at) {
        if (origin == null || at == null) {
            return null;
        }
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        if (stubReachable(obstacles, origin, at)) {
            List<Coordinate> path = stubPath(obstacles, origin, at);
            if (path != null && path.size() >= 2) {
                best = path;
                bestLen = OrthoPaths.length(path);
            }
        }
        Coordinate from = obstacles.exitToStreet(origin, at, 1.2);
        if (from == null) {
            from = origin;
        }
        List<Coordinate> hug = obstacles.hugAround(from, at);
        if (hug != null && hug.size() >= 2) {
            List<Coordinate> path = new ArrayList<>();
            path.add(new Coordinate(origin));
            if (origin.distance(hug.get(0)) > 0.45) {
                appendStub(path, obstacles, origin, hug.get(0));
            }
            for (Coordinate q : hug) {
                if (q != null && path.get(path.size() - 1).distance(q) >= 0.4) {
                    path.add(new Coordinate(q));
                }
            }
            if (path.get(path.size() - 1).distance(at) > 0.45) {
                path.add(new Coordinate(at));
            }
            double len = OrthoPaths.length(path);
            if (len + 0.4 < bestLen) {
                best = path;
                bestLen = len;
            }
        }
        if (frame != null) {
            Coordinate snap = frame.attach(at);
            List<Coordinate> via = frame.find(from, snap != null ? snap : at);
            if (via != null && via.size() >= 2) {
                List<Coordinate> joined = join(obstacles, origin, via);
                if (joined.get(joined.size() - 1).distance(at) > 0.8) {
                    joined.add(new Coordinate(at));
                }
                double len = OrthoPaths.length(joined);
                if (len + 0.4 < bestLen) {
                    best = joined;
                }
            }
        }
        return best;
    }

    private static void appendStub(List<Coordinate> path, ObstacleIndex obstacles,
                                   Coordinate origin, Coordinate at) {
        if (origin.distance(at) <= 0.45) {
            return;
        }
        if ((ortho(obstacles, origin, at) || origin.distance(at) <= 8) && stubLegal(obstacles, origin, at)) {
            path.add(new Coordinate(at));
            return;
        }
        List<Coordinate> elbow = OrthoPaths.streetElbow(obstacles, origin, at);
        if (elbow == null || elbow.size() < 3) {
            elbow = OrthoPaths.usefulElbow(obstacles, origin, at);
        }
        if (elbow != null && elbow.size() >= 3) {
            for (int i = 1; i < elbow.size(); i++) {
                Coordinate q = elbow.get(i);
                if (path.get(path.size() - 1).distance(q) >= 0.4) {
                    path.add(new Coordinate(q));
                }
            }
            return;
        }
        Coordinate exit = obstacles.exitToStreet(origin, at, 1.2);
        List<Coordinate> hug = obstacles.hugAround(exit != null ? exit : origin, at);
        if (hug != null && hug.size() >= 2) {
            for (Coordinate q : hug) {
                if (q != null && path.get(path.size() - 1).distance(q) >= 0.4) {
                    path.add(new Coordinate(q));
                }
            }
            return;
        }
        path.add(new Coordinate(at));
    }

    private static List<NewSegment> exclusiveSpur(Variant variant, String oks) {
        Map<String, List<NewSegment>> byFrom = new HashMap<>();
        Map<String, Integer> deg = new HashMap<>();
        Set<String> taps = new HashSet<>();
        for (TapPoint t : variant.taps) {
            if (t.id != null) {
                taps.add(t.id);
            }
            if (t.nodeId != null) {
                taps.add(t.nodeId);
            }
        }
        for (NewSegment s : variant.segments) {
            if (s.fromId != null) {
                byFrom.computeIfAbsent(s.fromId, k -> new ArrayList<>()).add(s);
                deg.merge(s.fromId, 1, Integer::sum);
            }
            if (s.toId != null) {
                deg.merge(s.toId, 1, Integer::sum);
            }
        }
        List<NewSegment> spur = new ArrayList<>();
        String cur = oks;
        Set<String> seen = new HashSet<>();
        while (cur != null && seen.add(cur)) {
            List<NewSegment> next = byFrom.getOrDefault(cur, List.of());
            if (next.size() != 1) {
                break;
            }
            NewSegment s = next.get(0);
            spur.add(s);
            String to = s.toId;
            if (to == null || taps.contains(to)) {
                break;
            }
            int d = deg.getOrDefault(to, 0);
            if (d >= 3) {
                break;
            }
            cur = to;
        }
        return spur;
    }

    private static Hit nearestTree(Variant variant, ObstacleIndex obstacles, Coordinate origin,
                                   List<NewSegment> spur, double reach) {
        Set<NewSegment> skip = new HashSet<>(spur);
        Hit[] best = {null};
        for (NewSegment s : variant.segments) {
            if (skip.contains(s) || s.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            for (int i = 0; i < pts.length - 1; i++) {
                Coordinate axis = new Coordinate(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y);
                consider(obstacles, origin, pts[i], s, i == 0 ? s.fromId : null, best, i, reach, axis);
                int n = Math.max(1, (int) Math.floor(pts[i].distance(pts[i + 1]) / 3.0));
                for (int k = 1; k < n; k++) {
                    double t = k / (double) n;
                    Coordinate p = new Coordinate(
                            pts[i].x + t * (pts[i + 1].x - pts[i].x),
                            pts[i].y + t * (pts[i + 1].y - pts[i].y));
                    consider(obstacles, origin, p, s, null, best, i, reach, axis);
                }
            }
            Coordinate lastAxis = pts.length >= 2
                    ? new Coordinate(pts[pts.length - 1].x - pts[pts.length - 2].x,
                    pts[pts.length - 1].y - pts[pts.length - 2].y)
                    : null;
            consider(obstacles, origin, pts[pts.length - 1], s, s.toId, best, pts.length - 1, reach, lastAxis);
        }
        return best[0];
    }

    private static Hit nearestOnPath(ObstacleIndex obstacles, Coordinate origin, List<Coordinate> path,
                                     double reach) {
        Hit[] best = {null};
        Coordinate axis = longestAxis(path);
        for (int i = 0; i < path.size() - 1; i++) {
            Coordinate a = path.get(i);
            Coordinate b = path.get(i + 1);
            if (a == null || b == null) {
                continue;
            }
            consider(obstacles, origin, a, null, null, best, i, reach, axis);
            int n = Math.max(1, (int) Math.floor(a.distance(b) / 3.0));
            for (int k = 1; k < n; k++) {
                double t = k / (double) n;
                Coordinate p = new Coordinate(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
                consider(obstacles, origin, p, null, null, best, i + 1, reach, axis);
            }
            consider(obstacles, origin, b, null, null, best, i + 1, reach, axis);
        }
        if (path.size() == 1) {
            consider(obstacles, origin, path.get(0), null, null, best, 0, reach, axis);
        }
        return best[0];
    }

    private static Coordinate longestAxis(List<Coordinate> path) {
        Coordinate best = null;
        double bestL = 0;
        if (path == null) {
            return null;
        }
        for (int i = 0; i < path.size() - 1; i++) {
            Coordinate a = path.get(i);
            Coordinate b = path.get(i + 1);
            if (a == null || b == null) {
                continue;
            }
            double d = a.distance(b);
            if (d > bestL) {
                bestL = d;
                best = new Coordinate(b.x - a.x, b.y - a.y);
            }
        }
        return best;
    }

    private static void consider(ObstacleIndex obstacles, Coordinate origin, Coordinate p,
                                 NewSegment seg, String nodeId, Hit[] best, int pathIndex, double reach,
                                 Coordinate axis) {
        if (p == null) {
            return;
        }
        double d = origin.distance(p);
        if (d < 0.6 || d > reach) {
            return;
        }
        if (!stubReachable(obstacles, origin, p)) {
            return;
        }
        double s = d;
        if (!ortho(obstacles, origin, p) && d > 8) {
            s += 14;
        }
        if (axis != null && d > 2.0) {
            double ang = SpecialLayer.crossingAngleDeg(origin, p, axis);
            if (ang <= 18) {
                s += 26;
            } else if (ang >= 72) {
                s -= 3;
            } else {
                s += 12;
            }
        }
        Coordinate mid = new Coordinate((origin.x + p.x) * 0.5, (origin.y + p.y) * 0.5);
        if (obstacles.inRoad(mid) && !perpStreet(obstacles, origin, p)
                && !stubLegal(obstacles, origin, p)) {
            return;
        }
        if (nodeId != null) {
            s -= 0.4;
        }
        if (best[0] != null && s >= best[0].score) {
            return;
        }
        Hit h = new Hit();
        h.at = new Coordinate(p);
        h.dist = d;
        h.score = s;
        h.seg = seg;
        h.nodeId = nodeId;
        h.pathIndex = pathIndex;
        best[0] = h;
    }

    static boolean stubReachable(ObstacleIndex obstacles, Coordinate origin, Coordinate p) {
        if (stubLegal(obstacles, origin, p)) {
            return true;
        }
        List<Coordinate> elbow = OrthoPaths.streetElbow(obstacles, origin, p);
        if (elbow == null || elbow.size() < 3) {
            elbow = OrthoPaths.usefulElbow(obstacles, origin, p);
        }
        if (elbow != null && elbow.size() >= 3) {
            return true;
        }
        List<Coordinate> hug = obstacles.hugAround(
                obstacles.exitToStreet(origin, p, 1.2), p);
        return hug != null && hug.size() >= 3;
    }

    private static String ensureNode(Variant variant, ObstacleIndex obstacles, AtomicInteger ids, Hit hit) {
        if (hit.nodeId != null) {
            return hit.nodeId;
        }
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-" + ids.getAndIncrement();
        node.geometryMeters = GeoJsonGeometries.GF.createPoint(hit.at);
        node.reason = "itp_snap";
        variant.technicalNodes.add(node);
        NewSegment seg = hit.seg;
        if (seg == null || seg.geometryMeters == null) {
            return node.id;
        }
        Coordinate[] pts = seg.geometryMeters.getCoordinates();
        List<Coordinate> left = new ArrayList<>();
        List<Coordinate> right = new ArrayList<>();
        boolean passed = false;
        left.add(new Coordinate(pts[0]));
        for (int i = 1; i < pts.length; i++) {
            Coordinate a = passed ? right.get(right.size() - 1) : left.get(left.size() - 1);
            Coordinate b = pts[i];
            if (!passed && onSeg(a, b, hit.at)) {
                if (a.distance(hit.at) >= 0.4) {
                    left.add(new Coordinate(hit.at));
                }
                right.add(new Coordinate(hit.at));
                if (hit.at.distance(b) >= 0.4) {
                    right.add(new Coordinate(b));
                }
                passed = true;
                continue;
            }
            if (passed) {
                right.add(new Coordinate(b));
            } else {
                left.add(new Coordinate(b));
            }
        }
        if (!passed || left.size() < 2 || right.size() < 2) {
            return node.id;
        }
        String oldTo = seg.toId;
        seg.toId = node.id;
        seg.geometryMeters = GeoJsonGeometries.GF.createLineString(left.toArray(Coordinate[]::new));
        seg.lengthM = seg.geometryMeters.getLength();
        NewSegment rest = new NewSegment();
        rest.id = "NS-" + ids.getAndIncrement();
        rest.fromId = node.id;
        rest.toId = oldTo;
        rest.flowTph = seg.flowTph;
        rest.layingMethod = seg.layingMethod;
        rest.kSpec = seg.kSpec;
        rest.geometryMeters = GeoJsonGeometries.GF.createLineString(right.toArray(Coordinate[]::new));
        rest.lengthM = rest.geometryMeters.getLength();
        variant.segments.add(rest);
        return node.id;
    }

    private static boolean onSeg(Coordinate a, Coordinate b, Coordinate p) {
        double ab = a.distance(b);
        if (ab < 1e-6) {
            return a.distance(p) <= 0.6;
        }
        double t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / (ab * ab);
        if (t < -0.02 || t > 1.02) {
            return false;
        }
        double px = a.x + t * (b.x - a.x);
        double py = a.y + t * (b.y - a.y);
        return Math.hypot(p.x - px, p.y - py) <= 0.8;
    }

    static boolean stubLegal(ObstacleIndex obstacles, Coordinate origin, Coordinate p) {
        if (origin == null || p == null) {
            return false;
        }
        if (!obstacles.segmentHitsAvoid(origin, p, 0, true)) {
            return obstacles.allowsTravel(origin, p)
                    || origin.distance(p) <= 14
                    || obstacles.alongAvoid(origin, p, OrthoPaths.FACADE_M);
        }
        Coordinate exit = obstacles.exitToStreet(origin, p, 1.2);
        if (exit == null) {
            return origin.distance(p) <= 12;
        }
        if (exit.distance(p) < 0.4) {
            return origin.distance(p) <= 18;
        }
        if (obstacles.segmentHitsAvoid(exit, p, 0, true)) {
            return false;
        }
        return obstacles.allowsTravel(exit, p)
                || exit.distance(p) <= 16
                || obstacles.alongAvoid(exit, p, OrthoPaths.FACADE_M);
    }

    static boolean ortho(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        if (a.distance(b) <= 8) {
            return true;
        }
        return perpStreet(obstacles, a, b) || alongStreet(obstacles, a, b)
                || obstacles.alongAvoid(a, b, OrthoPaths.FACADE_M);
    }

    private static boolean perpStreet(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        SpecialLayer.Corridor c = obstacles.special().nearestCorridor(
                new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5), 60);
        if (c == null || c.axis == null) {
            return OrthoPaths.nearlyAxis(a, b);
        }
        double ang = SpecialLayer.crossingAngleDeg(a, b, c.axis);
        return ang >= 78;
    }

    private static boolean alongStreet(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        if (obstacles.inRoad(mid)) {
            return false;
        }
        SpecialLayer.Corridor c = obstacles.special().nearestCorridor(mid, 24);
        if (c == null || c.axis == null) {
            return false;
        }
        return SpecialLayer.crossingAngleDeg(a, b, c.axis) <= 12;
    }

    private static List<Coordinate> copy(List<Coordinate> path) {
        if (path == null) {
            return new ArrayList<>();
        }
        List<Coordinate> out = new ArrayList<>(path.size());
        for (Coordinate c : path) {
            if (c != null) {
                out.add(new Coordinate(c));
            }
        }
        return out;
    }

    private static final class Hit {
        Coordinate at;
        double dist;
        double score;
        NewSegment seg;
        String nodeId;
        int pathIndex;
    }

    public static final class Cut {
        public final List<Coordinate> stub;
        public final List<Coordinate> rest;
        public final Coordinate joinAt;

        Cut(List<Coordinate> stub, List<Coordinate> rest, Coordinate joinAt) {
            this.stub = stub;
            this.rest = rest;
            this.joinAt = joinAt;
        }
    }

    private static final class Peel {
        final String nodeId;
        final Hit hit;
        final List<Coordinate> path;
        final double len;

        private Peel(String nodeId, Hit hit, List<Coordinate> path, double len) {
            this.nodeId = nodeId;
            this.hit = hit;
            this.path = path;
            this.len = len;
        }

        static Peel of(String nodeId, Hit hit, List<Coordinate> path) {
            if (path == null || path.size() < 2) {
                return null;
            }
            return new Peel(nodeId, hit, path, OrthoPaths.length(path));
        }
    }
}
