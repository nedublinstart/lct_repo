package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.OrthoPaths;
import ru.lct.heatnet.engine.greedy.SpecialLayer;
import ru.lct.heatnet.engine.greedy.StreetFrame;
import ru.lct.heatnet.geo.GeoJsonGeometries;

/**
 * Последняя геометрия перед сметой.
 * <p>
 * Порядок жёсткий. Сначала ИТП выходит перпендикулярно ближайшему фасаду —
 * короткий отрезок из точки подключения до стены. Дальше труба идёт только
 * от этой точки фасада: по тротуару, а проезжую пересекает коротко и поперёк.
 * Хорда сквозь корпус и ход вдоль проезжей на выдаче не остаются.
 * Лишняя вершина выкидывается, если прямая короче и не режет дом и проезжую.
 */
public final class LiquidRoutes {

    private LiquidRoutes() {
    }

    public static void apply(Variant variant, ObstacleIndex obstacles, StreetFrame frame,
                             AtomicInteger ids, List<OksPort> ports) {
        if (variant == null || obstacles == null || variant.segments.isEmpty()) {
            return;
        }
        Map<String, Coordinate> origin = origins(ports);
        pushBuriedNodes(variant, obstacles, origin);
        for (NewSegment seg : variant.segments) {
            rewrite(seg, obstacles, frame, origin);
        }
        shortenAll(variant, obstacles, origin);
        for (NewSegment seg : new ArrayList<>(variant.segments)) {
            liftCarriageway(seg, obstacles);
        }
        shortenAll(variant, obstacles, origin);
        resplit(variant, obstacles, ids);
    }

    private static Map<String, Coordinate> origins(List<OksPort> ports) {
        Map<String, Coordinate> origin = new HashMap<>();
        if (ports == null) {
            return origin;
        }
        for (OksPort p : ports) {
            if (p != null && p.id() != null && p.origin != null) {
                origin.put(p.id(), new Coordinate(p.origin));
            }
        }
        return origin;
    }

    /** Технический узел внутри корпуса переезжает на ближайший фасад. Точку ИТП не двигаем. */
    private static void pushBuriedNodes(Variant variant, ObstacleIndex obstacles,
                                        Map<String, Coordinate> origin) {
        for (TechnicalNode node : variant.technicalNodes) {
            if (node == null || node.id == null || node.geometryMeters == null) {
                continue;
            }
            Coordinate at = node.geometryMeters.getCoordinate();
            if (at == null || !obstacles.blocked(at) || nearOrigin(origin, at)) {
                continue;
            }
            Coordinate facade = nearestFacade(obstacles, at, null);
            if (facade == null || at.distance(facade) < 0.4 || at.distance(facade) > 80) {
                continue;
            }
            node.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(facade));
            for (NewSegment seg : variant.segments) {
                moveEnd(seg, node.id, at, facade, origin);
            }
        }
    }

    private static void moveEnd(NewSegment seg, String nodeId, Coordinate old, Coordinate facade,
                                Map<String, Coordinate> origin) {
        if (seg == null || seg.geometryMeters == null) {
            return;
        }
        Coordinate[] pts = seg.geometryMeters.getCoordinates();
        if (pts.length < 2) {
            return;
        }
        boolean moved = false;
        if (nodeId.equals(seg.fromId) && !isOrigin(origin, pts[0])) {
            pts[0] = new Coordinate(facade);
            moved = true;
        } else if (pts[0].distance(old) <= 0.8 && !isOrigin(origin, pts[0])) {
            pts[0] = new Coordinate(facade);
            moved = true;
        }
        int last = pts.length - 1;
        if (nodeId.equals(seg.toId)) {
            pts[last] = new Coordinate(facade);
            moved = true;
        } else if (pts[last].distance(old) <= 0.8 && !isOrigin(origin, pts[last])) {
            pts[last] = new Coordinate(facade);
            moved = true;
        }
        if (moved) {
            write(seg, java.util.Arrays.asList(pts));
        }
    }

    /**
     * Ввод ИТП: [точка, ближайший фасад ⊥ стене], затем наружная трасса до конца участка.
     * Остальные участки: каждое ребро, которое режет корпус или идёт вдоль проезжей,
     * заменяется обходом снаружи.
     */
    private static void rewrite(NewSegment seg, ObstacleIndex obstacles, StreetFrame frame,
                                Map<String, Coordinate> origin) {
        if (seg == null || seg.geometryMeters == null) {
            return;
        }
        Coordinate[] pts = seg.geometryMeters.getCoordinates();
        if (pts.length < 2) {
            return;
        }
        List<Coordinate> out = new ArrayList<>();
        Coordinate start = pts[0];
        boolean itp = seg.fromId != null && origin.containsKey(seg.fromId);
        if (itp) {
            start = origin.get(seg.fromId);
        }
        out.add(new Coordinate(start));
        Coordinate cursor = start;
        if (itp && obstacles.blocked(start)) {
            Coordinate facade = nearestFacade(obstacles, start, pts[pts.length - 1]);
            if (facade != null && start.distance(facade) > 0.35) {
                out.add(new Coordinate(facade));
                cursor = facade;
            }
        }
        for (int i = 1; i < pts.length; i++) {
            Coordinate target = pts[i];
            if (target == null) {
                continue;
            }
            if (obstacles.blocked(target) && i < pts.length - 1) {
                continue;
            }
            if (cursor.distance(target) < 0.35) {
                continue;
            }
            appendBridge(out, obstacles, frame, cursor, target);
            cursor = out.get(out.size() - 1);
        }
        if (out.size() >= 2) {
            write(seg, out);
        }
    }

    private static void appendBridge(List<Coordinate> out, ObstacleIndex obstacles, StreetFrame frame,
                                     Coordinate from, Coordinate to) {
        if (from.distance(to) < 0.35) {
            return;
        }
        if (clean(obstacles, from, to)) {
            out.add(new Coordinate(to));
            return;
        }
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        double cap = from.distance(to) * 6 + 120;
        List<Coordinate> via = frame == null ? null : frame.find(from, to);
        if (cleanPath(obstacles, via)) {
            best = consider(best, bestLen, via, cap);
            bestLen = OrthoPaths.length(best);
        }
        List<Coordinate> hug = obstacles.hugAround(from, to);
        if (cleanPath(obstacles, hug)) {
            best = consider(best, bestLen, hug, cap);
            bestLen = best == null ? bestLen : OrthoPaths.length(best);
        }
        List<Coordinate> elbow = OrthoPaths.streetElbow(obstacles, from, to);
        if (cleanPath(obstacles, elbow)) {
            best = consider(best, bestLen, elbow, cap);
            bestLen = best == null ? bestLen : OrthoPaths.length(best);
        }
        List<Coordinate> side = sidewalkDetour(obstacles, from, to);
        if (cleanPath(obstacles, side)) {
            best = consider(best, bestLen, side, cap);
        }
        if (best == null || best.size() < 2) {
            out.add(new Coordinate(to));
            return;
        }
        for (int i = 1; i < best.size(); i++) {
            Coordinate q = best.get(i);
            if (q != null && out.get(out.size() - 1).distance(q) >= 0.35) {
                out.add(new Coordinate(q));
            }
        }
        if (out.get(out.size() - 1).distance(to) > 0.6) {
            out.add(new Coordinate(to));
        }
    }

    private static List<Coordinate> consider(List<Coordinate> best, double bestLen,
                                             List<Coordinate> cand, double cap) {
        if (cand == null || cand.size() < 2) {
            return best;
        }
        double len = OrthoPaths.length(cand);
        if (len > cap || len + 0.4 >= bestLen) {
            return best;
        }
        return cand;
    }

    /** Почти прямые точки и лишний крюк выкидываются, если хорда короче и чистая. Фасад ИТП не трогаем. */
    private static void shortenAll(Variant variant, ObstacleIndex obstacles,
                                   Map<String, Coordinate> origin) {
        for (NewSegment seg : variant.segments) {
            if (seg == null || seg.geometryMeters == null) {
                continue;
            }
            List<Coordinate> pts = new ArrayList<>();
            for (Coordinate c : seg.geometryMeters.getCoordinates()) {
                pts.add(new Coordinate(c));
            }
            boolean lockedFacade = seg.fromId != null && origin.containsKey(seg.fromId)
                    && pts.size() >= 2 && obstacles.blocked(pts.get(0));
            boolean changed = true;
            int guard = 0;
            while (changed && pts.size() > 2 && guard++ < 12) {
                changed = false;
                for (int i = 1; i < pts.size() - 1; i++) {
                    if (lockedFacade && i == 1) {
                        continue;
                    }
                    Coordinate a = pts.get(i - 1);
                    Coordinate b = pts.get(i);
                    Coordinate c = pts.get(i + 1);
                    double turn = turnDeg(a, b, c);
                    double old = a.distance(b) + b.distance(c);
                    double neu = a.distance(c);
                    boolean almost = turn > 168;
                    if (!almost && old - neu < 1.2) {
                        continue;
                    }
                    if (!clean(obstacles, a, c)) {
                        continue;
                    }
                    pts.remove(i);
                    changed = true;
                    break;
                }
            }
            if (pts.size() >= 2) {
                write(seg, pts);
            }
        }
    }

    /**
     * Ребро вдоль проезжей заменяется выходом на тротуар. Само пересечение
     * остаётся коротким и поперёк — его пометит {@link #resplit}.
     */
    private static void liftCarriageway(NewSegment seg, ObstacleIndex obstacles) {
        if (seg == null || seg.geometryMeters == null || obstacles.special() == null) {
            return;
        }
        Coordinate[] pts = seg.geometryMeters.getCoordinates();
        if (pts.length < 2) {
            return;
        }
        List<Coordinate> out = new ArrayList<>();
        out.add(new Coordinate(pts[0]));
        for (int i = 1; i < pts.length; i++) {
            Coordinate a = out.get(out.size() - 1);
            Coordinate b = pts[i];
            List<Coordinate> side = sidewalkDetour(obstacles, a, b);
            if (side != null && side.size() >= 3 && cleanPath(obstacles, side)
                    && OrthoPaths.length(side) < a.distance(b) * 1.9 + 24) {
                for (int k = 1; k < side.size(); k++) {
                    if (out.get(out.size() - 1).distance(side.get(k)) >= 0.35) {
                        out.add(new Coordinate(side.get(k)));
                    }
                }
            } else if (out.get(out.size() - 1).distance(b) >= 0.35) {
                out.add(new Coordinate(b));
            }
        }
        if (out.size() >= 2) {
            write(seg, out);
        }
    }

    private static List<Coordinate> sidewalkDetour(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        if (a == null || b == null || obstacles.special() == null || a.distance(b) < 6) {
            return null;
        }
        SpecialLayer.Travel travel = obstacles.special().inspect(a, b);
        if (travel.allowed && !travel.special) {
            return null;
        }
        if (travel.allowed && travel.crossingAngleDeg >= 45) {
            return null;
        }
        Coordinate sa = sidewalk(obstacles, a, b);
        Coordinate sb = sidewalk(obstacles, b, a);
        if (sa == null || sb == null) {
            return null;
        }
        if (sa.distance(sb) < 0.4) {
            return null;
        }
        List<Coordinate> path = new ArrayList<>();
        path.add(new Coordinate(a));
        if (a.distance(sa) >= 0.4) {
            path.add(sa);
        }
        if (path.get(path.size() - 1).distance(sb) >= 0.4) {
            path.add(sb);
        }
        if (path.get(path.size() - 1).distance(b) >= 0.4) {
            path.add(new Coordinate(b));
        }
        return path.size() >= 3 ? path : null;
    }

    private static Coordinate sidewalk(ObstacleIndex obstacles, Coordinate c, Coordinate other) {
        if (c == null) {
            return null;
        }
        SpecialLayer.Corridor cor = obstacles.special().nearestCorridor(c, 40);
        if (cor == null || cor.axis == null) {
            return null;
        }
        if (!obstacles.inRoad(c) && cor.geom != null
                && cor.geom.distance(GeoJsonGeometries.GF.createPoint(c)) > 1.2) {
            return new Coordinate(c);
        }
        Coordinate n = cor.perp();
        double shift = Math.max(2.2, cor.widthM * 0.5 + 1.4);
        Coordinate p = new Coordinate(c.x + n.x * shift, c.y + n.y * shift);
        Coordinate q = new Coordinate(c.x - n.x * shift, c.y - n.y * shift);
        Coordinate prefer = other == null ? p : other;
        Coordinate best = null;
        double bestS = Double.POSITIVE_INFINITY;
        for (Coordinate cand : new Coordinate[]{p, q}) {
            if (obstacles.blocked(cand) || obstacles.inRoad(cand)) {
                continue;
            }
            double s = cand.distance(prefer);
            if (s < bestS) {
                bestS = s;
                best = cand;
            }
        }
        return best == null ? null : new Coordinate(best);
    }

    /** Спецметод только на разрешённом поперечном проходе. Подход по тротуару остаётся base. */
    private static void resplit(Variant variant, ObstacleIndex obstacles, AtomicInteger ids) {
        if (ids == null || obstacles.special() == null) {
            return;
        }
        List<NewSegment> next = new ArrayList<>();
        for (NewSegment seg : variant.segments) {
            if (seg == null || seg.geometryMeters == null || seg.geometryMeters.getNumPoints() < 2) {
                continue;
            }
            List<Coordinate> path = new ArrayList<>();
            for (Coordinate c : seg.geometryMeters.getCoordinates()) {
                path.add(new Coordinate(c));
            }
            List<SpecialLayer.Piece> pieces = obstacles.splitByTransport(path);
            if (pieces.size() <= 1) {
                applyPiece(seg, pieces.isEmpty() ? null : pieces.get(0), path);
                next.add(seg);
                continue;
            }
            String prev = seg.fromId;
            for (int i = 0; i < pieces.size(); i++) {
                SpecialLayer.Piece piece = pieces.get(i);
                if (piece.coords.size() < 2) {
                    continue;
                }
                boolean last = i == pieces.size() - 1;
                String to = last ? seg.toId : technical(variant, ids, piece.end());
                NewSegment part = new NewSegment();
                part.id = last && pieces.size() == 1 ? seg.id : "NS-" + ids.getAndIncrement();
                part.fromId = prev;
                part.toId = to;
                part.flowTph = seg.flowTph;
                part.dn = seg.dn;
                part.parentId = seg.parentId;
                applyPiece(part, piece, piece.coords);
                next.add(part);
                prev = to;
            }
        }
        variant.segments.clear();
        variant.segments.addAll(next);
    }

    private static void applyPiece(NewSegment seg, SpecialLayer.Piece piece, List<Coordinate> fallback) {
        List<Coordinate> coords = piece == null || piece.coords.size() < 2 ? fallback : piece.coords;
        write(seg, coords);
        if (piece != null && piece.special) {
            seg.layingMethod = "special";
            seg.kSpec = piece.kSpec > 1 ? piece.kSpec : 1.6;
            seg.specialReason = piece.reason == null || piece.reason.isBlank() ? "road" : piece.reason;
        } else {
            seg.layingMethod = "base";
            seg.kSpec = 1.0;
            seg.specialReason = null;
        }
    }

    private static String technical(Variant variant, AtomicInteger ids, Coordinate at) {
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-" + ids.getAndIncrement();
        node.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(at));
        node.reason = "leave_special";
        variant.technicalNodes.add(node);
        return node.id;
    }

    /**
     * ⊥ одной из ближайших стен. Если две стены почти одинаково близко,
     * берём ту, что смотрит в сторону уже построенного конца трассы.
     */
    private static Coordinate nearestFacade(ObstacleIndex obstacles, Coordinate origin, Coordinate toward) {
        List<Coordinate> exits = new ArrayList<>(obstacles.wallPerpExits(origin, 1.2));
        Coordinate aimed = obstacles.wallPerpExit(origin, toward, 1.2);
        if (aimed != null) {
            exits.add(aimed);
        }
        double shortest = Double.POSITIVE_INFINITY;
        for (Coordinate e : exits) {
            if (e != null) {
                shortest = Math.min(shortest, origin.distance(e));
            }
        }
        Coordinate best = null;
        double bestS = Double.POSITIVE_INFINITY;
        for (Coordinate e : exits) {
            if (e == null) {
                continue;
            }
            double indoor = origin.distance(e);
            if (indoor > shortest + 4) {
                continue;
            }
            double s = indoor * 3 + (toward == null ? 0 : e.distance(toward));
            if (s < bestS) {
                bestS = s;
                best = e;
            }
        }
        if (best == null) {
            best = obstacles.wallPerpExit(origin, toward, 1.2);
        }
        return best == null ? null : new Coordinate(best);
    }

    private static boolean clean(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        if (a == null || b == null || a.distance(b) < 0.05) {
            return true;
        }
        if (obstacles.segmentHitsAvoid(a, b, 0, true)) {
            return false;
        }
        if (obstacles.special() == null) {
            return true;
        }
        return obstacles.special().inspect(a, b).allowed;
    }

    private static boolean cleanPath(ObstacleIndex obstacles, List<Coordinate> path) {
        if (path == null || path.size() < 2) {
            return false;
        }
        for (int i = 1; i < path.size(); i++) {
            if (!clean(obstacles, path.get(i - 1), path.get(i))) {
                return false;
            }
        }
        return true;
    }

    private static double turnDeg(Coordinate a, Coordinate b, Coordinate c) {
        double ax = a.x - b.x;
        double ay = a.y - b.y;
        double cx = c.x - b.x;
        double cy = c.y - b.y;
        double na = Math.hypot(ax, ay);
        double nc = Math.hypot(cx, cy);
        if (na < 0.15 || nc < 0.15) {
            return 180;
        }
        double d = (ax * cx + ay * cy) / (na * nc);
        d = Math.max(-1, Math.min(1, d));
        return Math.toDegrees(Math.acos(d));
    }

    private static boolean nearOrigin(Map<String, Coordinate> origin, Coordinate at) {
        for (Coordinate c : origin.values()) {
            if (c != null && c.distance(at) <= 1.2) {
                return true;
            }
        }
        return false;
    }

    private static boolean isOrigin(Map<String, Coordinate> origin, Coordinate at) {
        return at != null && nearOrigin(origin, at);
    }

    private static void write(NewSegment seg, List<Coordinate> pts) {
        List<Coordinate> copy = new ArrayList<>();
        Coordinate prev = null;
        for (Coordinate c : pts) {
            if (c == null) {
                continue;
            }
            if (prev != null && prev.distance(c) < 0.05) {
                continue;
            }
            copy.add(new Coordinate(c));
            prev = c;
        }
        if (copy.size() < 2) {
            return;
        }
        seg.geometryMeters = GeoJsonGeometries.GF.createLineString(copy.toArray(Coordinate[]::new));
        seg.lengthM = seg.geometryMeters.getLength();
    }
}
