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
        repairHouses(variant, obstacles, origin);
        shortenAll(variant, obstacles, origin);
        for (NewSegment seg : new ArrayList<>(variant.segments)) {
            liftCarriageway(seg, obstacles);
        }
        repairHouses(variant, obstacles, origin);
        shortenAll(variant, obstacles, origin);
        smoothJoints(variant, obstacles, origin);
        repairHouses(variant, obstacles, origin);
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
        if (itp && (obstacles.blocked(start) || obstacles.insideFootprint(start))) {
            Coordinate facade = nearestFacade(obstacles, start, pts[pts.length - 1]);
            if (facade != null && start.distance(facade) > 0.35) {
                out.add(new Coordinate(facade));
                cursor = facade;
            }
            if (cursor != start) {
                List<Coordinate> fresh = new ArrayList<>();
                fresh.add(new Coordinate(cursor));
                appendBridge(fresh, obstacles, frame, cursor, pts[pts.length - 1]);
                double freshLen = OrthoPaths.length(fresh);
                double oldLen = OrthoPaths.length(java.util.Arrays.asList(pts));
                if (fresh.size() >= 2 && housesClear(obstacles, fresh)
                        && freshLen <= oldLen + 18) {
                    write(seg, join(out.get(0), fresh));
                    return;
                }
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
        if (appendSkirt(out, obstacles, from, to)) {
            return;
        }
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        double cap = from.distance(to) * 3.2 + 64;
        List<Coordinate> hug = obstacles.hugAround(from, to);
        if (cleanPath(obstacles, hug)) {
            best = consider(best, bestLen, hug, cap);
            bestLen = OrthoPaths.length(best);
        }
        List<Coordinate> elbow = OrthoPaths.streetElbow(obstacles, from, to);
        if (cleanPath(obstacles, elbow)) {
            best = consider(best, bestLen, elbow, cap);
            bestLen = best == null ? bestLen : OrthoPaths.length(best);
        }
        List<Coordinate> side = sidewalkDetour(obstacles, from, to);
        if (cleanPath(obstacles, side)) {
            best = consider(best, bestLen, side, cap);
            bestLen = best == null ? bestLen : OrthoPaths.length(best);
        }
        if (best == null && frame != null) {
            List<Coordinate> via = frame.find(from, to);
            if (cleanPath(obstacles, via)) {
                best = consider(best, bestLen, via, cap);
            }
        }
        if (best == null || best.size() < 2) {
            if (!appendSkirt(out, obstacles, from, to)) {
                out.add(new Coordinate(to));
            }
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
                    double legA = a.distance(b);
                    double legC = b.distance(c);
                    double old = legA + legC;
                    double neu = a.distance(c);
                    double save = old - neu;
                    boolean jog = Math.min(legA, legC) < 2.8;
                    boolean shallow = turn > 152 && save > 0.2;
                    if (turn <= 168 && save < 1.2 && !jog && !shallow) {
                        continue;
                    }
                    if (!clean(obstacles, a, c, jog || shallow)) {
                        continue;
                    }
                    pts.remove(i);
                    changed = true;
                    break;
                }
            }
            spanCollapse(pts, obstacles, lockedFacade);
            if (pts.size() >= 2) {
                write(seg, pts);
            }
        }
    }

    /**
     * Хорда через несколько вершин, если она короче лесенки и не режет дом и проезжую.
     * Вершину фасада ИТП не выкидываем.
     */
    private static void spanCollapse(List<Coordinate> pts, ObstacleIndex obstacles, boolean lockedFacade) {
        boolean changed = true;
        int guard = 0;
        while (changed && pts.size() > 2 && guard++ < 8) {
            changed = false;
            for (int i = 0; i < pts.size() - 2; i++) {
                if (lockedFacade && i == 0) {
                    continue;
                }
                int farthest = -1;
                for (int j = pts.size() - 1; j >= i + 2; j--) {
                    if (lockedFacade && j > 1 && i < 1) {
                        continue;
                    }
                    Coordinate a = pts.get(i);
                    Coordinate c = pts.get(j);
                    double via = 0;
                    for (int k = i + 1; k <= j; k++) {
                        via += pts.get(k - 1).distance(pts.get(k));
                    }
                    if (via - a.distance(c) < 0.6) {
                        continue;
                    }
                    if (clean(obstacles, a, c, false)) {
                        farthest = j;
                        break;
                    }
                    List<Coordinate> arc = obstacles.skirt(a, c);
                    if (arc != null && arc.size() >= 3 && housesClear(obstacles, arc)
                            && via - OrthoPaths.length(arc) >= 0.8) {
                        pts.subList(i + 1, j).clear();
                        List<Coordinate> mid = new ArrayList<>();
                        for (int k = 1; k < arc.size() - 1; k++) {
                            mid.add(new Coordinate(arc.get(k)));
                        }
                        pts.addAll(i + 1, mid);
                        changed = true;
                        break;
                    }
                }
                if (changed) {
                    break;
                }
                if (farthest > i + 1) {
                    pts.subList(i + 1, farthest).clear();
                    changed = true;
                    break;
                }
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
        List<Coordinate> whole = new ArrayList<>();
        for (Coordinate c : pts) {
            whole.add(new Coordinate(c));
        }
        List<Coordinate> shifted = obstacles.liftOffCarriage(whole);
        if (shifted.size() == whole.size()) {
            List<Coordinate> out = new ArrayList<>();
            out.add(whole.get(0));
            for (Coordinate q : shifted) {
                if (out.get(out.size() - 1).distance(q) >= 0.35) {
                    out.add(q);
                }
            }
            Coordinate end = whole.get(whole.size() - 1);
            if (out.get(out.size() - 1).distance(end) >= 0.35) {
                out.add(end);
            }
            if (out.size() >= 2 && housesClear(obstacles, out)
                    && OrthoPaths.length(out) < OrthoPaths.length(whole) + 6) {
                write(seg, out);
                return;
            }
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
        if (travel.allowed && travel.crossingAngleDeg >= 45 && !obstacles.runsAlongFacade(a, b)) {
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

    /**
     * Угол на техническом узле не виден {@link #shortenAll}: тот смотрит внутрь одного участка.
     * Узел степени 2 переезжает на хорду, если она короче и не режет дом.
     * На прямом стволе (два плеча почти в одну линию) узел сдвигается вдоль ствола,
     * чтобы ветка подходила коротко, а не петлёй вдоль трубы.
     * Камеру и чужой узел существующей сети не двигаем.
     */
    private static void smoothJoints(Variant variant, ObstacleIndex obstacles,
                                     Map<String, Coordinate> origin) {
        for (int pass = 0; pass < 4; pass++) {
            boolean moved = false;
            for (String nodeId : technicalIds(variant)) {
                if (nodeId == null || origin.containsKey(nodeId)) {
                    continue;
                }
                List<Arm> arms = armsAt(variant, nodeId);
                if (arms.size() < 2) {
                    continue;
                }
                if (arms.size() == 2) {
                    moved |= straightenDeg2(variant, obstacles, nodeId, arms);
                } else {
                    moved |= slideOnTrunk(variant, obstacles, nodeId, arms);
                }
            }
            if (!moved) {
                break;
            }
        }
    }

    private static List<String> technicalIds(Variant variant) {
        List<String> ids = new ArrayList<>();
        for (TechnicalNode node : variant.technicalNodes) {
            if (node != null && node.id != null) {
                ids.add(node.id);
            }
        }
        return ids;
    }

    private static final class Arm {
        Coordinate at;
        Coordinate next;
    }

    private static List<Arm> armsAt(Variant variant, String nodeId) {
        List<Arm> arms = new ArrayList<>();
        for (NewSegment seg : variant.segments) {
            if (seg == null || seg.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = seg.geometryMeters.getCoordinates();
            if (pts.length < 2) {
                continue;
            }
            if (nodeId.equals(seg.fromId)) {
                Arm arm = new Arm();
                arm.at = pts[0];
                arm.next = pts[1];
                arms.add(arm);
            }
            if (nodeId.equals(seg.toId) && !nodeId.equals(seg.fromId)) {
                Arm arm = new Arm();
                arm.at = pts[pts.length - 1];
                arm.next = pts[pts.length - 2];
                arms.add(arm);
            }
        }
        return arms;
    }

    /** Угол одной трубы: узел встаёт на хорду между соседними вершинами. */
    private static boolean straightenDeg2(Variant variant, ObstacleIndex obstacles, String nodeId,
                                          List<Arm> arms) {
        Coordinate n = arms.get(0).at;
        Coordinate a = arms.get(0).next;
        Coordinate b = arms.get(1).next;
        if (n == null || a == null || b == null) {
            return false;
        }
        if (n.distance(a) < 0.6 || n.distance(b) < 0.6 || a.distance(b) < 0.6) {
            return false;
        }
        double save = n.distance(a) + n.distance(b) - a.distance(b);
        if (save < 0.8) {
            return false;
        }
        if (obstacles.footprintCutM(a, b) > 1.05) {
            return false;
        }
        double t = paramOn(n, a, b);
        if (t < 0.04 || t > 0.96) {
            t = 0.5;
        }
        Coordinate q = pointOn(a, b, t);
        if (n.distance(q) < 0.4 || obstacles.insideFootprint(q)) {
            return false;
        }
        if (obstacles.footprintCutM(a, q) > 1.05 || obstacles.footprintCutM(q, b) > 1.05) {
            return false;
        }
        moveTechnical(variant, nodeId, q);
        return true;
    }

    /**
     * Два плеча уже почти прямые. Остальные ветки тянут узел в угол.
     * Сдвиг вдоль этой прямой укорачивает ветку и не ломает ствол.
     */
    private static boolean slideOnTrunk(Variant variant, ObstacleIndex obstacles, String nodeId,
                                        List<Arm> arms) {
        int bestI = -1;
        int bestJ = -1;
        double bestTurn = 0;
        for (int i = 0; i < arms.size(); i++) {
            for (int j = i + 1; j < arms.size(); j++) {
                double turn = turnDeg(arms.get(i).next, arms.get(i).at, arms.get(j).next);
                if (turn > bestTurn) {
                    bestTurn = turn;
                    bestI = i;
                    bestJ = j;
                }
            }
        }
        if (bestI < 0 || bestTurn < 158) {
            return false;
        }
        Coordinate a = arms.get(bestI).next;
        Coordinate b = arms.get(bestJ).next;
        Coordinate n = arms.get(bestI).at;
        if (a.distance(b) < 1.0 || n.distance(a) < 0.6 || n.distance(b) < 0.6) {
            return false;
        }
        double old = 0;
        for (Arm arm : arms) {
            old += n.distance(arm.next);
        }
        Coordinate bestQ = null;
        double bestSave = 1.2;
        List<Double> samples = new ArrayList<>();
        for (int s = 1; s <= 16; s++) {
            samples.add(s / 17.0);
        }
        for (Arm arm : arms) {
            samples.add(paramOn(arm.next, a, b));
        }
        for (double raw : samples) {
            double t = Math.max(0.03, Math.min(0.97, raw));
            Coordinate q = pointOn(a, b, t);
            if (n.distance(q) < 0.8 || obstacles.insideFootprint(q)) {
                continue;
            }
            boolean clear = true;
            double neu = 0;
            for (Arm arm : arms) {
                if (obstacles.footprintCutM(q, arm.next) > 1.05) {
                    clear = false;
                    break;
                }
                neu += q.distance(arm.next);
            }
            if (!clear) {
                continue;
            }
            double save = old - neu;
            if (save > bestSave) {
                bestSave = save;
                bestQ = q;
            }
        }
        if (bestQ == null) {
            return false;
        }
        moveTechnical(variant, nodeId, bestQ);
        return true;
    }

    private static void moveTechnical(Variant variant, String nodeId, Coordinate q) {
        Coordinate at = new Coordinate(q);
        for (TechnicalNode node : variant.technicalNodes) {
            if (node != null && nodeId.equals(node.id)) {
                node.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(at));
            }
        }
        for (NewSegment seg : variant.segments) {
            if (seg == null || seg.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = seg.geometryMeters.getCoordinates();
            if (pts.length < 2) {
                continue;
            }
            boolean changed = false;
            if (nodeId.equals(seg.fromId)) {
                pts[0] = new Coordinate(at);
                changed = true;
            }
            if (nodeId.equals(seg.toId)) {
                pts[pts.length - 1] = new Coordinate(at);
                changed = true;
            }
            if (changed) {
                write(seg, java.util.Arrays.asList(pts));
            }
        }
    }

    private static double paramOn(Coordinate p, Coordinate a, Coordinate b) {
        double vx = b.x - a.x;
        double vy = b.y - a.y;
        double l2 = vx * vx + vy * vy;
        if (l2 < 1e-6 || p == null) {
            return 0.5;
        }
        return ((p.x - a.x) * vx + (p.y - a.y) * vy) / l2;
    }

    private static Coordinate pointOn(Coordinate a, Coordinate b, double t) {
        return new Coordinate(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
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
                applyPiece(obstacles, seg, pieces.isEmpty() ? null : pieces.get(0), path);
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
                applyPiece(obstacles, part, piece, piece.coords);
                next.add(part);
                prev = to;
            }
        }
        variant.segments.clear();
        variant.segments.addAll(next);
    }

    private static void applyPiece(ObstacleIndex obstacles, NewSegment seg, SpecialLayer.Piece piece,
                                   List<Coordinate> fallback) {
        List<Coordinate> coords = piece == null || piece.coords.size() < 2 ? fallback : piece.coords;
        write(seg, coords);
        boolean crossing = realCrossing(obstacles, piece) || gapCrossing(obstacles, coords);
        if (crossing) {
            seg.layingMethod = "special";
            double k = piece != null && piece.kSpec > 1 ? piece.kSpec : 1.6;
            seg.kSpec = k;
            String reason = piece == null ? null : piece.reason;
            seg.specialReason = reason == null || reason.isBlank() ? "road" : reason;
        } else {
            seg.layingMethod = "base";
            seg.kSpec = 1.0;
            seg.specialReason = null;
        }
    }

    /**
     * Спецметод — короткий проход поперёк проезжей. Ход вдоль фасада и длинный
     * кусок, даже если ось синтезированной полосы смотрит поперёк, остаются base.
     */
    private static boolean realCrossing(ObstacleIndex obstacles, SpecialLayer.Piece piece) {
        if (piece == null || !piece.special || piece.coords.size() < 2 || obstacles.special() == null) {
            return false;
        }
        Coordinate a = piece.coords.get(0);
        Coordinate b = piece.coords.get(piece.coords.size() - 1);
        double len = a.distance(b);
        // Короче 4.5 м — зацеп угла, не проезжая. Длиннее ~50 м — уже ход вдоль,
        // даже если синтезированная ось смотрит поперёк. Широкая улица (до ~40 м
        // плюс вылет) остаётся спецметодом, если угол к фасаду прямой.
        if (len < 4.5 || len > 52) {
            return false;
        }
        if (obstacles.runsAlongFacade(a, b)) {
            return false;
        }
        SpecialLayer.Travel travel = obstacles.special().inspect(a, b);
        if (!travel.allowed || !travel.special) {
            return false;
        }
        if (travel.crossingAngleDeg + 1e-6 < 55) {
            return false;
        }
        // Ось синтезированной полосы иногда повёрнута, и ход вдоль фасада
        // выглядит как пересечение. Настоящий проход либо поперёк стены,
        // либо глубоко внутри проезжей и под большим углом к её оси.
        double facade = obstacles.angleToNearestFacade(a, b);
        boolean deepCross = travel.hitM >= 8
                && travel.hitM + 1e-6 >= len * 0.45
                && travel.crossingAngleDeg >= 70;
        if (!deepCross && facade < 60) {
            return false;
        }
        return true;
    }

    /**
     * Полоса синтеза могла не накрыть щель. Короткий ход поперёк двух фасадов
     * всё равно спецпроход; длинный кусок вдоль стены — нет.
     */
    private static List<Coordinate> join(Coordinate origin, List<Coordinate> rest) {
        List<Coordinate> all = new ArrayList<>();
        all.add(new Coordinate(origin));
        if (rest != null) {
            for (Coordinate c : rest) {
                if (c != null && all.get(all.size() - 1).distance(c) >= 0.3) {
                    all.add(new Coordinate(c));
                }
            }
        }
        return all;
    }

    private static boolean gapCrossing(ObstacleIndex obstacles, List<Coordinate> coords) {
        if (coords == null || coords.size() < 2) {
            return false;
        }
        Coordinate a = coords.get(0);
        Coordinate b = coords.get(coords.size() - 1);
        return obstacles.crossesStreetGap(a, b);
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
     * Вывод ⊥ ближайшему фасаду. Стена дальше чем на 1.2 м от самой короткой
     * не берётся: иначе труба идёт внутри корпуса вдоль стены до торца.
     * Среди почти равных сторон короче та, от которой ближе уже построенный конец.
     */
    private static Coordinate nearestFacade(ObstacleIndex obstacles, Coordinate origin, Coordinate toward) {
        List<Coordinate> sides = obstacles.facadeNormals(origin, 1.2);
        if (sides == null || sides.isEmpty()) {
            sides = obstacles.sideExits(origin, 1.2);
        }
        if (sides == null || sides.isEmpty()) {
            Coordinate fallback = obstacles.wallPerpExit(origin, toward, 1.2);
            return fallback == null ? null : new Coordinate(fallback);
        }
        double shortest = Double.POSITIVE_INFINITY;
        for (Coordinate q : sides) {
            if (q != null) {
                shortest = Math.min(shortest, origin.distance(q));
            }
        }
        Coordinate best = null;
        double bestS = Double.POSITIVE_INFINITY;
        for (Coordinate q : sides) {
            if (q == null) {
                continue;
            }
            double indoor = origin.distance(q);
            if (indoor > shortest + 1.2) {
                continue;
            }
            double outdoor = 0;
            if (toward != null && q.distance(toward) > 0.4) {
                outdoor = q.distance(toward);
                if (obstacles.footprintCutM(q, toward) > 1.05) {
                    List<Coordinate> arc = obstacles.skirt(q, toward);
                    if (arc != null && housesClear(obstacles, arc)) {
                        outdoor = OrthoPaths.length(arc);
                    } else {
                        outdoor += 30;
                    }
                }
            }
            double score = indoor + outdoor;
            if (score < bestS) {
                bestS = score;
                best = q;
            }
        }
        if (best == null) {
            best = obstacles.wallPerpExit(origin, null, 1.2);
        }
        return best == null ? null : new Coordinate(best);
    }

    /**
     * Ребро сквозь кадастр заменяется дугой по фасаду. Ввод ИТП до своей стены не трогаем,
     * если он короткий и поперёк этой стены.
     */
    private static void repairHouses(Variant variant, ObstacleIndex obstacles, Map<String, Coordinate> origin) {
        for (NewSegment seg : variant.segments) {
            if (seg == null || seg.geometryMeters == null) {
                continue;
            }
            List<Coordinate> pts = new ArrayList<>();
            for (Coordinate c : seg.geometryMeters.getCoordinates()) {
                pts.add(new Coordinate(c));
            }
            boolean itp = seg.fromId != null && origin.containsKey(seg.fromId);
            boolean changed = false;
            for (int i = 0; i < pts.size(); i++) {
                if (itp && i == 0) {
                    continue;
                }
                if (!obstacles.insideFootprint(pts.get(i))) {
                    continue;
                }
                Coordinate exit = obstacles.wallPerpExit(pts.get(i), i == 0 ? null : pts.get(0), 1.2);
                if (exit != null && pts.get(i).distance(exit) > 0.4 && pts.get(i).distance(exit) < 40
                        && !obstacles.insideFootprint(exit)) {
                    pts.set(i, new Coordinate(exit));
                    changed = true;
                }
            }
            List<Coordinate> out = new ArrayList<>();
            if (pts.isEmpty()) {
                continue;
            }
            out.add(pts.get(0));
            for (int i = 1; i < pts.size(); i++) {
                Coordinate a = out.get(out.size() - 1);
                Coordinate b = pts.get(i);
                boolean stub = itp && out.size() == 1 && obstacles.blocked(pts.get(0))
                        && allowedStub(obstacles, pts.get(0), b);
                if (!stub && obstacles.footprintCutM(a, b) > 1.05) {
                    List<Coordinate> arc = obstacles.skirt(a, b);
                    if (arc != null && arc.size() >= 3 && housesClear(obstacles, arc)) {
                        for (int k = 1; k < arc.size(); k++) {
                            if (out.get(out.size() - 1).distance(arc.get(k)) >= 0.4) {
                                out.add(new Coordinate(arc.get(k)));
                            }
                        }
                        changed = true;
                        continue;
                    }
                }
                if (a.distance(b) >= 0.35) {
                    out.add(new Coordinate(b));
                }
            }
            if (changed && out.size() >= 2) {
                write(seg, out);
            }
        }
    }

    /**
     * Короткий ввод из точки ИТП до своей стены: один заход в свой корпус,
     * без повторного входа в чужой дом.
     */
    private static boolean allowedStub(ObstacleIndex obstacles, Coordinate origin, Coordinate exit) {
        if (origin == null || exit == null) {
            return false;
        }
        double len = origin.distance(exit);
        if (len > 24 || len < 0.3 || obstacles.insideFootprint(exit)) {
            return false;
        }
        int steps = Math.max(4, (int) Math.ceil(len / 2.0));
        boolean left = false;
        for (int t = 1; t < steps; t++) {
            double u = t / (double) steps;
            Coordinate q = new Coordinate(
                    origin.x + u * (exit.x - origin.x),
                    origin.y + u * (exit.y - origin.y));
            boolean in = obstacles.insideFootprint(q);
            if (!in) {
                left = true;
            } else if (left) {
                return false;
            }
        }
        return true;
    }

    private static boolean appendSkirt(List<Coordinate> out, ObstacleIndex obstacles,
                                       Coordinate from, Coordinate to) {
        if (obstacles.footprintCutM(from, to) < 1.05) {
            return false;
        }
        List<Coordinate> arc = obstacles.skirt(from, to);
        if (arc == null || arc.size() < 3 || !housesClear(obstacles, arc)) {
            return false;
        }
        for (int i = 1; i < arc.size(); i++) {
            Coordinate q = arc.get(i);
            if (q != null && out.get(out.size() - 1).distance(q) >= 0.35) {
                out.add(new Coordinate(q));
            }
        }
        return true;
    }

    private static boolean housesClear(ObstacleIndex obstacles, List<Coordinate> path) {
        if (path == null || path.size() < 2) {
            return false;
        }
        for (int i = 1; i < path.size(); i++) {
            if (obstacles.footprintCutM(path.get(i - 1), path.get(i)) > 1.05) {
                return false;
            }
        }
        return true;
    }

    private static boolean clean(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        return clean(obstacles, a, b, false);
    }

    /**
     * Хорда законна, если не входит в кадастр и проезжая её пускает.
     * Буфер квартала (закрытый двор) сам по себе хорду не запрещает: иначе
     * остаётся лесенка вокруг искусственного пятна, хотя между домами свободно.
     */
    private static boolean clean(ObstacleIndex obstacles, Coordinate a, Coordinate b, boolean strict) {
        if (a == null || b == null || a.distance(b) < 0.05) {
            return true;
        }
        double cut = obstacles.footprintCutM(a, b);
        if (cut > (strict ? 0.45 : 1.05)) {
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
