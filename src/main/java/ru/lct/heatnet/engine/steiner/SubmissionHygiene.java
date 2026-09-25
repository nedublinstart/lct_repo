package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.operation.distance.DistanceOp;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.Scene;

/**
 * Последний проход перед стоимостью и выгрузкой GeoJSON.
 * <p>
 * ТЗ считает врезку ({@code tie_in}, 5 млн ₽) только там, где новая труба
 * реально приходит на существующую сеть. Фантом — маркер без инцидентной
 * трубы, вторая врезка в ту же точку той же компоненты, или новая камера
 * «на врезке», которую никто не использует. На карте такой объект выглядит
 * как лишняя точка, а в смете — как лишние миллионы.
 * <p>
 * Второй дефект — разрыв геометрии при целых идентификаторах: участок
 * кончается в 1–4 м от {@code technical_node} или {@code tie_in}. В графе
 * ОКС «подключён», на карте труба не доходит до маркера. Концы короче 4 м
 * подтягиваются к узлу, длина пересчитывается по геометрии.
 */
public final class SubmissionHygiene {

    /** Дальше 4 м конец не двигаем: это уже другая трасса, не погрешность стыка. */
    private static final double WELD_M = 4.0;
    /** Врезка обязана лежать на существующей трубе или камере. */
    private static final double ON_EXISTING_M = 3.5;

    private SubmissionHygiene() {
    }

    public static void prepare(Variant variant, Scene scene) {
        if (variant == null) {
            return;
        }
        variant.segments.removeIf(s -> s == null || s.geometryMeters == null
                || s.geometryMeters.getNumPoints() < 2);
        dropDuplicateTraces(variant);
        weldToNodes(variant);
        dropPhantomTaps(variant, scene);
        dropUnused(variant);
    }

    /**
     * DN врезки — максимум DN труб, которые в неё приходят.
     * Чужой ствол на другом конце карты сюда не подмешивается: иначе
     * камера получает «требуемый» DN300 и фантомную реконструкцию.
     */
    public static void assignTapDiameters(Variant variant) {
        if (variant == null) {
            return;
        }
        for (TapPoint tap : variant.taps) {
            if (tap == null) {
                continue;
            }
            int req = 0;
            for (NewSegment seg : variant.segments) {
                if (incident(tap, seg)) {
                    req = Math.max(req, seg.dn);
                }
            }
            tap.requiredDiameter = req;
        }
    }

    private static void dropDuplicateTraces(Variant variant) {
        List<NewSegment> drop = new ArrayList<>();
        List<NewSegment> segs = variant.segments;
        for (int i = 0; i < segs.size(); i++) {
            NewSegment a = segs.get(i);
            if (a == null || drop.contains(a)) {
                continue;
            }
            for (int j = i + 1; j < segs.size(); j++) {
                NewSegment b = segs.get(j);
                if (b == null || drop.contains(b)) {
                    continue;
                }
                if (!sameEnds(a, b)) {
                    continue;
                }
                if (Math.abs(len(a) - len(b)) > 1.0) {
                    continue;
                }
                if (a.geometryMeters.distance(b.geometryMeters) > 1.2) {
                    continue;
                }
                drop.add(len(b) >= len(a) ? b : a);
                if (drop.get(drop.size() - 1) == a) {
                    break;
                }
            }
        }
        variant.segments.removeAll(drop);
    }

    private static boolean sameEnds(NewSegment a, NewSegment b) {
        if (a.fromId == null || a.toId == null || b.fromId == null || b.toId == null) {
            return false;
        }
        return (a.fromId.equals(b.fromId) && a.toId.equals(b.toId))
                || (a.fromId.equals(b.toId) && a.toId.equals(b.fromId));
    }

    /**
     * У каждого id узла одна точка. Концы участков, которые ссылаются на
     * этот id и стоят ближе {@link #WELD_M}, переезжают в неё.
     */
    private static void weldToNodes(Variant variant) {
        Map<String, Coordinate> anchor = new HashMap<>();
        for (TapPoint t : variant.taps) {
            if (t == null || t.geometryMeters == null) {
                continue;
            }
            Coordinate c = t.geometryMeters.getCoordinate();
            putAnchor(anchor, t.id, c);
            putAnchor(anchor, t.nodeId, c);
        }
        for (TechnicalNode n : variant.technicalNodes) {
            if (n != null && n.id != null && n.geometryMeters != null) {
                putAnchor(anchor, n.id, n.geometryMeters.getCoordinate());
            }
        }
        for (NewChamber ch : variant.chambers) {
            if (ch != null && ch.id != null && ch.geometryMeters != null) {
                putAnchor(anchor, ch.id, ch.geometryMeters.getCoordinate());
            }
        }
        agreeSharedEnds(variant, anchor);
        for (NewSegment seg : variant.segments) {
            if (seg.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = seg.geometryMeters.getCoordinates();
            if (pts.length < 2) {
                continue;
            }
            boolean moved = false;
            moved |= pull(pts, 0, anchor.get(seg.fromId));
            moved |= pull(pts, pts.length - 1, anchor.get(seg.toId));
            if (moved) {
                seg.geometryMeters = GeoJsonGeometries.GF.createLineString(pts);
                seg.lengthM = seg.geometryMeters.getLength();
            }
        }
    }

    /** Два участка с общим id, но без узла в списке: усреднить концы, если щель меньше 4 м. */
    private static void agreeSharedEnds(Variant variant, Map<String, Coordinate> anchor) {
        Map<String, List<Coordinate>> ends = new HashMap<>();
        for (NewSegment seg : variant.segments) {
            if (seg.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = seg.geometryMeters.getCoordinates();
            if (pts.length < 2) {
                continue;
            }
            if (seg.fromId != null && !anchor.containsKey(seg.fromId)) {
                ends.computeIfAbsent(seg.fromId, k -> new ArrayList<>()).add(pts[0]);
            }
            if (seg.toId != null && !anchor.containsKey(seg.toId)) {
                ends.computeIfAbsent(seg.toId, k -> new ArrayList<>()).add(pts[pts.length - 1]);
            }
        }
        for (Map.Entry<String, List<Coordinate>> e : ends.entrySet()) {
            List<Coordinate> pts = e.getValue();
            if (pts.size() < 2) {
                continue;
            }
            double max = 0;
            for (int i = 0; i < pts.size(); i++) {
                for (int j = i + 1; j < pts.size(); j++) {
                    max = Math.max(max, pts.get(i).distance(pts.get(j)));
                }
            }
            if (max < 0.25 || max > WELD_M) {
                continue;
            }
            double x = 0;
            double y = 0;
            for (Coordinate c : pts) {
                x += c.x;
                y += c.y;
            }
            anchor.put(e.getKey(), new Coordinate(x / pts.size(), y / pts.size()));
        }
    }

    private static boolean pull(Coordinate[] pts, int index, Coordinate anchor) {
        if (anchor == null || pts[index] == null) {
            return false;
        }
        double d = pts[index].distance(anchor);
        if (d < 0.2 || d > WELD_M) {
            return false;
        }
        pts[index] = new Coordinate(anchor);
        return true;
    }

    private static void putAnchor(Map<String, Coordinate> anchor, String id, Coordinate c) {
        if (id == null || id.isBlank() || c == null || anchor.containsKey(id)) {
            return;
        }
        anchor.put(id, new Coordinate(c));
    }

    /**
     * Врезка без трубы удаляется. Врезка, чья точка не на существующей сети,
     * тоже удаляется — иначе в сдаче висит tie_in, которого на теплотрассе нет.
     * Если труба кончается в 4 м от врезки, но id не совпал, конец пересаживается
     * на врезку: это тот же физический стык, не новая трасса.
     */
    private static void dropPhantomTaps(Variant variant, Scene scene) {
        List<TapPoint> drop = new ArrayList<>();
        for (TapPoint tap : variant.taps) {
            if (tap == null || tap.geometryMeters == null) {
                drop.add(tap);
                continue;
            }
            if (!referenced(variant, tap)) {
                if (!adoptDanglingEnd(variant, tap)) {
                    drop.add(tap);
                    continue;
                }
            }
            if (scene != null && !snapOntoExisting(variant, tap, scene)) {
                drop.add(tap);
            }
        }
        variant.taps.removeAll(drop);
    }

    /**
     * Точка врезки должна лежать на существующей теплотрассе.
     * Если она в пределах 8 м — переносим маркер и концы труб на ближайшую
     * точку сети. Дальше 8 м это не врезка, а висящая точка.
     */
    private static boolean snapOntoExisting(Variant variant, TapPoint tap, Scene scene) {
        Coordinate at = tap.geometryMeters.getCoordinate();
        if (onExisting(scene, at)) {
            return true;
        }
        Coordinate snapped = nearestExisting(scene, at, 8.0);
        if (snapped == null) {
            return false;
        }
        tap.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(snapped));
        for (NewSegment seg : variant.segments) {
            if (!incident(tap, seg) || seg.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = seg.geometryMeters.getCoordinates();
            if (pts.length < 2) {
                continue;
            }
            boolean moved = false;
            if (named(tap, seg.fromId) && pts[0].distance(snapped) <= 8) {
                pts[0] = new Coordinate(snapped);
                moved = true;
            }
            if (named(tap, seg.toId) && pts[pts.length - 1].distance(snapped) <= 8) {
                pts[pts.length - 1] = new Coordinate(snapped);
                moved = true;
            }
            if (moved) {
                seg.geometryMeters = GeoJsonGeometries.GF.createLineString(pts);
                seg.lengthM = seg.geometryMeters.getLength();
            }
        }
        return true;
    }

    private static Coordinate nearestExisting(Scene scene, Coordinate at, double cap) {
        Coordinate best = null;
        double bestD = cap;
        for (Chamber ch : scene.chambers) {
            if (ch.point == null) {
                continue;
            }
            double d = at.distance(ch.point.getCoordinate());
            if (d < bestD) {
                bestD = d;
                best = ch.point.getCoordinate();
            }
        }
        Geometry p = GeoJsonGeometries.GF.createPoint(at);
        for (ExistingSegment seg : scene.segments) {
            if (seg.line == null) {
                continue;
            }
            DistanceOp op = new DistanceOp(seg.line, p);
            double d = op.distance();
            if (d < bestD && op.nearestPoints().length > 0) {
                bestD = d;
                best = op.nearestPoints()[0];
            }
        }
        return best == null ? null : new Coordinate(best);
    }

    private static boolean adoptDanglingEnd(Variant variant, TapPoint tap) {
        Coordinate at = tap.geometryMeters.getCoordinate();
        String node = tap.nodeId != null ? tap.nodeId : tap.id;
        if (node == null) {
            return false;
        }
        NewSegment best = null;
        boolean bestFrom = false;
        double bestD = WELD_M;
        for (NewSegment seg : variant.segments) {
            if (seg.geometryMeters == null) {
                continue;
            }
            Coordinate[] pts = seg.geometryMeters.getCoordinates();
            if (degree(variant, seg.fromId) <= 1 && pts[0].distance(at) < bestD && !oksId(seg.fromId)) {
                bestD = pts[0].distance(at);
                best = seg;
                bestFrom = true;
            }
            if (degree(variant, seg.toId) <= 1 && pts[pts.length - 1].distance(at) < bestD && !oksId(seg.toId)) {
                bestD = pts[pts.length - 1].distance(at);
                best = seg;
                bestFrom = false;
            }
        }
        if (best == null) {
            return false;
        }
        Coordinate[] pts = best.geometryMeters.getCoordinates();
        if (bestFrom) {
            best.fromId = node;
            pts[0] = new Coordinate(at);
        } else {
            best.toId = node;
            pts[pts.length - 1] = new Coordinate(at);
        }
        best.geometryMeters = GeoJsonGeometries.GF.createLineString(pts);
        best.lengthM = best.geometryMeters.getLength();
        return true;
    }

    private static int degree(Variant variant, String id) {
        if (id == null) {
            return 0;
        }
        int n = 0;
        for (NewSegment seg : variant.segments) {
            if (id.equals(seg.fromId)) {
                n++;
            }
            if (id.equals(seg.toId)) {
                n++;
            }
        }
        return n;
    }

    private static boolean referenced(Variant variant, TapPoint tap) {
        for (NewSegment seg : variant.segments) {
            if (incident(tap, seg)) {
                return true;
            }
        }
        return false;
    }

    static boolean incident(TapPoint tap, NewSegment seg) {
        if (tap == null || seg == null) {
            return false;
        }
        return named(tap, seg.fromId) || named(tap, seg.toId);
    }

    private static boolean named(TapPoint tap, String id) {
        if (id == null) {
            return false;
        }
        return id.equals(tap.id) || id.equals(tap.nodeId) || id.equals(tap.existingObjectId);
    }

    private static boolean onExisting(Scene scene, Coordinate at) {
        if (at == null) {
            return false;
        }
        for (Chamber ch : scene.chambers) {
            if (ch.point != null && ch.point.getCoordinate().distance(at) <= ON_EXISTING_M) {
                return true;
            }
        }
        Geometry p = GeoJsonGeometries.GF.createPoint(at);
        for (ExistingSegment seg : scene.segments) {
            if (seg.line == null) {
                continue;
            }
            if (seg.line.distance(p) <= ON_EXISTING_M) {
                return true;
            }
            DistanceOp op = new DistanceOp(seg.line, p);
            if (op.distance() <= ON_EXISTING_M) {
                return true;
            }
        }
        return false;
    }

    private static void dropUnused(Variant variant) {
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
            if (t != null && t.nodeId != null) {
                used.add(t.nodeId);
            }
            if (t != null && t.id != null) {
                used.add(t.id);
            }
        }
        variant.technicalNodes.removeIf(n -> n == null || n.id == null || !used.contains(n.id));
        variant.chambers.removeIf(c -> c == null || c.id == null || !used.contains(c.id));
    }

    private static boolean oksId(String id) {
        if (id == null || id.isBlank()) {
            return false;
        }
        if (id.startsWith("TN-") || id.startsWith("CH-") || id.startsWith("TI-") || id.startsWith("NS-")) {
            return false;
        }
        try {
            int n = Integer.parseInt(id);
            return n > 0 && n < 100;
        } catch (NumberFormatException e) {
            return id.startsWith("OKS") || id.startsWith("oks");
        }
    }

    private static double len(NewSegment s) {
        if (s.lengthM > 0) {
            return s.lengthM;
        }
        return s.geometryMeters == null ? 0 : s.geometryMeters.getLength();
    }
}
