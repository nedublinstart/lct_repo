package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
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
import ru.lct.heatnet.engine.greedy.OrthoPaths;
import ru.lct.heatnet.engine.greedy.PathSmoother;
import ru.lct.heatnet.engine.greedy.PipeEmitter;
import ru.lct.heatnet.geo.GeoJsonGeometries;

/**
 * После Mehlhorn: склеить степень-2 (лишние узлы) и выкинуть вершину, если
 * хорда/П короче. Новые камеры не ставим.
 */
public final class ForestCompactor {

    private ForestCompactor() {
    }

    public static void compact(Variant variant, ObstacleIndex obstacles, AtomicInteger ids) {
        if (variant == null || variant.segments.isEmpty() || obstacles == null) {
            return;
        }
        polishSegments(variant, obstacles);
        for (int round = 0; round < 12; round++) {
            if (!mergeDegree2(variant, obstacles, ids)) {
                break;
            }
        }
        dropUnused(variant);
    }

    private static void polishSegments(Variant variant, ObstacleIndex obstacles) {
        for (NewSegment s : variant.segments) {
            List<Coordinate> pts = coords(s);
            if (pts.size() < 3) {
                continue;
            }
            List<Coordinate> slim = PathSmoother.emitPolish(pts, obstacles);
            if (slim == null || slim.size() < 2) {
                continue;
            }
            double old = OrthoPaths.length(pts);
            double neu = OrthoPaths.length(slim);
            if (neu + 0.8 >= old && slim.size() >= pts.size()) {
                continue;
            }
            s.geometryMeters = GeoJsonGeometries.GF.createLineString(
                    slim.toArray(new Coordinate[0]));
            s.lengthM = s.geometryMeters.getLength();
        }
    }

    private static boolean mergeDegree2(Variant variant, ObstacleIndex obstacles, AtomicInteger ids) {
        Set<String> frozen = frozen(variant);
        Map<String, Integer> deg = new HashMap<>();
        Map<String, List<NewSegment>> at = new HashMap<>();
        for (NewSegment s : variant.segments) {
            if (s == null) {
                continue;
            }
            if (s.fromId != null) {
                deg.merge(s.fromId, 1, Integer::sum);
                at.computeIfAbsent(s.fromId, k -> new ArrayList<>()).add(s);
            }
            if (s.toId != null) {
                deg.merge(s.toId, 1, Integer::sum);
                at.computeIfAbsent(s.toId, k -> new ArrayList<>()).add(s);
            }
        }
        for (Map.Entry<String, Integer> e : deg.entrySet()) {
            String tn = e.getKey();
            if (tn == null || !tn.startsWith("TN-") || e.getValue() != 2 || frozen.contains(tn)) {
                continue;
            }
            List<NewSegment> inc = at.getOrDefault(tn, List.of());
            if (inc.size() != 2) {
                continue;
            }
            NewSegment in = null;
            NewSegment out = null;
            for (NewSegment s : inc) {
                if (tn.equals(s.toId)) {
                    in = s;
                }
                if (tn.equals(s.fromId)) {
                    out = s;
                }
            }
            if (in == null || out == null || in == out || in.fromId == null || out.toId == null) {
                continue;
            }
            if (in.fromId.equals(out.toId)) {
                continue;
            }
            String la = in.layingMethod == null ? "base" : in.layingMethod;
            String lb = out.layingMethod == null ? "base" : out.layingMethod;
            if (!la.equals(lb)) {
                continue;
            }
            List<Coordinate> path = concat(in, out);
            if (path.size() < 2) {
                continue;
            }
            path = PathSmoother.emitPolish(path, obstacles);
            if (path == null || path.size() < 2) {
                continue;
            }
            double old = (in.lengthM > 0 ? in.lengthM : OrthoPaths.length(coords(in)))
                    + (out.lengthM > 0 ? out.lengthM : OrthoPaths.length(coords(out)));
            double neu = OrthoPaths.length(path);
            if (neu > old + 4) {
                continue;
            }
            variant.segments.remove(in);
            variant.segments.remove(out);
            PipeEmitter.emit(variant, obstacles, ids, in.fromId, out.toId,
                    Math.max(in.flowTph, out.flowTph), path);
            dropUnused(variant);
            return true;
        }
        return false;
    }

    private static Set<String> frozen(Variant variant) {
        Set<String> ids = new HashSet<>();
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
        }
        for (NewChamber c : variant.chambers) {
            if (c != null && c.id != null) {
                ids.add(c.id);
            }
        }
        return ids;
    }

    private static List<Coordinate> concat(NewSegment a, NewSegment b) {
        List<Coordinate> out = coords(a);
        List<Coordinate> rest = coords(b);
        int start = 0;
        if (!out.isEmpty() && !rest.isEmpty() && out.get(out.size() - 1).distance(rest.get(0)) < 1.2) {
            start = 1;
        }
        for (int i = start; i < rest.size(); i++) {
            out.add(rest.get(i));
        }
        return out;
    }

    private static List<Coordinate> coords(NewSegment s) {
        List<Coordinate> out = new ArrayList<>();
        if (s == null || s.geometryMeters == null) {
            return out;
        }
        for (Coordinate c : s.geometryMeters.getCoordinates()) {
            if (c != null) {
                out.add(new Coordinate(c));
            }
        }
        return out;
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
        variant.technicalNodes.removeIf(n -> n == null || n.id == null || !used.contains(n.id));
        variant.chambers.removeIf(c -> {
            if (c == null || c.id == null) {
                return true;
            }
            if (used.contains(c.id)) {
                return false;
            }
            for (TapPoint t : variant.taps) {
                if (c.id.equals(t.nodeId) || c.id.equals(t.id)) {
                    return false;
                }
            }
            return true;
        });
        variant.taps.removeIf(t -> {
            if (t == null) {
                return true;
            }
            String n = t.nodeId != null ? t.nodeId : t.id;
            return (n == null || !used.contains(n)) && (t.id == null || !used.contains(t.id));
        });
    }
}
