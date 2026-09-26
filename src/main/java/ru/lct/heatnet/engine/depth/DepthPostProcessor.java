package ru.lct.heatnet.engine.depth;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.depth.DepthProfile.Band;
import ru.lct.heatnet.engine.depth.DepthProfile.Choice;
import ru.lct.heatnet.engine.depth.DepthProfile.Cover;
import ru.lct.heatnet.engine.depth.DepthProfile.Edge;
import ru.lct.heatnet.engine.depth.DepthProfile.Knot;
import ru.lct.heatnet.engine.depth.DepthProfile.Pin;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

/**
 * Режим с глубиной, раздел 5 и вертикальные условия таблицы 2.
 * <p>
 * План не перестраивается: к готовой трассе назначается профиль. Обычная глубина до верха
 * габарита 3,0 м. Дорога и трамвай уже закрыты этой отметкой (требуется не меньше 1,0 и 1,2 м).
 * Газ, кабель и существующая сеть без врезки проходятся сверху, если хватает минимума 0,7 м,
 * иначе снизу. Уклон не круче 0,10. Шаг и потолок глубины не задаются.
 * Врезка в существующую сеть пересечением не считается.
 */
@Component
public class DepthPostProcessor {

    private static final Logger log = LoggerFactory.getLogger(DepthPostProcessor.class);

    /** Газопровод: 0,40×0,40 м, верх на 2,8 м, просвет 0,2 м. */
    private static final double GAS_TOP = 2.8;
    private static final double GAS_SIZE = 0.40;
    private static final double GAS_CLEAR = 0.20;
    /** Кабель до 35 кВ: 0,20×0,20 м, верх на 2,7 м, просвет 0,5 м. */
    private static final double CABLE_TOP = 2.7;
    private static final double CABLE_SIZE = 0.20;
    private static final double CABLE_CLEAR = 0.50;
    /** Существующая теплосеть: верх на 3,0 м, габарит по таблице 1, просвет 0,5 м. */
    private static final double HEAT_TOP = 3.0;
    private static final double HEAT_CLEAR = 0.50;
    private static final double ROAD_COVER = 1.0;
    private static final double TRAM_COVER = 1.2;
    /** Короче этого внутренний излом профиля отдельным узлом не режется. */
    private static final double MIN_PIECE_M = 0.05;

    public void apply(Variant variant, Scene scene, AppendixModel appendix) {
        if (variant == null || variant.segments.isEmpty() || appendix == null) {
            return;
        }
        AppendixModel.DepthSpec spec = appendix.getDepth();
        double ordinary = positive(spec.defaultDepthM, 3.0);
        double minDepth = positive(spec.minDepthM, 0.7);
        double slope = positive(spec.maxSlope, 0.10);
        double factor = positive(spec.costFactorPerMDepth, 0.10);
        // Кусок после нарезки чуть короче станции. Строим на 0,99 предела, чтобы опубликованный уклон не вышел за 0,10.
        double designSlope = slope * 0.99;

        List<NewSegment> source = new ArrayList<>(variant.segments);
        List<Edge> edges = new ArrayList<>();
        for (int i = 0; i < source.size(); i++) {
            NewSegment seg = source.get(i);
            double length = seg.geometryMeters != null ? seg.geometryMeters.getLength() : Math.max(0.0, seg.lengthM);
            String from = seg.fromId != null ? seg.fromId : "f" + i;
            String to = seg.toId != null ? seg.toId : "t" + i;
            edges.add(new Edge(from, to, length));
        }
        List<Pin> pins = new ArrayList<>();
        List<Cover> covers = new ArrayList<>();
        if (scene != null) {
            detect(variant, source, scene, appendix, minDepth, pins, covers);
        }
        for (Pin pin : pins) {
            Choice choice = DepthProfile.choose(ordinary, Math.max(minDepth, pin.floor), pin.bands, factor);
            pin.depth = choice.depth;
            pin.low = choice.low;
            pin.high = choice.high;
            pin.floor = Math.max(minDepth, pin.floor);
        }
        List<List<Knot>> profiles = DepthProfile.solve(edges, pins, covers, ordinary, minDepth, designSlope, factor);
        AtomicInteger ids = nextIds(variant);
        List<NewSegment> rebuilt = new ArrayList<>();
        for (int i = 0; i < source.size(); i++) {
            rebuilt.addAll(cut(variant, source.get(i), profiles.get(i), ids, ordinary));
        }
        variant.segments.clear();
        variant.segments.addAll(rebuilt);
        variant.notes.add("Глубина до верха габарита: обычно " + num(ordinary)
                + " м, не меньше " + num(minDepth) + " м, уклон не круче " + num(slope)
                + ". Kгл = 1 до обычной глубины; глубже Kгл = 1 + 0,10·(h − 3), на уклоне — среднее по концам.");
        log.info("Профиль глубины: участков {}, пересечений {}, покрытий {}",
                rebuilt.size(), pins.size(), covers.size());
    }

    private static void detect(Variant variant, List<NewSegment> segments, Scene scene, AppendixModel appendix,
                               double minDepth, List<Pin> pins, List<Cover> covers) {
        Set<String> networkIds = new HashSet<>();
        for (ExistingSegment existing : scene.segments) {
            if (existing.id != null) {
                networkIds.add(existing.id);
            }
        }
        for (int i = 0; i < segments.size(); i++) {
            NewSegment seg = segments.get(i);
            if (seg.geometryMeters == null || seg.geometryMeters.getNumPoints() < 2) {
                continue;
            }
            double ourHalf = halfWidth(appendix, seg.dn);
            double ourHeight = height(appendix, seg.dn);
            LineString line = seg.geometryMeters;
            for (ExistingSegment existing : scene.segments) {
                if (existing.line == null || existing.line.isEmpty()) {
                    continue;
                }
                addHeat(variant, pins, i, line, seg, existing.line, existing.dn, ourHalf, ourHeight, scene, appendix);
            }
            for (SpatialConstraint constraint : scene.constraints) {
                if (constraint.geometry == null || constraint.geometry.isEmpty()) {
                    continue;
                }
                String kind = kind(constraint.type);
                if ("heat".equals(kind) && networkIds.contains(constraint.id)) {
                    continue;
                }
                if ("gas".equals(kind)) {
                    addUtility(variant, pins, i, line, constraint.geometry, ourHalf, GAS_SIZE / 2.0,
                            above(GAS_TOP, GAS_CLEAR, ourHeight), below(GAS_TOP, GAS_SIZE, GAS_CLEAR), false, seg, scene);
                } else if ("cable".equals(kind)) {
                    addUtility(variant, pins, i, line, constraint.geometry, ourHalf, CABLE_SIZE / 2.0,
                            above(CABLE_TOP, CABLE_CLEAR, ourHeight), below(CABLE_TOP, CABLE_SIZE, CABLE_CLEAR), false, seg, scene);
                } else if ("heat".equals(kind)) {
                    addHeat(variant, pins, i, line, seg, constraint.geometry, 150, ourHalf, ourHeight, scene, appendix);
                } else if ("road".equals(kind)) {
                    addCover(covers, i, line, constraint.geometry, ourHalf, ROAD_COVER);
                } else if ("tram".equals(kind)) {
                    addCover(covers, i, line, constraint.geometry, ourHalf, TRAM_COVER);
                }
            }
        }
        for (Pin pin : pins) {
            double floor = minDepth;
            for (Cover cover : covers) {
                if (cover.edge == pin.edge && overlap(pin.s0, pin.s1, cover.s0, cover.s1)) {
                    floor = Math.max(floor, cover.minDepth);
                }
            }
            pin.floor = floor;
        }
    }

    private static void addHeat(Variant variant, List<Pin> pins, int edge, LineString line, NewSegment seg,
                                Geometry network, int dn, double ourHalf, double ourHeight, Scene scene,
                                AppendixModel appendix) {
        double theirHalf = halfWidth(appendix, dn);
        double theirHeight = height(appendix, dn);
        addUtility(variant, pins, edge, line, network, ourHalf, theirHalf,
                above(HEAT_TOP, HEAT_CLEAR, ourHeight), below(HEAT_TOP, theirHeight, HEAT_CLEAR),
                true, seg, scene);
    }

    private static void addUtility(Variant variant, List<Pin> pins, int edge, LineString line, Geometry obstacle,
                                   double ourHalf, double theirHalf, double aboveMax, double belowMin,
                                   boolean tieIn, NewSegment seg, Scene scene) {
        double reach = ourHalf + theirHalf;
        List<double[]> intervals = intervals(line, obstacle, reach);
        if (tieIn) {
            intervals = withoutTieIn(variant, line, seg, obstacle, intervals, reach + 0.5, scene);
        }
        for (double[] span : intervals) {
            Pin pin = new Pin(edge, span[0], span[1]);
            pin.bands.add(new Band(aboveMax, belowMin));
            pins.add(pin);
        }
    }

    private static void addCover(List<Cover> covers, int edge, LineString line, Geometry obstacle,
                                 double ourHalf, double minCover) {
        for (double[] span : intervals(line, obstacle, ourHalf)) {
            covers.add(new Cover(edge, span[0], span[1], minCover));
        }
    }

    /** Интервалы станции, где габариты в плане пересекаются. */
    static List<double[]> intervals(LineString line, Geometry obstacle, double pad) {
        List<double[]> raw = new ArrayList<>();
        if (line == null || obstacle == null || obstacle.isEmpty()) {
            return raw;
        }
        Geometry zone = obstacle;
        if (pad > 1e-6) {
            try {
                zone = obstacle.buffer(pad);
            } catch (RuntimeException e) {
                return raw;
            }
        }
        Geometry hit;
        try {
            hit = line.intersection(zone);
        } catch (RuntimeException e) {
            return raw;
        }
        double length = line.getLength();
        LengthIndexedLine indexed = new LengthIndexedLine(line);
        collect(hit, indexed, length, raw);
        raw.sort((a, b) -> Double.compare(a[0], b[0]));
        List<double[]> merged = new ArrayList<>();
        for (double[] span : raw) {
            double a = clamp(Math.min(span[0], span[1]), 0.0, length);
            double b = clamp(Math.max(span[0], span[1]), 0.0, length);
            if (merged.isEmpty() || a > merged.get(merged.size() - 1)[1] + MIN_PIECE_M) {
                merged.add(new double[]{a, b});
            } else {
                merged.get(merged.size() - 1)[1] = Math.max(merged.get(merged.size() - 1)[1], b);
            }
        }
        return merged;
    }

    private static void collect(Geometry geometry, LengthIndexedLine indexed, double length, List<double[]> out) {
        if (geometry == null || geometry.isEmpty()) {
            return;
        }
        if (geometry instanceof LineString) {
            LineString line = (LineString) geometry;
            if (line.getNumPoints() == 0) {
                return;
            }
            double a = indexed.indexOf(line.getCoordinateN(0));
            double b = indexed.indexOf(line.getCoordinateN(line.getNumPoints() - 1));
            out.add(new double[]{a, b});
            return;
        }
        if (geometry instanceof Point) {
            double station = indexed.indexOf(geometry.getCoordinate());
            out.add(new double[]{station, station});
            return;
        }
        for (int i = 0; i < geometry.getNumGeometries(); i++) {
            collect(geometry.getGeometryN(i), indexed, length, out);
        }
    }

    /** Подход к камере врезки — присоединение, не пересечение без врезки. */
    private static List<double[]> withoutTieIn(Variant variant, LineString line, NewSegment seg, Geometry network,
                                                List<double[]> intervals, double reach, Scene scene) {
        if (intervals.isEmpty()) {
            return intervals;
        }
        double length = line.getLength();
        boolean dropStart = joint(scene, seg.fromId, variant) && near(network, line.getCoordinateN(0), reach);
        boolean dropEnd = joint(scene, seg.toId, variant) && near(network, line.getCoordinateN(line.getNumPoints() - 1), reach);
        if (!dropStart && !dropEnd) {
            return intervals;
        }
        List<double[]> kept = new ArrayList<>();
        for (double[] span : intervals) {
            boolean atStart = span[0] <= 0.30;
            boolean atEnd = span[1] >= length - 0.30;
            if (atStart && dropStart) {
                continue;
            }
            if (atEnd && dropEnd) {
                continue;
            }
            kept.add(span);
        }
        return kept;
    }

    private static boolean joint(Scene scene, String id) {
        if (id == null || scene == null) {
            return false;
        }
        for (Chamber chamber : scene.chambers) {
            if (id.equals(chamber.id)) {
                return true;
            }
        }
        return false;
    }

    private static boolean near(Geometry geometry, Coordinate coordinate, double reach) {
        if (geometry == null || coordinate == null) {
            return false;
        }
        return geometry.distance(GeoJsonGeometries.GF.createPoint(coordinate)) <= reach + 1e-6;
    }

    private static List<NewSegment> cut(Variant variant, NewSegment seg, List<Knot> knots, AtomicInteger ids, double ordinary) {
        List<NewSegment> single = new ArrayList<>();
        single.add(seg);
        if (knots == null || knots.isEmpty()) {
            seg.depthFrom = ordinary;
            seg.depthTo = ordinary;
            seg.depthM = null;
            return single;
        }
        if (seg.geometryMeters == null || seg.geometryMeters.getNumPoints() < 2 || knots.size() < 2) {
            seg.depthFrom = knots.get(0).depth;
            seg.depthTo = knots.get(knots.size() - 1).depth;
            seg.depthM = null;
            return single;
        }
        List<Knot> use = keep(knots, ordinary);
        seg.depthM = null;
        if (use.size() < 3) {
            seg.depthFrom = use.get(0).depth;
            seg.depthTo = use.get(use.size() - 1).depth;
            return single;
        }
        LineString original = seg.geometryMeters;
        String originalFrom = seg.fromId;
        String originalTo = seg.toId;
        double length = original.getLength();
        LengthIndexedLine indexed = new LengthIndexedLine(original);
        LineString[] slices = new LineString[use.size() - 1];
        for (int k = 1; k < use.size(); k++) {
            double a = k == 1 ? 0.0 : use.get(k - 1).station;
            double b = k == use.size() - 1 ? length : use.get(k).station;
            slices[k - 1] = slice(indexed, a, b);
            if (slices[k - 1] == null) {
                seg.depthFrom = use.get(0).depth;
                seg.depthTo = use.get(use.size() - 1).depth;
                return single;
            }
        }
        List<NewSegment> pieces = new ArrayList<>();
        String fromId = originalFrom;
        for (int k = 1; k < use.size(); k++) {
            boolean last = k == use.size() - 1;
            String toId = last ? originalTo : nodeAt(variant, indexed.extractPoint(use.get(k).station), ids);
            NewSegment piece = k == 1 ? seg : copy(seg, ids);
            piece.geometryMeters = slices[k - 1];
            piece.lengthM = slices[k - 1].getLength();
            piece.depthFrom = use.get(k - 1).depth;
            piece.depthTo = use.get(k).depth;
            piece.depthM = null;
            piece.fromId = fromId;
            piece.toId = toId;
            pieces.add(piece);
            fromId = toId;
        }
        return pieces;
    }

    private static List<Knot> keep(List<Knot> knots, double ordinary) {
        List<Knot> use = new ArrayList<>();
        use.add(knots.get(0));
        for (int i = 1; i < knots.size() - 1; i++) {
            Knot cur = knots.get(i);
            Knot prev = use.get(use.size() - 1);
            boolean crossing = Math.abs(cur.depth - ordinary) <= 0.002
                    && (prev.depth - ordinary) * (knots.get(i + 1).depth - ordinary) < -1e-8;
            if (cur.station - prev.station < MIN_PIECE_M && !crossing) {
                continue;
            }
            use.add(cur);
        }
        Knot last = knots.get(knots.size() - 1);
        if (last.station - use.get(use.size() - 1).station < 1e-4 && use.size() > 1) {
            use.remove(use.size() - 1);
        }
        use.add(last);
        return use;
    }

    private static LineString slice(LengthIndexedLine indexed, double from, double to) {
        if (to - from < 1e-4) {
            return null;
        }
        Geometry geometry = indexed.extractLine(from, to);
        if (geometry instanceof LineString) {
            LineString line = (LineString) geometry;
            if (line.getNumPoints() >= 2 && line.getLength() >= 1e-4) {
                return line;
            }
        }
        if (geometry != null) {
            for (int i = 0; i < geometry.getNumGeometries(); i++) {
                Geometry part = geometry.getGeometryN(i);
                if (part instanceof LineString && part.getNumPoints() >= 2 && part.getLength() >= 1e-4) {
                    return (LineString) part;
                }
            }
        }
        return null;
    }

    private static NewSegment copy(NewSegment seg, AtomicInteger ids) {
        NewSegment copy = new NewSegment();
        copy.id = "NS-" + ids.getAndIncrement();
        copy.flowTph = seg.flowTph;
        copy.dn = seg.dn;
        copy.layingMethod = seg.layingMethod;
        copy.kSpec = seg.kSpec;
        copy.specialReason = seg.specialReason;
        copy.parentId = seg.parentId;
        return copy;
    }

    private static String nodeAt(Variant variant, Coordinate coordinate, AtomicInteger ids) {
        if (coordinate != null) {
            for (TechnicalNode node : variant.technicalNodes) {
                if (node.geometryMeters != null && node.geometryMeters.getCoordinate().distance(coordinate) <= MIN_PIECE_M) {
                    return node.id;
                }
            }
            for (NewChamber chamber : variant.chambers) {
                if (chamber.geometryMeters != null && chamber.geometryMeters.getCoordinate().distance(coordinate) <= MIN_PIECE_M) {
                    return chamber.id;
                }
            }
        }
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-" + ids.getAndIncrement();
        node.geometryMeters = GeoJsonGeometries.GF.createPoint(coordinate == null ? new Coordinate() : coordinate);
        node.reason = "профиль глубины";
        variant.technicalNodes.add(node);
        return node.id;
    }

    private static boolean overlap(double a0, double a1, double b0, double b1) {
        return Math.min(a1, b1) + 1e-6 >= Math.max(a0, b0);
    }

    private static double above(double top, double clearance, double ourHeight) {
        return top - clearance - ourHeight;
    }

    private static double below(double top, double theirHeight, double clearance) {
        return top + theirHeight + clearance;
    }

    private static double halfWidth(AppendixModel appendix, int dn) {
        AppendixModel.DiameterSpec spec = spec(appendix, dn);
        if (spec == null || spec.widthM <= 0) {
            return 0.25;
        }
        return spec.widthM / 2.0;
    }

    private static double height(AppendixModel appendix, int dn) {
        AppendixModel.DiameterSpec spec = spec(appendix, dn);
        if (spec == null || spec.heightM <= 0) {
            return 0.25;
        }
        return spec.heightM;
    }

    private static AppendixModel.DiameterSpec spec(AppendixModel appendix, int dn) {
        AppendixModel.DiameterSpec exact = appendix.diameter(dn);
        if (exact != null) {
            return exact;
        }
        AppendixModel.DiameterSpec nearest = null;
        int gap = Integer.MAX_VALUE;
        for (AppendixModel.DiameterSpec candidate : appendix.getDiameters()) {
            int delta = Math.abs(candidate.dn - dn);
            if (delta < gap) {
                gap = delta;
                nearest = candidate;
            }
        }
        return nearest;
    }

    static String kind(String type) {
        if (type == null || type.isBlank()) {
            return "";
        }
        String name = type.toLowerCase(Locale.ROOT).replace('ё', 'е');
        if (name.contains("rail") || name.contains("желез")) {
            return "railway";
        }
        if (name.contains("tram") || name.contains("трам")) {
            return "tram";
        }
        if (name.contains("gas") || name.contains("газ")) {
            return "gas";
        }
        if (name.contains("cable") || name.contains("кабел") || name.contains("power")) {
            return "cable";
        }
        if (name.contains("heat") || name.contains("тепл")) {
            return "heat";
        }
        if (name.contains("tdtp") || name.contains("тдтп") || name.contains("carriage") || name.contains("проез")) {
            return "road";
        }
        if (name.contains("road") || name.contains("street") || name.contains("улиц") || name.contains("автомоб")
                || name.contains("дорог")) {
            return "road";
        }
        return "";
    }

    private static boolean joint(Scene scene, String id, Variant variant) {
        if (id == null) {
            return false;
        }
        if (variant != null) {
            for (NewChamber chamber : variant.chambers) {
                if (id.equals(chamber.id)) {
                    return true;
                }
            }
            for (TapPoint tap : variant.taps) {
                if (id.equals(tap.nodeId) || id.equals(tap.id) || id.equals(tap.existingObjectId)) {
                    return true;
                }
            }
        }
        return joint(scene, id);
    }

    private static double positive(double value, double fallback) {
        return value > 0 ? value : fallback;
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    private static String num(double value) {
        return String.format(Locale.ROOT, "%.2f", value).replace('.', ',');
    }

    private static AtomicInteger nextIds(Variant variant) {
        int max = 0;
        for (NewSegment seg : variant.segments) {
            max = Math.max(max, tail(seg.id));
        }
        for (NewChamber chamber : variant.chambers) {
            max = Math.max(max, tail(chamber.id));
        }
        for (TechnicalNode node : variant.technicalNodes) {
            max = Math.max(max, tail(node.id));
        }
        for (TapPoint tap : variant.taps) {
            max = Math.max(max, tail(tap.id));
        }
        return new AtomicInteger(max + 1);
    }

    private static int tail(String id) {
        if (id == null) {
            return 0;
        }
        int i = id.length() - 1;
        while (i >= 0 && Character.isDigit(id.charAt(i))) {
            i--;
        }
        if (i == id.length() - 1) {
            return 0;
        }
        try {
            return Integer.parseInt(id.substring(i + 1));
        } catch (NumberFormatException e) {
            return 0;
        }
    }
}
