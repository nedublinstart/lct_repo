package ru.lct.heatnet.engine.greedy;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.locationtech.jts.algorithm.MinimumDiameter;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.GeometryCollection;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.MultiLineString;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.geom.prep.PreparedGeometry;
import org.locationtech.jts.geom.prep.PreparedGeometryFactory;
import org.locationtech.jts.index.strtree.STRtree;
import org.locationtech.jts.linearref.LengthIndexedLine;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.SpatialConstraint;

/**
 * Дороги, трамвай, ТДТП: вдоль проезжей нельзя, пересечение ≥ 45° и только спецметодом
 * на самом проходе. Если полигонов дорог во входе нет — проезжая синтезируется из щелей
 * между кварталами (тротуары у фасадов остаются обычной прокладкой).
 */
public final class SpecialLayer {

    private static final double DENSIFY_M = 2.0;
    private static final double MIN_PIECE_M = 0.35;
    private static final double LONG_ALONG_MUL = 1.85;

    private final GeometryFactory gf = GeoJsonGeometries.GF;
    private final List<Band> bands = new ArrayList<>();
    private final STRtree tree = new STRtree();
    private final double grazeM;
    private final double sidewalkM;
    private final double streetMinM;
    private final double streetMaxM;
    private final double maxStreetEdgeM;
    private final double maxOpenEdgeM;
    private final double alongPenalty;
    private final AppendixModel.ConstraintSpec roadRule;
    private boolean built;

    SpecialLayer(AppendixModel appendix) {
        AppendixModel.RoutingSpec r = appendix == null ? new AppendixModel.RoutingSpec() : appendix.getRouting();
        this.grazeM = r.grazeM;
        this.sidewalkM = r.sidewalkM;
        this.streetMinM = r.streetMinM;
        this.streetMaxM = r.streetMaxM;
        this.maxStreetEdgeM = r.maxStreetEdgeM;
        this.maxOpenEdgeM = r.maxOpenEdgeM;
        this.alongPenalty = r.alongRoadPenalty;
        this.roadRule = appendix == null ? roadFallback() : appendix.constraintRule("road");
    }

    static SpecialLayer empty() {
        SpecialLayer layer = new SpecialLayer(null);
        layer.finish();
        return layer;
    }

    void addExplicit(Geometry geometry, SpatialConstraint raw) {
        if (geometry == null || geometry.isEmpty() || raw == null || raw.rule == null) {
            return;
        }
        if (!raw.rule.special() && !raw.rule.cross()) {
            return;
        }
        addGeometry(geometry, raw, roadLike(raw.type) || angled(raw.rule), false);
    }

    void inferFromBlocks(List<Polygon> blocks) {
        if (blocks == null || blocks.size() < 2) {
            return;
        }
        int n = blocks.size();
        SpatialConstraint synthetic = new SpatialConstraint();
        synthetic.id = "inferred-road";
        synthetic.type = "road";
        synthetic.rule = roadRule;
        STRtree tree = new STRtree();
        for (int i = 0; i < n; i++) {
            tree.insert(blocks.get(i).getEnvelopeInternal(), i);
        }
        tree.build();
        for (int i = 0; i < n; i++) {
            Polygon a = blocks.get(i);
            Envelope q = new Envelope(a.getEnvelopeInternal());
            q.expandBy(streetMaxM + 2);
            @SuppressWarnings("unchecked")
            List<Integer> near = tree.query(q);
            if (near == null) {
                continue;
            }
            for (Integer j : near) {
                if (j == null || j <= i) {
                    continue;
                }
                Polygon b = blocks.get(j);
                if (a.getEnvelopeInternal().distance(b.getEnvelopeInternal()) > streetMaxM + 2) {
                    continue;
                }
                Geometry corridor = corridorBetween(a, b);
                if (corridor == null || corridor.isEmpty()) {
                    continue;
                }
                Envelope ce = corridor.getEnvelopeInternal();
                @SuppressWarnings("unchecked")
                List<Integer> cutHits = tree.query(ce);
                if (cutHits != null) {
                    for (Integer k : cutHits) {
                        if (k == null || k == i || k == j || corridor.isEmpty()) {
                            continue;
                        }
                        Polygon other = blocks.get(k);
                        try {
                            if (corridor.intersects(other)) {
                                Geometry cut = corridor.difference(other);
                                if (cut != null && !cut.isEmpty()) {
                                    corridor = cut;
                                }
                            }
                        } catch (RuntimeException ignored) {
                        }
                    }
                }
                addGeometry(corridor, synthetic, true, true);
            }
        }
    }

    void finish() {
        if (built) {
            return;
        }
        for (Band band : bands) {
            tree.insert(band.geom.getEnvelopeInternal(), band);
        }
        tree.build();
        built = true;
    }

    public double maxStreetEdgeM() {
        return maxStreetEdgeM;
    }

    public double maxOpenEdgeM() {
        return maxOpenEdgeM;
    }

    public double sidewalkM() {
        return sidewalkM;
    }

    public boolean isEmpty() {
        return bands.isEmpty();
    }

    public Travel inspect(Coordinate a, Coordinate b) {
        double len = a == null || b == null ? 0 : a.distance(b);
        if (a == null || b == null) {
            return Travel.blocked(len);
        }
        if (len < 1e-6) {
            return Travel.free(0);
        }
        if (bands.isEmpty()) {
            return Travel.free(len);
        }
        LineString ls = line(a, b);
        List<Band> near = query(ls.getEnvelopeInternal());
        if (near.isEmpty()) {
            return Travel.free(len);
        }
        double hitM = 0;
        double bestAng = 0;
        double minNeed = 45;
        double maxWidth = 0;
        double k = 1.0;
        double extend = 0;
        String reason = null;
        boolean anySpecial = false;
        boolean anyAngle = false;
        for (Band band : near) {
            if (!band.prepared.intersects(ls)) {
                continue;
            }
            double hit = hitLength(ls, band.geom);
            boolean nick = hit < grazeM && (len - hit) >= Math.max(0.8, 0.35 * len);
            if (nick) {
                continue;
            }
            anySpecial = true;
            hitM += hit;
            k = Math.max(k, band.kSpec);
            extend = Math.max(extend, band.extendM);
            if (reason == null || band.kSpec >= k) {
                reason = band.type;
            }
            if (band.angleSensitive) {
                anyAngle = true;
                minNeed = Math.max(minNeed, band.minAngleDeg);
                maxWidth = Math.max(maxWidth, band.widthM);
                bestAng = Math.max(bestAng, crossingAngleDeg(a, b, band.axis));
            }
        }
        if (!anySpecial) {
            return Travel.free(len);
        }
        hitM = Math.min(len, hitM);
        double extra = Math.max(0, k - 1.0) * hitM;
        double cost = len + extra;
        if (!anyAngle) {
            return Travel.ok(len, cost, true, k, reason, extend, hitM, 90);
        }
        if (maxWidth < 1) {
            maxWidth = Math.max(hitM, grazeM);
        }
        boolean longAlong = hitM > Math.max(grazeM * 2.0, maxWidth * LONG_ALONG_MUL);
        if (longAlong && bestAng < 70) {
            return Travel.blocked(len);
        }
        if (bestAng + 1e-6 < minNeed) {
            return Travel.blocked(len);
        }
        return Travel.ok(len, cost, true, k, reason, extend, hitM, bestAng);
    }

    public boolean allows(Coordinate a, Coordinate b) {
        return inspect(a, b).allowed;
    }

    public double travelCost(Coordinate a, Coordinate b) {
        Travel t = inspect(a, b);
        return t.allowed ? t.cost : Double.POSITIVE_INFINITY;
    }

    public double stepMultiplier(Coordinate a, Coordinate b) {
        if (a == null || b == null || bands.isEmpty()) {
            return 1.0;
        }
        LineString ls = line(a, b);
        List<Band> near = query(ls.getEnvelopeInternal());
        if (near.isEmpty()) {
            return 1.0;
        }
        Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        org.locationtech.jts.geom.Point midP = gf.createPoint(mid);
        double mul = 1.0;
        for (Band band : near) {
            if (!band.prepared.intersects(ls)) {
                continue;
            }
            if (band.angleSensitive) {
                double ang = crossingAngleDeg(a, b, band.axis);
                boolean inside = band.prepared.covers(midP);
                if (inside && ang < 70) {
                    return alongPenalty;
                }
                if (ang + 1e-6 < band.minAngleDeg) {
                    return alongPenalty;
                }
            }
            mul = Math.max(mul, Math.max(1.05, band.kSpec));
        }
        return mul;
    }

    public boolean inRoad(Coordinate c) {
        if (c == null || bands.isEmpty()) {
            return false;
        }
        org.locationtech.jts.geom.Point p = gf.createPoint(c);
        for (Band band : query(p.getEnvelopeInternal())) {
            if (band.angleSensitive && band.prepared.covers(p)) {
                return true;
            }
        }
        return false;
    }

    public int extraAt(Coordinate c) {
        if (c == null || bands.isEmpty()) {
            return 0;
        }
        org.locationtech.jts.geom.Point p = gf.createPoint(c);
        int sum = 0;
        for (Band band : query(p.getEnvelopeInternal())) {
            if (band.prepared.intersects(p) || band.geom.distance(p) < 0.5) {
                sum += Math.max(1, band.extraGrid);
            }
        }
        return sum;
    }

    public List<Piece> splitByTransport(List<Coordinate> path) {
        if (path == null || path.size() < 2) {
            return List.of();
        }
        if (bands.isEmpty()) {
            return List.of(Piece.base(path));
        }
        LineString sparse = lineOf(path);
        if (sparse == null || sparse.getLength() < MIN_PIECE_M) {
            return List.of(Piece.base(path));
        }
        List<Coordinate> dense = densify(path, DENSIFY_M);
        LineString ls = lineOf(dense);
        if (ls == null || ls.getLength() < MIN_PIECE_M) {
            return List.of(Piece.base(path));
        }
        LengthIndexedLine lil = new LengthIndexedLine(sparse);
        double total = ls.getLength();
        List<Run> runs = new ArrayList<>();
        Run cur = null;
        for (int i = 0; i < dense.size() - 1; i++) {
            Travel t = inspect(dense.get(i), dense.get(i + 1));
            boolean spec = t.allowed && t.special && t.hitM >= grazeM * 0.5;
            if (!t.allowed && t.special) {
                spec = true;
            }
            if (!t.allowed && !t.special && t.hitM >= grazeM) {
                spec = true;
            }
            String specReason;
            if (spec && (t.reason == null || t.reason.isBlank())) {
                specReason = "road";
            } else if (spec) {
                specReason = t.reason;
            } else {
                specReason = null;
            }
            if (cur == null || cur.special != spec) {
                cur = new Run();
                cur.special = spec;
                cur.start = accLength(dense, i);
                cur.kSpec = spec ? Math.max(1.0, t.kSpec) : 1.0;
                cur.reason = specReason;
                cur.extendM = spec ? t.extendM : 0;
                runs.add(cur);
            } else if (spec) {
                cur.kSpec = Math.max(cur.kSpec, t.kSpec);
                cur.extendM = Math.max(cur.extendM, t.extendM);
                if (specReason != null) {
                    cur.reason = specReason;
                }
            }
            cur.end = accLength(dense, i + 1);
        }
        if (runs.isEmpty()) {
            return List.of(Piece.base(path));
        }
        for (Run run : runs) {
            if (run.special && run.extendM > 0) {
                run.start = Math.max(0, run.start - run.extendM);
                run.end = Math.min(total, run.end + run.extendM);
            }
        }
        List<Run> merged = new ArrayList<>();
        for (Run run : runs) {
            if (merged.isEmpty()) {
                merged.add(run);
                continue;
            }
            Run last = merged.get(merged.size() - 1);
            if (last.special == run.special && run.start <= last.end + 0.2) {
                last.end = Math.max(last.end, run.end);
                last.kSpec = Math.max(last.kSpec, run.kSpec);
                last.extendM = Math.max(last.extendM, run.extendM);
                if (run.reason != null) {
                    last.reason = run.reason;
                }
            } else {
                merged.add(run);
            }
        }
        List<Piece> pieces = new ArrayList<>();
        double cursor = 0;
        for (Run run : merged) {
            if (run.start > cursor + MIN_PIECE_M) {
                pieces.add(extract(lil, cursor, run.start, false, 1.0, null));
            }
            pieces.add(extract(lil, Math.max(cursor, run.start), run.end, run.special, run.kSpec, run.reason));
            cursor = Math.max(cursor, run.end);
        }
        if (total - cursor > MIN_PIECE_M) {
            pieces.add(extract(lil, cursor, total, false, 1.0, null));
        }
        List<Piece> compact = new ArrayList<>();
        for (Piece piece : pieces) {
            if (piece == null || piece.lengthM < MIN_PIECE_M) {
                continue;
            }
            if (!compact.isEmpty()) {
                Piece last = compact.get(compact.size() - 1);
                if (last.special == piece.special) {
                    last.coords.addAll(piece.coords.subList(1, piece.coords.size()));
                    last.lengthM += piece.lengthM;
                    last.kSpec = Math.max(last.kSpec, piece.kSpec);
                    continue;
                }
            }
            compact.add(piece);
        }
        return compact.isEmpty() ? List.of(Piece.base(path)) : compact;
    }

    public static double crossingAngleDeg(Coordinate a, Coordinate b, Coordinate axis) {
        double sx = b.x - a.x;
        double sy = b.y - a.y;
        double sn = Math.hypot(sx, sy);
        if (sn < 1e-9 || axis == null) {
            return 90;
        }
        double an = Math.hypot(axis.x, axis.y);
        if (an < 1e-9) {
            return 90;
        }
        double dot = Math.abs((sx * axis.x + sy * axis.y) / (sn * an));
        dot = Math.max(0, Math.min(1, dot));
        return Math.toDegrees(Math.acos(dot));
    }

    static boolean roadLike(String type) {
        String n = type == null ? "" : type.toLowerCase(Locale.ROOT);
        return n.contains("road") || n.contains("tdtp") || n.contains("тдтп")
                || n.contains("tram") || n.contains("трам") || n.contains("проезж")
                || n.contains("carriage") || n.contains("дорог") || n.contains("улиц")
                || n.contains("inferred");
    }

    private void addGeometry(Geometry geometry, SpatialConstraint raw, boolean angleSensitive, boolean inferred) {
        if (geometry == null || geometry.isEmpty()) {
            return;
        }
        List<Polygon> parts = new ArrayList<>();
        ObstacleIndex.collectPolygons(geometry, parts);
        for (Polygon part : parts) {
            if (part == null || part.isEmpty() || part.getArea() < 40) {
                continue;
            }
            Band band = bandOf(part, raw, angleSensitive, inferred);
            if (band != null) {
                bands.add(band);
            }
        }
    }

    private Band bandOf(Polygon part, SpatialConstraint raw, boolean angleSensitive, boolean inferred) {
        Coordinate axis = new Coordinate(1, 0);
        double width = 8;
        double length = 8;
        try {
            Geometry rect = new MinimumDiameter(part).getMinimumRectangle();
            Coordinate[] c = rect.getCoordinates();
            double best = -1;
            double shortest = Double.POSITIVE_INFINITY;
            for (int i = 0; i < c.length - 1; i++) {
                double dx = c[i + 1].x - c[i].x;
                double dy = c[i + 1].y - c[i].y;
                double n = Math.hypot(dx, dy);
                if (n > best) {
                    best = n;
                    if (n > 1e-6) {
                        axis = new Coordinate(dx / n, dy / n);
                    }
                }
                if (n > 1e-6 && n < shortest) {
                    shortest = n;
                }
            }
            length = Math.max(best, 0);
            width = shortest < Double.POSITIVE_INFINITY ? shortest : Math.sqrt(part.getArea());
        } catch (RuntimeException ignored) {
        }
        if (inferred && length < 12) {
            return null;
        }
        if (inferred && width > 25 && length / Math.max(width, 1) < 2.2) {
            return null;
        }
        Band band = new Band();
        band.geom = part;
        band.prepared = PreparedGeometryFactory.prepare(part);
        band.axis = axis;
        band.widthM = Math.max(1.0, width);
        band.type = raw.type == null ? "road" : raw.type;
        AppendixModel.ConstraintSpec rule = raw.rule;
        band.minAngleDeg = rule.minAngleDeg == null ? 0 : rule.minAngleDeg;
        band.kSpec = rule.kSpec > 0 ? rule.kSpec : 1.0;
        band.extendM = rule.extendM;
        band.extraGrid = Math.max(1, rule.extraGridCost);
        band.angleSensitive = angleSensitive && band.minAngleDeg > 0;
        return band;
    }

    private Geometry corridorBetween(Polygon a, Polygon b) {
        double d;
        try {
            d = a.distance(b);
        } catch (RuntimeException e) {
            return null;
        }
        if (d < streetMinM || d > streetMaxM) {
            return null;
        }
        try {
            Geometry hull = a.union(b).convexHull();
            Geometry gap = hull.difference(a);
            gap = gap.difference(b);
            if (gap == null || gap.isEmpty() || gap.getArea() < 50) {
                return null;
            }
            Geometry carriage = gap.buffer(-sidewalkM, 8);
            if (carriage == null || carriage.isEmpty()) {
                carriage = gap.buffer(-Math.min(1.2, sidewalkM * 0.35), 8);
            }
            if (carriage == null || carriage.isEmpty()) {
                return null;
            }
            return largestPolygon(carriage);
        } catch (RuntimeException e) {
            return null;
        }
    }

    private Geometry largestPolygon(Geometry geometry) {
        List<Polygon> parts = new ArrayList<>();
        ObstacleIndex.collectPolygons(geometry, parts);
        if (parts.isEmpty()) {
            return null;
        }
        Polygon best = parts.get(0);
        for (Polygon p : parts) {
            if (p.getArea() > best.getArea()) {
                best = p;
            }
        }
        return best;
    }

    @SuppressWarnings("unchecked")
    private List<Band> query(Envelope env) {
        List<Band> hits = tree.query(env);
        return hits == null ? List.of() : hits;
    }

    private LineString line(Coordinate a, Coordinate b) {
        return gf.createLineString(new Coordinate[]{new Coordinate(a), new Coordinate(b)});
    }

    private LineString lineOf(List<Coordinate> path) {
        List<Coordinate> pts = new ArrayList<>();
        Coordinate prev = null;
        for (Coordinate c : path) {
            if (prev != null && prev.distance(c) < 1e-6) {
                continue;
            }
            pts.add(new Coordinate(c));
            prev = c;
        }
        if (pts.size() < 2) {
            return null;
        }
        return gf.createLineString(pts.toArray(new Coordinate[0]));
    }

    private Piece extract(LengthIndexedLine lil, double from, double to, boolean special, double kSpec, String reason) {
        if (to - from < MIN_PIECE_M) {
            return null;
        }
        Geometry g = lil.extractLine(from, to);
        Coordinate[] coords = g.getCoordinates();
        if (coords == null || coords.length < 2) {
            return null;
        }
        Piece piece = new Piece();
        for (Coordinate c : coords) {
            piece.coords.add(new Coordinate(c));
        }
        piece.special = special;
        piece.kSpec = special ? Math.max(1.0, kSpec) : 1.0;
        piece.reason = special ? reason : null;
        piece.method = special ? "special" : "base";
        piece.lengthM = to - from;
        return piece;
    }

    static List<Coordinate> densify(List<Coordinate> path, double step) {
        List<Coordinate> out = new ArrayList<>();
        out.add(new Coordinate(path.get(0)));
        for (int i = 1; i < path.size(); i++) {
            Coordinate a = path.get(i - 1);
            Coordinate b = path.get(i);
            double len = a.distance(b);
            int parts = Math.max(1, (int) Math.floor(len / step));
            for (int k = 1; k < parts; k++) {
                double t = k / (double) parts;
                out.add(new Coordinate(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
            }
            out.add(new Coordinate(b));
        }
        return out;
    }

    static double accLength(List<Coordinate> path, int index) {
        double s = 0;
        for (int i = 1; i <= index && i < path.size(); i++) {
            s += path.get(i - 1).distance(path.get(i));
        }
        return s;
    }

    public static double hitLength(Geometry line, Geometry band) {
        try {
            Geometry g = line.intersection(band);
            return lineLength(g);
        } catch (RuntimeException e) {
            return 0;
        }
    }

    public static double lineLength(Geometry g) {
        if (g == null || g.isEmpty()) {
            return 0;
        }
        if (g instanceof LineString || g instanceof MultiLineString) {
            return g.getLength();
        }
        if (g instanceof GeometryCollection && !(g instanceof Polygon)) {
            double s = 0;
            for (int i = 0; i < g.getNumGeometries(); i++) {
                Geometry part = g.getGeometryN(i);
                if (part != null && part != g) {
                    s += lineLength(part);
                }
            }
            return s;
        }
        return 0;
    }

    private static boolean angled(AppendixModel.ConstraintSpec rule) {
        return rule != null && rule.minAngleDeg != null && rule.minAngleDeg > 0;
    }

    private static AppendixModel.ConstraintSpec roadFallback() {
        AppendixModel.ConstraintSpec spec = new AppendixModel.ConstraintSpec();
        spec.action = "SPECIAL";
        spec.minAngleDeg = 45.0;
        spec.kSpec = 1.6;
        spec.extendM = 3;
        spec.extraGridCost = 12;
        spec.method = "special";
        return spec;
    }

    public static final class Travel {
        public final double length;
        public final double cost;
        public final boolean allowed;
        public final boolean special;
        public final double kSpec;
        public final String reason;
        public final double extendM;
        public final double hitM;
        public final double crossingAngleDeg;

        private Travel(double length, double cost, boolean allowed, boolean special, double kSpec,
                       String reason, double extendM, double hitM, double crossingAngleDeg) {
            this.length = length;
            this.cost = cost;
            this.allowed = allowed;
            this.special = special;
            this.kSpec = kSpec;
            this.reason = reason;
            this.extendM = extendM;
            this.hitM = hitM;
            this.crossingAngleDeg = crossingAngleDeg;
        }

        static Travel free(double length) {
            return new Travel(length, length, true, false, 1.0, null, 0, 0, 90);
        }

        static Travel ok(double length, double cost, boolean special, double kSpec, String reason,
                         double extendM, double hitM, double ang) {
            return new Travel(length, cost, true, special, kSpec, reason, extendM, hitM, ang);
        }

        static Travel blocked(double length) {
            return new Travel(length, Double.POSITIVE_INFINITY, false, true, 1.0, null, 0, length, 0);
        }
    }

    public static final class Piece {
        public final List<Coordinate> coords = new ArrayList<>();
        public boolean special;
        public double kSpec = 1.0;
        public String reason;
        public String method = "base";
        public double lengthM;

        static Piece base(List<Coordinate> path) {
            Piece p = new Piece();
            Coordinate prev = null;
            for (Coordinate c : path) {
                if (prev != null && prev.distance(c) < 1e-6) {
                    continue;
                }
                p.coords.add(new Coordinate(c));
                if (prev != null) {
                    p.lengthM += prev.distance(c);
                }
                prev = c;
            }
            return p;
        }

        public Coordinate start() {
            return coords.get(0);
        }

        public Coordinate end() {
            return coords.get(coords.size() - 1);
        }
    }

    private static final class Band {
        Geometry geom;
        PreparedGeometry prepared;
        Coordinate axis;
        double widthM;
        double minAngleDeg;
        double kSpec;
        double extendM;
        int extraGrid;
        String type;
        boolean angleSensitive;
    }

    private static final class Run {
        boolean special;
        double start;
        double end;
        double kSpec;
        double extendM;
        String reason;
    }
}
