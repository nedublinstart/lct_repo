package ru.lct.heatnet.engine.greedy;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Locale;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.geom.prep.PreparedGeometry;
import org.locationtech.jts.geom.prep.PreparedGeometryFactory;
import org.locationtech.jts.index.strtree.STRtree;
import org.locationtech.jts.operation.distance.DistanceOp;
import org.locationtech.jts.operation.overlayng.OverlayNGRobust;
import org.locationtech.jts.operation.union.UnaryUnionOp;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

/**
 * Препятствия для трассировки: кварталы ОКС закрываются (дворы не являются коридором),
 * трасса идёт по уличной сети вокруг кварталов.
 */
public final class ObstacleIndex {

    private final GeometryFactory gf = GeoJsonGeometries.GF;
    private final List<Prepared> avoids = new ArrayList<>();
    private final List<Prepared> costlies = new ArrayList<>();
    private final List<PreparedGeometry> allows = new ArrayList<>();
    private final List<Polygon> blocks = new ArrayList<>();
    private final STRtree avoidTree = new STRtree();
    private SpecialLayer special = SpecialLayer.empty();

    public static ObstacleIndex build(Scene scene) {
        return build(scene, null);
    }

    public static ObstacleIndex build(Scene scene, AppendixModel appendix) {
        double closeM = appendix != null ? appendix.getRouting().blockCloseM : 10.0;
        double clearanceM = appendix != null ? appendix.getRouting().clearanceM : 2.0;
        ObstacleIndex index = new ObstacleIndex();
        List<Geometry> buildings = new ArrayList<>();
        for (SpatialConstraint c : scene.constraints) {
            if (c.geometry == null || c.rule == null) {
                continue;
            }
            if (c.rule.cross() || c.rule.special()) {
                Prepared p = prepared(c, c.geometry);
                index.costlies.add(p);
                continue;
            }
            if (!c.rule.avoid()) {
                continue;
            }
            if (isBlockType(c.type)) {
                buildings.add(c.geometry);
            } else {
                Geometry g = c.geometry;
                if (c.rule.bufferM > 0) {
                    try {
                        Geometry buffered = g.buffer(c.rule.bufferM);
                        if (buffered != null && !buffered.isEmpty()) {
                            g = buffered;
                        }
                    } catch (RuntimeException ignored) {
                    }
                }
                index.addAvoid(prepared(c, g));
            }
        }
        Geometry blocks = closeBlocks(buildings, closeM, clearanceM, index.gf);
        if (blocks != null && !blocks.isEmpty()) {
            SpatialConstraint synthetic = new SpatialConstraint();
            synthetic.id = "blocks";
            synthetic.type = "oks";
            if (appendix != null) {
                synthetic.rule = appendix.constraintRule("oks");
            }
            collectPolygons(blocks, index.blocks);
            for (int i = 0; i < blocks.getNumGeometries(); i++) {
                Geometry part = blocks.getGeometryN(i);
                if (part == null || part.isEmpty()) {
                    continue;
                }
                index.addAvoid(prepared(synthetic, part));
            }
        }
        index.avoidTree.build();
        SpecialLayer layer = new SpecialLayer(appendix);
        for (Prepared p : index.costlies) {
            layer.addExplicit(p.geom, p.raw);
        }
        layer.inferFromBlocks(index.blocks);
        layer.finish();
        index.special = layer;
        return index;
    }

    public void allowCoordinates(Collection<Coordinate> coordinates, double radius) {
        for (Coordinate c : coordinates) {
            if (c != null) {
                allowGeometry(gf.createPoint(c).buffer(Math.max(1.0, radius)));
            }
        }
    }

    public void allowGeometry(Geometry geometry) {
        if (geometry == null || geometry.isEmpty()) {
            return;
        }
        allows.add(PreparedGeometryFactory.prepare(geometry));
    }

    public boolean blocked(Coordinate c) {
        if (c == null) {
            return true;
        }
        return hitsAvoid(gf.createPoint(c), true);
    }

    public int extra(Coordinate c) {
        return special.extraAt(c);
    }

    public boolean allowsTravel(Coordinate a, Coordinate b) {
        return special.allows(a, b);
    }

    public double travelCost(Coordinate a, Coordinate b) {
        return special.travelCost(a, b);
    }

    public double stepMultiplier(Coordinate a, Coordinate b) {
        return special.stepMultiplier(a, b);
    }

    public boolean inRoad(Coordinate c) {
        return special.inRoad(c);
    }

    public double maxStreetEdgeM() {
        return special.maxStreetEdgeM();
    }

    public double maxOpenEdgeM() {
        return special.maxOpenEdgeM();
    }

    public double sidewalkM() {
        return special.sidewalkM();
    }

    /**
     * Середина и тело отрезка у фасада: длинная хорда через двор/парк не проходит.
     */
    public boolean alongAvoid(Coordinate a, Coordinate b, double radius) {
        if (a == null || b == null) {
            return false;
        }
        double d = a.distance(b);
        if (d < 0.6) {
            return nearAvoid(a, radius);
        }
        int n = Math.min(12, Math.max(3, (int) Math.ceil(d / 16.0)));
        int hits = 0;
        for (int k = 0; k <= n; k++) {
            double t = k / (double) n;
            Coordinate p = new Coordinate(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
            if (nearAvoid(p, radius)) {
                hits++;
            }
        }
        return hits >= Math.max(3, (int) Math.ceil(0.7 * (n + 1)));
    }

    public boolean nearAvoid(Coordinate c, double radius) {
        if (c == null || radius < 0) {
            return false;
        }
        Point p = gf.createPoint(c);
        Envelope env = new Envelope(c);
        env.expandBy(radius);
        for (Prepared a : queryAvoids(env)) {
            try {
                if (a.geom.distance(p) <= radius + 1e-6) {
                    return true;
                }
            } catch (RuntimeException ignored) {
            }
        }
        return false;
    }

    public List<SpecialLayer.Piece> splitByTransport(List<Coordinate> path) {
        return special.splitByTransport(path);
    }

    public SpecialLayer special() {
        return special;
    }

    public boolean segmentHitsAvoid(Coordinate a, Coordinate b) {
        return segmentHitsAvoid(a, b, 0, false);
    }

    public boolean segmentHitsAvoid(Coordinate a, Coordinate b, double width) {
        return segmentHitsAvoid(a, b, width, false);
    }

    public boolean segmentHitsAvoid(Coordinate a, Coordinate b, double width, boolean interiorOnly) {
        if (a == null || b == null) {
            return true;
        }
        LineString ls = gf.createLineString(new Coordinate[]{new Coordinate(a), new Coordinate(b)});
        Geometry g = width > 1e-6 ? ls.buffer(width, 4) : ls;
        return hitsAvoid(g, false, interiorOnly);
    }

    public SpatialConstraint specialHit(Geometry line) {
        for (Prepared a : costlies) {
            if (a.raw.rule.special() && a.prepared.intersects(line)) {
                return a.raw;
            }
        }
        for (Prepared a : costlies) {
            if (a.raw.rule.cross() && a.prepared.intersects(line)) {
                return a.raw;
            }
        }
        return null;
    }

    public Coordinate nearestFree(Coordinate c, double maxRadius) {
        List<Coordinate> free = sampleFree(c, maxRadius);
        if (free.isEmpty()) {
            return c == null ? null : new Coordinate(c);
        }
        Coordinate best = free.get(0);
        double bestD = c.distance(best);
        for (Coordinate p : free) {
            double d = c.distance(p);
            if (d < bestD) {
                bestD = d;
                best = p;
            }
        }
        return best;
    }

    public Coordinate nearestFreeToward(Coordinate c, Coordinate target, double maxRadius) {
        return exitToStreet(c, target, 1.2);
    }

    /**
     * Выход из здания/квартала на улицу: короткий ввод к фасаду со стороны сети.
     */
    public Coordinate exitToStreet(Coordinate origin, Coordinate toward, double extraOut) {
        if (origin == null) {
            return null;
        }
        if (!blocked(origin)) {
            return new Coordinate(origin);
        }
        Prepared host = containing(origin);
        if (host == null) {
            return nearestFree(origin, 80);
        }
        Coordinate interior;
        try {
            interior = host.geom.getInteriorPoint().getCoordinate();
        } catch (RuntimeException e) {
            interior = host.geom.getCentroid().getCoordinate();
        }
        List<Coordinate> candidates = new ArrayList<>();
        try {
            Coordinate[] nearest = DistanceOp.nearestPoints(gf.createPoint(origin), host.geom.getBoundary());
            if (nearest != null && nearest.length > 1) {
                candidates.add(pushOut(nearest[1], interior, extraOut));
            }
        } catch (RuntimeException ignored) {
        }
        Coordinate[] ring = host.geom.getCoordinates();
        int step = Math.max(1, ring.length / 160);
        for (int i = 0; i < ring.length; i += step) {
            candidates.add(pushOut(ring[i], interior, extraOut));
        }
        if (toward != null) {
            double base = Math.atan2(toward.y - origin.y, toward.x - origin.x);
            double[] offsets = {0, 12, -12, 25, -25, 40, -40, 60, -60, 90, -90, 130, -130, 180};
            for (double deg : offsets) {
                double ang = base + Math.toRadians(deg);
                Coordinate far = new Coordinate(origin.x + 2500 * Math.cos(ang), origin.y + 2500 * Math.sin(ang));
                LineString ray = gf.createLineString(new Coordinate[]{new Coordinate(origin), far});
                try {
                    Geometry hit = ray.intersection(host.geom.getBoundary());
                    if (hit != null && !hit.isEmpty()) {
                        Coordinate[] pts = hit.getCoordinates();
                        Coordinate bestHit = pts[0];
                        double bestD = origin.distance(bestHit);
                        for (Coordinate p : pts) {
                            double d = origin.distance(p);
                            if (d < bestD) {
                                bestD = d;
                                bestHit = p;
                            }
                        }
                        candidates.add(pushOut(bestHit, interior, extraOut + 0.4));
                    }
                } catch (RuntimeException ignored) {
                }
            }
        }
        Coordinate best = null;
        double bestS = Double.POSITIVE_INFINITY;
        for (Coordinate q : candidates) {
            if (q == null || blocked(q)) {
                continue;
            }
            double stub = origin.distance(q);
            if (stub > 180) {
                continue;
            }
            double toNet = toward == null ? 0 : q.distance(toward);
            double s = stub + 0.35 * toNet;
            if (s < bestS) {
                bestS = s;
                best = q;
            }
        }
        if (best != null) {
            return best;
        }
        return nearestFree(origin, 120);
    }

    public List<Polygon> avoidPolygons() {
        List<Polygon> list = new ArrayList<>();
        for (Prepared a : avoids) {
            collectPolygons(a.geom, list);
        }
        return list;
    }

    public boolean pathHitsAvoid(List<Coordinate> path, int skipEnds) {
        if (path == null || path.size() < 2) {
            return false;
        }
        int from = Math.max(0, skipEnds);
        int to = path.size() - 1 - Math.max(0, skipEnds);
        if (to <= from) {
            return false;
        }
        for (int i = from; i < to; i++) {
            if (segmentHitsAvoid(path.get(i), path.get(i + 1), 0, true)) {
                return true;
            }
        }
        return false;
    }

    private List<Coordinate> sampleFree(Coordinate c, double maxRadius) {
        List<Coordinate> found = new ArrayList<>();
        if (c == null) {
            return found;
        }
        if (!blocked(c)) {
            found.add(new Coordinate(c));
            return found;
        }
        Point p = gf.createPoint(c);
        for (Prepared a : queryAvoids(p.getEnvelopeInternal())) {
            if (!a.prepared.intersects(p) && a.geom.distance(p) > 1.5) {
                continue;
            }
            Coordinate interior;
            try {
                interior = a.geom.getInteriorPoint().getCoordinate();
            } catch (RuntimeException e) {
                interior = a.geom.getCentroid().getCoordinate();
            }
            Coordinate[] pts = a.geom.getCoordinates();
            int step = Math.max(1, pts.length / 120);
            for (int i = 0; i < pts.length; i += step) {
                for (double extra : new double[]{1.0, 3.0, 6.0, 10.0}) {
                    Coordinate o = pushOut(pts[i], interior, extra);
                    if (!blocked(o) && c.distance(o) <= maxRadius + 40) {
                        found.add(o);
                    }
                }
            }
        }
        if (!found.isEmpty()) {
            return found;
        }
        for (int r = 2; r <= (int) Math.ceil(maxRadius) + 40; r += 2) {
            for (int ang = 0; ang < 360; ang += 8) {
                double rad = Math.toRadians(ang);
                Coordinate o = new Coordinate(c.x + r * Math.cos(rad), c.y + r * Math.sin(rad));
                if (!blocked(o)) {
                    found.add(o);
                }
            }
            if (found.size() >= 8) {
                break;
            }
        }
        return found;
    }

    private Prepared containing(Coordinate c) {
        Point p = gf.createPoint(c);
        Prepared best = null;
        double bestD = Double.POSITIVE_INFINITY;
        for (Prepared a : queryAvoids(p.getEnvelopeInternal())) {
            if (a.prepared.covers(p) || a.prepared.intersects(p)) {
                return a;
            }
            double d = a.geom.distance(p);
            if (d < bestD) {
                bestD = d;
                best = a;
            }
        }
        return best;
    }

    private boolean hitsAvoid(Geometry g, boolean point) {
        return hitsAvoid(g, point, false);
    }

    private boolean hitsAvoid(Geometry g, boolean point, boolean skipTouch) {
        Envelope env = g.getEnvelopeInternal();
        for (Prepared a : queryAvoids(env)) {
            if (!a.prepared.intersects(g)) {
                continue;
            }
            if ((point || skipTouch) && a.prepared.touches(g)) {
                continue;
            }
            if (point && allowed(g)) {
                return false;
            }
            return true;
        }
        return false;
    }

    private boolean allowed(Geometry g) {
        for (PreparedGeometry a : allows) {
            if (a.covers(g) || a.contains(g)) {
                return true;
            }
        }
        return false;
    }

    @SuppressWarnings("unchecked")
    private List<Prepared> queryAvoids(Envelope env) {
        List<Prepared> hits = avoidTree.query(env);
        return hits == null ? List.of() : hits;
    }

    private void addAvoid(Prepared p) {
        avoids.add(p);
        avoidTree.insert(p.geom.getEnvelopeInternal(), p);
    }

    private static Prepared prepared(SpatialConstraint raw, Geometry geom) {
        Prepared p = new Prepared();
        p.raw = raw;
        p.geom = geom;
        p.prepared = PreparedGeometryFactory.prepare(geom);
        return p;
    }

    static Geometry closeBlocks(List<Geometry> buildings, double closeM, double clearanceM, GeometryFactory gf) {
        if (buildings == null || buildings.isEmpty()) {
            return null;
        }
        List<Polygon> parts = new ArrayList<>();
        for (Geometry g : buildings) {
            collectPolygons(g, parts);
        }
        if (parts.isEmpty()) {
            return null;
        }
        // Кластеризуем только корпуса одного квартала (щели до ~5 м), улицы 7+ м не запечатываем.
        double mergeM = Math.min(5.0, Math.max(3.0, closeM * 0.5));
        int n = parts.size();
        boolean[] used = new boolean[n];
        List<Geometry> bodies = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            if (used[i]) {
                continue;
            }
            List<Geometry> cluster = new ArrayList<>();
            ArrayDeque<Integer> dq = new ArrayDeque<>();
            dq.add(i);
            used[i] = true;
            while (!dq.isEmpty()) {
                int k = dq.poll();
                Geometry seed = parts.get(k);
                cluster.add(seed);
                Envelope seedEnv = seed.getEnvelopeInternal();
                for (int j = 0; j < n; j++) {
                    if (used[j]) {
                        continue;
                    }
                    Envelope otherEnv = parts.get(j).getEnvelopeInternal();
                    if (seedEnv.distance(otherEnv) > mergeM) {
                        continue;
                    }
                    if (seed.distance(parts.get(j)) <= mergeM) {
                        used[j] = true;
                        dq.add(j);
                    }
                }
            }
            Geometry union = unionQuiet(cluster, gf);
            Geometry body = fillHoles(union, gf);
            try {
                double gate = 3.0;
                Geometry sealed = body.buffer(gate, 2);
                if (sealed != null && !sealed.isEmpty()) {
                    sealed = fillHoles(sealed, gf);
                    Geometry opened = sealed.buffer(-gate, 2);
                    if (opened != null && !opened.isEmpty()) {
                        body = fillHoles(opened, gf);
                    }
                }
            } catch (RuntimeException ignored) {
            }
            if (body != null && !body.isEmpty()) {
                bodies.add(body);
            }
        }
        if (bodies.isEmpty()) {
            return null;
        }
        List<Geometry> out = new ArrayList<>();
        for (Geometry body : bodies) {
            Geometry piece = fillHoles(body, gf);
            if (clearanceM > 0 && piece != null && !piece.isEmpty()) {
                try {
                    Geometry buffered = piece.buffer(clearanceM, 2);
                    if (buffered != null && !buffered.isEmpty()) {
                        piece = buffered;
                    }
                } catch (RuntimeException ignored) {
                }
            }
            if (piece != null && !piece.isEmpty()) {
                out.add(piece);
            }
        }
        if (out.isEmpty()) {
            return null;
        }
        return out.size() == 1 ? out.get(0) : gf.buildGeometry(out);
    }

    static Geometry unionQuiet(List<? extends Geometry> geoms, GeometryFactory gf) {
        if (geoms == null || geoms.isEmpty()) {
            return gf.createGeometryCollection();
        }
        if (geoms.size() == 1) {
            return geoms.get(0);
        }
        Geometry packed = gf.buildGeometry(new ArrayList<>(geoms));
        try {
            Geometry overlay = OverlayNGRobust.union(packed);
            if (overlay != null && !overlay.isEmpty()) {
                return overlay;
            }
        } catch (RuntimeException ignored) {
        }
        try {
            return UnaryUnionOp.union(new ArrayList<>(geoms));
        } catch (RuntimeException e) {
            return packed;
        }
    }

    static Geometry fillHoles(Geometry geometry, GeometryFactory gf) {
        if (geometry == null || geometry.isEmpty()) {
            return geometry;
        }
        List<Polygon> polys = new ArrayList<>();
        collectPolygons(geometry, polys);
        if (polys.isEmpty()) {
            return geometry;
        }
        List<Polygon> filled = new ArrayList<>();
        for (Polygon p : polys) {
            filled.add(gf.createPolygon(p.getExteriorRing()));
        }
        if (filled.size() == 1) {
            return filled.get(0);
        }
        return unionQuiet(filled, gf);
    }

    static void collectPolygons(Geometry geometry, List<Polygon> out) {
        if (geometry == null || geometry.isEmpty()) {
            return;
        }
        if (geometry instanceof Polygon) {
            out.add((Polygon) geometry);
            return;
        }
        for (int i = 0; i < geometry.getNumGeometries(); i++) {
            collectPolygons(geometry.getGeometryN(i), out);
        }
    }

    static boolean isBlockType(String type) {
        if (type == null || type.isBlank()) {
            return true;
        }
        String t = type.toLowerCase(Locale.ROOT);
        if (t.contains("water") || t.contains("river") || t.contains("вод")) {
            return false;
        }
        if (t.contains("rail") || t.contains("желез")) {
            return false;
        }
        return true;
    }

    private static Coordinate pushOut(Coordinate q, Coordinate interior, double extra) {
        double vx = q.x - interior.x;
        double vy = q.y - interior.y;
        double n = Math.hypot(vx, vy);
        if (n < 1e-6) {
            return new Coordinate(q.x + extra, q.y);
        }
        return new Coordinate(q.x + extra * vx / n, q.y + extra * vy / n);
    }

    private static final class Prepared {
        SpatialConstraint raw;
        Geometry geom;
        PreparedGeometry prepared;
    }
}
