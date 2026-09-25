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
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.geom.prep.PreparedGeometry;
import org.locationtech.jts.geom.prep.PreparedGeometryFactory;
import org.locationtech.jts.index.strtree.STRtree;
import org.locationtech.jts.operation.distance.DistanceOp;
import org.locationtech.jts.simplify.DouglasPeuckerSimplifier;
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
    /** Кадастровые контуры домов до закрытия квартала и буфера. Ввод ИТП считается по ним. */
    private final List<Polygon> footprints = new ArrayList<>();
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
                collectPolygons(c.geometry, index.footprints);
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
     * Каркас/порты Steiner — как раньше; ввод ИТП на выдаче идёт через {@link #wallPerpExit}.
     */
    public Coordinate exitToStreet(Coordinate origin, Coordinate toward, double extraOut) {
        if (origin == null) {
            return null;
        }
        if (!blocked(origin)) {
            Coordinate onFacade = facadeExit(origin, extraOut);
            return onFacade != null ? onFacade : new Coordinate(origin);
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
            double[] offsets = {0, 12, -12, 25, -25, 40, -40, 90, -90};
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
        addStreetExits(origin, extraOut, candidates);
        Coordinate best = null;
        double bestS = Double.POSITIVE_INFINITY;
        for (Coordinate q : candidates) {
            if (q == null || blocked(q)) {
                continue;
            }
            double stub = origin.distance(q);
            if (stub > 80) {
                continue;
            }
            double s = exitScore(origin, q, toward);
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

    /**
     * Выход из корпуса в сторону близкой цели (камера в 100–180 м), не на дальний торец.
     */
    public Coordinate exitFacing(Coordinate origin, Coordinate toward, double extraOut) {
        if (origin == null) {
            return null;
        }
        if (toward == null || origin.distance(toward) > 220) {
            return exitToStreet(origin, toward, extraOut);
        }
        Coordinate base = exitToStreet(origin, toward, extraOut);
        if (!blocked(origin)) {
            return base;
        }
        Prepared host = containing(origin);
        if (host == null) {
            return base;
        }
        Coordinate interior;
        try {
            interior = host.geom.getInteriorPoint().getCoordinate();
        } catch (RuntimeException e) {
            interior = host.geom.getCentroid().getCoordinate();
        }
        List<Coordinate> candidates = new ArrayList<>();
        if (base != null) {
            candidates.add(base);
        }
        double ang0 = Math.atan2(toward.y - origin.y, toward.x - origin.x);
        for (double deg : new double[]{0, 15, -15, 30, -30, 50, -50}) {
            double ang = ang0 + Math.toRadians(deg);
            Coordinate far = new Coordinate(origin.x + 2500 * Math.cos(ang), origin.y + 2500 * Math.sin(ang));
            LineString ray = gf.createLineString(new Coordinate[]{new Coordinate(origin), far});
            try {
                Geometry hit = ray.intersection(host.geom.getBoundary());
                if (hit == null || hit.isEmpty()) {
                    continue;
                }
                for (Coordinate p : hit.getCoordinates()) {
                    candidates.add(pushOut(p, interior, extraOut + 0.4));
                }
            } catch (RuntimeException ignored) {
            }
        }
        addStreetExits(origin, extraOut, candidates);
        Coordinate best = base;
        double bestS = best == null ? Double.POSITIVE_INFINITY : origin.distance(best) + origin.distance(toward);
        for (Coordinate q : candidates) {
            if (q == null || blocked(q) || origin.distance(q) > 80) {
                continue;
            }
            double s = origin.distance(q) * 0.35 + q.distance(toward);
            if (inRoad(q)) {
                s += 40;
            }
            SpecialLayer.Corridor near = special.nearestCorridor(q, sidewalkM() + 16);
            if (near != null && near.axis != null) {
                double ang = SpecialLayer.crossingAngleDeg(origin, q, near.axis);
                if (ang <= 16) {
                    s += 18;
                }
            }
            if (s < bestS) {
                bestS = s;
                best = q;
            }
        }
        return best != null ? best : base;
    }

    /**
     * Ввод ИТП: ⊥ одной стене или ⊥ другой, до фактической границы дома.
     */
    public Coordinate wallPerpExit(Coordinate origin, Coordinate toward, double extraOut) {
        if (origin == null) {
            return null;
        }
        if (!blocked(origin)) {
            Coordinate onFacade = facadeExit(origin, extraOut);
            return onFacade != null ? onFacade : new Coordinate(origin);
        }
        Prepared host = containing(origin);
        if (host == null) {
            return nearestFree(origin, 80);
        }
        List<Coordinate> sides = sideExits(origin, extraOut);
        Coordinate side = pickSide(origin, toward, sides);
        if (side != null) {
            return side;
        }
        Coordinate picked = pickWallPerpExit(host, origin, toward, extraOut);
        if (picked != null) {
            return picked;
        }
        return nearestFree(origin, 120);
    }

    /**
     * Первый отрезок ввода ИТП: вдоль нормали к стене (∥ одной из двух осей корпуса).
     */
    public boolean wallPerpOk(Coordinate inside, Coordinate outside) {
        if (inside == null || outside == null) {
            return false;
        }
        if (inside.distance(outside) <= 8) {
            return true;
        }
        Prepared host = containing(inside);
        Coordinate[] axes = host == null ? axisPair(new Coordinate(1, 0)) : wallAxes(host.geom, inside);
        return alongWallAxis(inside, outside, axes, 16);
    }

    /**
     * Два варианта: ⊥ одной стене или ⊥ другой. Берём короткий выход на
     * внешний фасад, двор и длинный прокоп через корпус отбрасываем.
     */
    private Coordinate pickWallPerpExit(Prepared host, Coordinate origin, Coordinate toward,
                                        double extraOut) {
        Coordinate interior;
        try {
            interior = host.geom.getInteriorPoint().getCoordinate();
        } catch (RuntimeException e) {
            interior = host.geom.getCentroid().getCoordinate();
        }
        Coordinate[] axes = wallAxes(host.geom, origin);
        List<Coordinate> candidates = new ArrayList<>();
        for (Coordinate axis : axes) {
            if (axis == null) {
                continue;
            }
            for (int sign : new int[]{1, -1}) {
                Coordinate dir = new Coordinate(sign * axis.x, sign * axis.y);
                for (double deg : new double[]{0, 12, -12}) {
                    Coordinate hit = rayHitBoundary(origin, rotateUnit(dir, deg), host);
                    if (hit == null || origin.distance(hit) < 0.35 || origin.distance(hit) > 80) {
                        continue;
                    }
                    Coordinate q = pushOut(hit, interior, extraOut);
                    if (q == null || blocked(q)) {
                        q = pushOut(hit, interior, extraOut + 1.1);
                    }
                    if (q != null && !blocked(q)) {
                        candidates.add(q);
                    }
                }
            }
        }
        try {
            Coordinate[] nearest = DistanceOp.nearestPoints(gf.createPoint(origin), host.geom.getBoundary());
            if (nearest != null && nearest.length > 1) {
                Coordinate hit = nearest[1];
                if (hit != null && origin.distance(hit) <= 80 && alongWallAxis(origin, hit, axes, 16)) {
                    Coordinate q = pushOut(hit, interior, extraOut);
                    if (q != null && !blocked(q)) {
                        candidates.add(q);
                    }
                }
            }
        } catch (RuntimeException ignored) {
        }
        double shortest = Double.POSITIVE_INFINITY;
        double shortestOuter = Double.POSITIVE_INFINITY;
        for (Coordinate q : candidates) {
            double d = origin.distance(q);
            shortest = Math.min(shortest, d);
            if (onOuterFacade(host.geom, q)) {
                shortestOuter = Math.min(shortestOuter, d);
            }
        }
        Coordinate best = null;
        double bestS = Double.POSITIVE_INFINITY;
        for (Coordinate q : candidates) {
            double indoor = origin.distance(q);
            boolean outer = onOuterFacade(host.geom, q);
            if (!outer && Double.isFinite(shortestOuter) && indoor > shortestOuter + 4) {
                continue;
            }
            double cap = Double.isFinite(shortestOuter) ? shortestOuter : shortest;
            if (indoor > Math.max(28, cap * 1.7) && indoor > cap + 8) {
                continue;
            }
            double s = exitScore(origin, q, toward);
            if (indoor > 22) {
                s += (indoor - 22) * 1.5;
            }
            if (!outer) {
                s += 140;
            }
            if (toward != null) {
                s += q.distance(toward) * 0.04;
            }
            if (s < bestS) {
                bestS = s;
                best = q;
            }
        }
        return best;
    }

    private Coordinate[] wallAxes(Geometry geom, Coordinate origin) {
        Coordinate u = null;
        if (special != null) {
            double bestArea = Double.POSITIVE_INFINITY;
            for (Coordinate axis : special.dominantAxes()) {
                if (axis == null) {
                    continue;
                }
                double n = Math.hypot(axis.x, axis.y);
                if (n < 1e-9) {
                    continue;
                }
                Coordinate cand = new Coordinate(axis.x / n, axis.y / n);
                double area = obbArea(geom, cand);
                if (area < bestArea) {
                    bestArea = area;
                    u = cand;
                }
            }
            if (u == null) {
                SpecialLayer.Corridor cor = special.nearestCorridor(origin, 140);
                if (cor != null && cor.axis != null) {
                    double n = Math.hypot(cor.axis.x, cor.axis.y);
                    if (n > 1e-9) {
                        u = new Coordinate(cor.axis.x / n, cor.axis.y / n);
                    }
                }
            }
        }
        if (u == null) {
            u = longestEdgeDir(geom);
        }
        if (u == null) {
            u = new Coordinate(1, 0);
        }
        return axisPair(u);
    }

    private static Coordinate[] axisPair(Coordinate u) {
        return new Coordinate[]{u, new Coordinate(-u.y, u.x)};
    }

    private static Coordinate longestEdgeDir(Geometry geom) {
        if (geom == null) {
            return null;
        }
        Coordinate[] pts = geom.getCoordinates();
        Coordinate best = null;
        double bestL = 0;
        for (int i = 0; i < pts.length - 1; i++) {
            if (pts[i] == null || pts[i + 1] == null) {
                continue;
            }
            double d = pts[i].distance(pts[i + 1]);
            if (d > bestL) {
                bestL = d;
                double n = Math.hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y);
                if (n > 1e-9) {
                    best = new Coordinate((pts[i + 1].x - pts[i].x) / n, (pts[i + 1].y - pts[i].y) / n);
                }
            }
        }
        return best;
    }

    private static boolean alongWallAxis(Coordinate a, Coordinate b, Coordinate[] axes, double deg) {
        if (a == null || b == null || axes == null) {
            return false;
        }
        for (Coordinate axis : axes) {
            if (axis == null) {
                continue;
            }
            if (SpecialLayer.crossingAngleDeg(a, b, axis) <= deg) {
                return true;
            }
        }
        return false;
    }

    private Coordinate rayHitBoundary(Coordinate origin, Coordinate dir, Prepared host) {
        if (origin == null || dir == null || host == null || host.geom == null) {
            return null;
        }
        double n = Math.hypot(dir.x, dir.y);
        if (n < 1e-9) {
            return null;
        }
        Coordinate far = new Coordinate(origin.x + 2500 * dir.x / n, origin.y + 2500 * dir.y / n);
        LineString ray = gf.createLineString(new Coordinate[]{new Coordinate(origin), far});
        try {
            Geometry hit = ray.intersection(host.geom.getBoundary());
            if (hit == null || hit.isEmpty()) {
                return null;
            }
            Coordinate best = null;
            double bestD = Double.POSITIVE_INFINITY;
            for (Coordinate p : hit.getCoordinates()) {
                if (p == null) {
                    continue;
                }
                double d = origin.distance(p);
                double along = (p.x - origin.x) * dir.x / n + (p.y - origin.y) * dir.y / n;
                if (d < 0.3 || along < 0.3 || d >= bestD) {
                    continue;
                }
                bestD = d;
                best = p;
            }
            return best == null ? null : new Coordinate(best);
        } catch (RuntimeException e) {
            return null;
        }
    }

    private boolean onOuterFacade(Geometry geom, Coordinate q) {
        if (geom == null || q == null) {
            return false;
        }
        List<Polygon> polys = new ArrayList<>();
        collectPolygons(geom, polys);
        Point p = gf.createPoint(q);
        for (Polygon poly : polys) {
            if (poly != null && !poly.isEmpty() && poly.getExteriorRing().distance(p) <= 2.2) {
                return true;
            }
        }
        return false;
    }

    private static Coordinate rotateUnit(Coordinate dir, double deg) {
        if (dir == null || Math.abs(deg) < 1e-9) {
            return dir;
        }
        double a = Math.toRadians(deg);
        double c = Math.cos(a);
        double s = Math.sin(a);
        return new Coordinate(dir.x * c - dir.y * s, dir.x * s + dir.y * c);
    }

    /**
     * Выходы ⊥ упрощённому фасаду (лестница кадастра схлопнута).
     * Точка стоит сразу за контуром, даже если буфер квартала ещё «закрыт»:
     * ввод — это нормаль к ближайшей стене, а не прокоп вдоль неё до свободного торца.
     */
    public List<Coordinate> facadeNormals(Coordinate origin, double extraOut) {
        List<Coordinate> out = new ArrayList<>();
        if (origin == null) {
            return out;
        }
        Polygon host = footprintAt(origin);
        if (host == null) {
            return out;
        }
        Polygon shell = host;
        try {
            Geometry simplified = DouglasPeuckerSimplifier.simplify(host, 2.4);
            Polygon largest = largestFoot(simplified);
            if (largest != null && largest.getExteriorRing() != null
                    && largest.getExteriorRing().getNumPoints() >= 4) {
                shell = largest;
            }
        } catch (RuntimeException ignored) {
        }
        Coordinate[] ring = shell.getExteriorRing().getCoordinates();
        double push = Math.max(1.05, extraOut);
        for (int i = 0; i < ring.length - 1; i++) {
            Coordinate a = ring[i];
            Coordinate b = ring[i + 1];
            if (a == null || b == null) {
                continue;
            }
            double vx = b.x - a.x;
            double vy = b.y - a.y;
            double el = Math.hypot(vx, vy);
            if (el < 8) {
                continue;
            }
            double t = ((origin.x - a.x) * vx + (origin.y - a.y) * vy) / (el * el);
            if (t < -0.06 || t > 1.06) {
                continue;
            }
            double tc = Math.max(0.015, Math.min(0.985, t));
            Coordinate hit = new Coordinate(a.x + tc * vx, a.y + tc * vy);
            if (origin.distance(hit) > 80) {
                continue;
            }
            double nx = -vy / el;
            double ny = vx / el;
            Coordinate probe = new Coordinate(hit.x + nx * 0.7, hit.y + ny * 0.7);
            if (coversFoot(shell, probe) || coversFoot(host, probe)) {
                nx = -nx;
                ny = -ny;
            }
            double indoor = origin.distance(hit);
            if (indoor > 1.2 && footprintCutM(origin, hit) + 1.0 < indoor * 0.5) {
                continue;
            }
            Coordinate inGap = null;
            Coordinate q = null;
            for (double d = push; d <= push + 7.0; d += 0.35) {
                Coordinate c = new Coordinate(hit.x + nx * d, hit.y + ny * d);
                if (insideFootprint(c)) {
                    if (inGap != null) {
                        break;
                    }
                    continue;
                }
                if (inGap == null) {
                    inGap = c;
                }
                if (!blocked(c)) {
                    q = c;
                    break;
                }
            }
            if (q == null) {
                q = inGap;
            }
            if (q == null) {
                continue;
            }
            boolean dup = false;
            for (Coordinate e : out) {
                if (e.distance(q) < 1.6) {
                    dup = true;
                    break;
                }
            }
            if (!dup) {
                out.add(new Coordinate(q));
            }
        }
        return out;
    }

    private static boolean coversFoot(Polygon poly, Coordinate c) {
        if (poly == null || c == null) {
            return false;
        }
        try {
            Point p = poly.getFactory().createPoint(c);
            return poly.covers(p) || poly.contains(p);
        } catch (RuntimeException e) {
            return false;
        }
    }

    /**
     * Короткий отрезок поперёк щели между фасадами: дома с обеих сторон, угол к стене ближе к 90°.
     * Ход вдоль фасада и длинная диагональ сюда не попадают.
     */
    public boolean crossesStreetGap(Coordinate a, Coordinate b) {
        if (a == null || b == null) {
            return false;
        }
        double len = a.distance(b);
        if (len < 4.5 || len > 22) {
            return false;
        }
        if (insideFootprint(a) || insideFootprint(b)) {
            return false;
        }
        if (runsAlongFacade(a, b)) {
            return false;
        }
        double wall = angleToNearestFacade(a, b);
        if (wall < 68) {
            return false;
        }
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double n = Math.hypot(dx, dy);
        if (n < 1e-6) {
            return false;
        }
        double nx = -dy / n;
        double ny = dx / n;
        Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        Double left = facadeRay(mid, nx, ny);
        Double right = facadeRay(mid, -nx, -ny);
        if (left == null || right == null) {
            return false;
        }
        double width = left + right;
        return width >= 8 && width <= 48 && Math.min(left, right) <= 16;
    }

    /** Угол к ближайшему длинному фасаду упрощённого контура: 0 вдоль, 90 поперёк. */
    public double angleToNearestFacade(Coordinate a, Coordinate b) {
        if (a == null || b == null || a.distance(b) < 0.4) {
            return 90;
        }
        Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        Coordinate axis = null;
        double bestD = 42;
        for (Polygon poly : footprints) {
            if (poly == null || poly.isEmpty()) {
                continue;
            }
            Polygon shell = poly;
            try {
                Polygon simplified = largestFoot(DouglasPeuckerSimplifier.simplify(poly, 2.4));
                if (simplified != null && simplified.getExteriorRing() != null) {
                    shell = simplified;
                }
            } catch (RuntimeException ignored) {
            }
            Coordinate[] ring = shell.getExteriorRing().getCoordinates();
            for (int i = 0; i < ring.length - 1; i++) {
                Coordinate p = ring[i];
                Coordinate q = ring[i + 1];
                if (p == null || q == null || p.distance(q) < 12) {
                    continue;
                }
                double d = distPointSeg(mid, p, q);
                if (d < bestD) {
                    bestD = d;
                    axis = new Coordinate(q.x - p.x, q.y - p.y);
                }
            }
        }
        if (axis == null) {
            return angleToNearestWall(a, b);
        }
        return SpecialLayer.crossingAngleDeg(a, b, axis);
    }

    private Double facadeRay(Coordinate origin, double ux, double uy) {
        if (origin == null) {
            return null;
        }
        for (double d = 2.0; d <= 42; d += 1.0) {
            Coordinate q = new Coordinate(origin.x + ux * d, origin.y + uy * d);
            if (insideFootprint(q)) {
                return d;
            }
        }
        return null;
    }

    /**
     * Перпендикуляры к ближайшим стенам кадастрового контура, до точки снаружи дома.
     * Не луч вдоль оси улицы: тот промахивается мимо ближней стены и выходит в торец.
     */
    public List<Coordinate> sideExits(Coordinate origin, double extraOut) {
        List<Coordinate> out = new ArrayList<>();
        if (origin == null) {
            return out;
        }
        Polygon host = footprintAt(origin);
        if (host == null) {
            if (!blocked(origin)) {
                out.add(new Coordinate(origin));
            }
            return out;
        }
        List<Side> sides = new ArrayList<>();
        collectSides(host.getExteriorRing(), host, origin, sides);
        for (int h = 0; h < host.getNumInteriorRing(); h++) {
            collectSides(host.getInteriorRingN(h), host, origin, sides);
        }
        if (sides.isEmpty()) {
            return out;
        }
        double nearest = Double.POSITIVE_INFINITY;
        for (Side s : sides) {
            nearest = Math.min(nearest, s.wall);
        }
        emergeSides(sides, host, nearest, 8.0, extraOut, out);
        if (out.isEmpty()) {
            emergeSides(sides, host, nearest, 14.0, extraOut, out);
        }
        return out;
    }

    private void emergeSides(List<Side> sides, Polygon host, double nearest, double slack,
                             double extraOut, List<Coordinate> out) {
        for (Side s : sides) {
            if (s.wall > nearest + slack) {
                continue;
            }
            Coordinate q = emerge(s.hit, s.dir, host, extraOut);
            if (q == null) {
                q = emergeShifted(s, host, extraOut);
            }
            if (q == null) {
                continue;
            }
            boolean dup = false;
            for (Coordinate e : out) {
                if (e.distance(q) < 1.4) {
                    dup = true;
                    break;
                }
            }
            if (!dup) {
                out.add(q);
            }
        }
    }

    /** Сдвиг вдоль той же стены, если прямо напротив выхода стоит соседний корпус. */
    private Coordinate emergeShifted(Side s, Polygon host, double extraOut) {
        for (double shift : new double[]{2.2, -2.2, 4.5, -4.5, 7.0, -7.0}) {
            Coordinate hit = new Coordinate(s.hit.x + s.along.x * shift, s.hit.y + s.along.y * shift);
            if (host.getBoundary().distance(gf.createPoint(hit)) > 1.6) {
                continue;
            }
            Coordinate q = emerge(hit, s.dir, host, extraOut);
            if (q != null) {
                return q;
            }
        }
        return null;
    }

    /**
     * От стены наружу по нормали, пока точка не выйдет из буфера квартала.
     * Направление не меняется: ввод остаётся перпендикулярным фасаду.
     */
    private Coordinate emerge(Coordinate hit, Coordinate dir, Polygon host, double extraOut) {
        double n = Math.hypot(dir.x, dir.y);
        if (n < 1e-9) {
            return null;
        }
        double ux = dir.x / n;
        double uy = dir.y / n;
        double start = Math.max(0.35, extraOut);
        for (double d = start; d <= 8.5; d += 0.3) {
            Coordinate q = new Coordinate(hit.x + ux * d, hit.y + uy * d);
            Point p = gf.createPoint(q);
            if (host.covers(p) || host.contains(p)) {
                continue;
            }
            if (!blocked(q)) {
                return q;
            }
        }
        return null;
    }

    private void collectSides(LineString ring, Polygon host, Coordinate origin, List<Side> sides) {
        if (ring == null) {
            return;
        }
        Coordinate[] pts = ring.getCoordinates();
        boolean inside = host.covers(gf.createPoint(origin)) || host.contains(gf.createPoint(origin));
        for (int i = 0; i < pts.length - 1; i++) {
            Coordinate a = pts[i];
            Coordinate b = pts[i + 1];
            if (a == null || b == null) {
                continue;
            }
            double vx = b.x - a.x;
            double vy = b.y - a.y;
            double len2 = vx * vx + vy * vy;
            if (len2 < 0.36) {
                continue;
            }
            double t = ((origin.x - a.x) * vx + (origin.y - a.y) * vy) / len2;
            if (t <= 0.03 || t >= 0.97) {
                continue;
            }
            Coordinate hit = new Coordinate(a.x + t * vx, a.y + t * vy);
            double wall = origin.distance(hit);
            if (wall < 0.15 || wall > 80) {
                continue;
            }
            double len = Math.sqrt(len2);
            double ox = hit.x - origin.x;
            double oy = hit.y - origin.y;
            if (!inside) {
                ox = -ox;
                oy = -oy;
            }
            Side s = new Side();
            s.hit = hit;
            s.wall = wall;
            s.edge = len;
            s.dir = new Coordinate(ox, oy);
            s.along = new Coordinate(vx / len, vy / len);
            sides.add(s);
        }
    }

    private Polygon footprintAt(Coordinate origin) {
        if (origin == null || footprints.isEmpty()) {
            return null;
        }
        Point p = gf.createPoint(origin);
        Polygon containing = null;
        for (Polygon poly : footprints) {
            if (poly == null || poly.isEmpty()) {
                continue;
            }
            if (!(poly.covers(p) || poly.contains(p))) {
                continue;
            }
            if (containing == null || poly.getArea() < containing.getArea()) {
                containing = poly;
            }
        }
        if (containing != null) {
            return containing;
        }
        Polygon nearest = null;
        double best = 8.0;
        for (Polygon poly : footprints) {
            if (poly == null || poly.isEmpty()) {
                continue;
            }
            double d = poly.distance(p);
            if (d <= best) {
                best = d;
                nearest = poly;
            }
        }
        return nearest;
    }

    private static Coordinate pickSide(Coordinate origin, Coordinate toward, List<Coordinate> sides) {
        if (sides == null || sides.isEmpty() || origin == null) {
            return null;
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
            if (indoor > shortest + 8) {
                continue;
            }
            double s = indoor * 2.2;
            if (toward != null) {
                s += q.distance(toward) * 0.45;
            }
            if (s < bestS) {
                bestS = s;
                best = q;
            }
        }
        return best == null ? null : new Coordinate(best);
    }

    /**
     * Сколько метров отрезка лежит внутри кадастрового контура (не буфера квартала).
     * Короткий ввод ИТП до своей стены даёт положительную длину — это ожидаемо.
     * Хорда, которая задевает чужой дом, тоже.
     */
    public boolean insideFootprint(Coordinate c) {
        if (c == null || footprints.isEmpty()) {
            return false;
        }
        Point p = gf.createPoint(c);
        for (Polygon poly : footprints) {
            if (poly == null || poly.isEmpty()) {
                continue;
            }
            try {
                if (poly.covers(p) || poly.contains(p)) {
                    return true;
                }
            } catch (RuntimeException ignored) {
            }
        }
        return false;
    }

    public double footprintCutM(Coordinate a, Coordinate b) {
        if (a == null || b == null || a.distance(b) < 0.05 || footprints.isEmpty()) {
            return 0;
        }
        LineString ls = gf.createLineString(new Coordinate[]{new Coordinate(a), new Coordinate(b)});
        double sum = 0;
        for (Polygon poly : footprints) {
            if (poly == null || poly.isEmpty()) {
                continue;
            }
            try {
                if (poly.getEnvelopeInternal().distance(ls.getEnvelopeInternal()) > 0.3) {
                    continue;
                }
                if (!poly.intersects(ls)) {
                    continue;
                }
                Geometry inter = ls.intersection(poly);
                if (inter != null && !inter.isEmpty()) {
                    sum += inter.getLength();
                }
            } catch (RuntimeException ignored) {
            }
        }
        return sum;
    }

    /**
     * Угол отрезка к ближайшей длинной стене: 0 — вдоль фасада, 90 — поперёк.
     */
    public double angleToNearestWall(Coordinate origin, Coordinate exit) {
        Coordinate wall = nearestWallAxis(origin);
        if (wall == null || origin == null || exit == null || origin.distance(exit) < 0.4) {
            return 90;
        }
        return SpecialLayer.crossingAngleDeg(origin, exit, wall);
    }

    /**
     * Короткий обход угла: дуга по контуру дома с отступом 1.5 м вместо хорды сквозь корпус.
     * {@code null} — отрезок дом не режет.
     */
    public List<Coordinate> skirt(Coordinate a, Coordinate b) {
        if (a == null || b == null || footprintCutM(a, b) < 1.05) {
            return null;
        }
        List<Coordinate> path = new ArrayList<>();
        path.add(new Coordinate(a));
        path.add(new Coordinate(b));
        for (int pass = 0; pass < 6; pass++) {
            boolean changed = false;
            List<Coordinate> next = new ArrayList<>();
            next.add(new Coordinate(path.get(0)));
            for (int i = 1; i < path.size(); i++) {
                Coordinate from = next.get(next.size() - 1);
                Coordinate to = path.get(i);
                List<Coordinate> arc = skirtOne(from, to);
                if (arc != null && arc.size() >= 3) {
                    for (int k = 1; k < arc.size(); k++) {
                        Coordinate q = arc.get(k);
                        if (q != null && next.get(next.size() - 1).distance(q) >= 0.4) {
                            next.add(q);
                        }
                    }
                    changed = true;
                } else if (to != null && next.get(next.size() - 1).distance(to) >= 0.4) {
                    next.add(new Coordinate(to));
                }
            }
            path = next;
            if (!changed) {
                break;
            }
        }
        simplifySkirt(path);
        return path.size() >= 2 ? path : null;
    }

    /** Почти прямые точки буфера схлопываются, угол дома остаётся. */
    private void simplifySkirt(List<Coordinate> path) {
        boolean changed = true;
        int guard = 0;
        while (changed && path.size() > 2 && guard++ < 8) {
            changed = false;
            for (int i = 1; i < path.size() - 1; i++) {
                Coordinate a = path.get(i - 1);
                Coordinate b = path.get(i);
                Coordinate c = path.get(i + 1);
                double leg = Math.min(a.distance(b), b.distance(c));
                if (leg > 6 && turnSkirt(a, b, c) < 168) {
                    continue;
                }
                if (footprintCutM(a, c) > 0.7) {
                    continue;
                }
                path.remove(i);
                changed = true;
                break;
            }
        }
    }

    private static double turnSkirt(Coordinate a, Coordinate b, Coordinate c) {
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
        return Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, d))));
    }

    private List<Coordinate> skirtOne(Coordinate a, Coordinate b) {
        if (a == null || b == null || a.distance(b) < 0.8) {
            return null;
        }
        LineString ls = gf.createLineString(new Coordinate[]{new Coordinate(a), new Coordinate(b)});
        Polygon hit = null;
        double bestCut = 1.05;
        for (Polygon poly : footprints) {
            if (poly == null || poly.isEmpty()) {
                continue;
            }
            try {
                if (poly.getEnvelopeInternal().distance(ls.getEnvelopeInternal()) > 0.3 || !poly.intersects(ls)) {
                    continue;
                }
                Geometry inter = ls.intersection(poly);
                double cut = inter == null || inter.isEmpty() ? 0 : inter.getLength();
                if (cut > bestCut) {
                    bestCut = cut;
                    hit = poly;
                }
            } catch (RuntimeException ignored) {
            }
        }
        if (hit == null) {
            return null;
        }
        Geometry buf;
        try {
            buf = hit.buffer(1.55, 2);
        } catch (RuntimeException e) {
            return null;
        }
        Polygon shell = largestFoot(buf);
        if (shell == null || shell.getExteriorRing() == null) {
            return null;
        }
        LineString ring = shell.getExteriorRing();
        Geometry cross;
        try {
            cross = ls.intersection(ring);
        } catch (RuntimeException e) {
            return null;
        }
        if (cross == null || cross.isEmpty()) {
            return null;
        }
        List<Coordinate> hits = new ArrayList<>();
        for (Coordinate p : cross.getCoordinates()) {
            if (p != null) {
                hits.add(new Coordinate(p));
            }
        }
        if (hits.size() < 2) {
            Coordinate inside = closerToPoly(hit, a, b);
            if (hits.size() == 1 && inside != null && ring.distance(gf.createPoint(inside)) < 3.5) {
                try {
                    Coordinate[] near = DistanceOp.nearestPoints(gf.createPoint(inside), ring);
                    if (near != null && near.length > 1 && near[1] != null) {
                        hits.add(new Coordinate(near[1]));
                    }
                } catch (RuntimeException ignored) {
                }
            }
        }
        if (hits.size() < 2) {
            return null;
        }
        LengthIndexedLine along = new LengthIndexedLine(ls);
        hits.sort((p, q) -> Double.compare(along.indexOf(p), along.indexOf(q)));
        Coordinate enter = hits.get(0);
        Coordinate leave = hits.get(hits.size() - 1);
        if (enter.distance(leave) < 0.4) {
            return null;
        }
        List<Coordinate> arc = shorterArc(ring, enter, leave);
        if (arc == null || arc.size() < 2) {
            return null;
        }
        double arcLen = pathLen(arc);
        double direct = a.distance(b);
        if (arcLen > direct * 4.2 + 90) {
            return null;
        }
        List<Coordinate> out = new ArrayList<>();
        out.add(new Coordinate(a));
        for (Coordinate p : arc) {
            if (out.get(out.size() - 1).distance(p) >= 0.45) {
                out.add(new Coordinate(p));
            }
        }
        if (out.get(out.size() - 1).distance(b) >= 0.45) {
            out.add(new Coordinate(b));
        }
        for (int i = 1; i < out.size(); i++) {
            if (out.get(i - 1).distance(out.get(i)) > 1.2 && footprintCutM(out.get(i - 1), out.get(i)) > 1.05) {
                return null;
            }
        }
        return out.size() >= 3 ? out : null;
    }

    private static Coordinate closerToPoly(Polygon poly, Coordinate a, Coordinate b) {
        if (poly == null) {
            return null;
        }
        try {
            double da = poly.distance(poly.getFactory().createPoint(a));
            double db = poly.distance(poly.getFactory().createPoint(b));
            return da <= db ? a : b;
        } catch (RuntimeException e) {
            return a;
        }
    }

    private Polygon largestFoot(Geometry geometry) {
        List<Polygon> polys = new ArrayList<>();
        collectPolygons(geometry, polys);
        Polygon best = null;
        for (Polygon p : polys) {
            if (p == null || p.isEmpty()) {
                continue;
            }
            if (best == null || p.getArea() > best.getArea()) {
                best = p;
            }
        }
        return best;
    }

    private static List<Coordinate> shorterArc(LineString ring, Coordinate enter, Coordinate leave) {
        if (ring == null || ring.getNumPoints() < 2) {
            return null;
        }
        LengthIndexedLine indexed = new LengthIndexedLine(ring);
        double len = indexed.getEndIndex();
        if (len < 1) {
            return null;
        }
        double i0 = indexed.indexOf(enter);
        double i1 = indexed.indexOf(leave);
        double forward = i1 >= i0 ? i1 - i0 : (len - i0) + i1;
        double back = len - forward;
        boolean useForward = forward <= back + 0.05;
        Geometry extracted;
        boolean reverse = false;
        if (useForward) {
            extracted = i0 <= i1 ? indexed.extractLine(i0, i1) : joinExtract(indexed, i0, len, 0, i1);
        } else {
            reverse = true;
            extracted = i1 <= i0 ? indexed.extractLine(i1, i0) : joinExtract(indexed, i1, len, 0, i0);
        }
        if (extracted == null || extracted.isEmpty()) {
            return null;
        }
        Coordinate[] raw = extracted.getCoordinates();
        List<Coordinate> arc = new ArrayList<>();
        if (reverse) {
            for (int i = raw.length - 1; i >= 0; i--) {
                if (raw[i] != null) {
                    arc.add(new Coordinate(raw[i]));
                }
            }
        } else {
            for (Coordinate p : raw) {
                if (p != null) {
                    arc.add(new Coordinate(p));
                }
            }
        }
        return arc.size() >= 2 ? arc : null;
    }

    private static Geometry joinExtract(LengthIndexedLine indexed, double a0, double a1, double b0, double b1) {
        Geometry first = indexed.extractLine(a0, a1);
        Geometry second = indexed.extractLine(b0, b1);
        if (first == null || first.isEmpty()) {
            return second;
        }
        if (second == null || second.isEmpty()) {
            return first;
        }
        Coordinate[] fa = first.getCoordinates();
        Coordinate[] sa = second.getCoordinates();
        List<Coordinate> all = new ArrayList<>();
        for (Coordinate p : fa) {
            if (p != null) {
                all.add(new Coordinate(p));
            }
        }
        for (Coordinate p : sa) {
            if (p != null && (all.isEmpty() || all.get(all.size() - 1).distance(p) >= 0.05)) {
                all.add(new Coordinate(p));
            }
        }
        if (all.size() < 2) {
            return first;
        }
        return first.getFactory().createLineString(all.toArray(Coordinate[]::new));
    }

    /** Выход идёт вдоль ближайшей длинной стены, а не поперёк неё. */
    public boolean exitParallelToNearestWall(Coordinate origin, Coordinate exit) {
        Coordinate wall = nearestWallAxis(origin);
        if (wall == null || origin == null || exit == null || origin.distance(exit) < 1) {
            return false;
        }
        return SpecialLayer.crossingAngleDeg(origin, exit, wall) < 28;
    }

    /** Направление ближайшей длинной стены кадастрового контура. */
    private Coordinate nearestWallAxis(Coordinate origin) {
        Polygon host = footprintAt(origin);
        if (host == null || origin == null) {
            return null;
        }
        Coordinate best = null;
        double bestD = Double.POSITIVE_INFINITY;
        Coordinate[] ring = host.getExteriorRing().getCoordinates();
        for (int i = 0; i < ring.length - 1; i++) {
            Coordinate p = ring[i];
            Coordinate q = ring[i + 1];
            if (p == null || q == null || p.distance(q) < 8) {
                continue;
            }
            double d = distPointSeg(origin, p, q);
            if (d < bestD) {
                bestD = d;
                best = new Coordinate(q.x - p.x, q.y - p.y);
            }
        }
        return best;
    }

    /**
     * Длинный участок идёт вдоль фасада: оба конца на одном расстоянии от длинной стены.
     * Такое ребро — не пересечение проезжей, даже если ось синтезированной полосы смотрит поперёк.
     */
    public boolean runsAlongFacade(Coordinate a, Coordinate b) {
        return parallelFacade(a, b) != null;
    }

    /**
     * Сдвиг длинного хода с проезжей на тротуар, параллельно фасаду.
     * Пустой список — ребро и так не вдоль стены в проезжей.
     */
    public List<Coordinate> liftOffCarriage(List<Coordinate> pts) {
        if (pts == null || pts.size() < 2 || special == null) {
            return List.of();
        }
        Coordinate a = pts.get(0);
        Coordinate b = pts.get(pts.size() - 1);
        Facade face = parallelFacade(a, b);
        if (face == null) {
            return List.of();
        }
        int inside = 0;
        int samples = 0;
        for (int i = 1; i < pts.size(); i++) {
            for (int t = 0; t <= 2; t++) {
                double u = t / 2.0;
                Coordinate q = new Coordinate(
                        pts.get(i - 1).x + u * (pts.get(i).x - pts.get(i - 1).x),
                        pts.get(i - 1).y + u * (pts.get(i).y - pts.get(i - 1).y));
                samples++;
                if (inRoad(q)) {
                    inside++;
                }
            }
        }
        if (samples == 0 || inside * 2 < samples) {
            return List.of();
        }
        Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        Coordinate out = face.toward(mid);
        Coordinate back = new Coordinate(-out.x, -out.y);
        double dOut = shiftClear(pts, out);
        double dBack = shiftClear(pts, back);
        Coordinate dir = dOut <= dBack ? out : back;
        double d = Math.min(dOut, dBack);
        if (!Double.isFinite(d) || d < 0.4) {
            return List.of();
        }
        List<Coordinate> shifted = new ArrayList<>();
        for (Coordinate c : pts) {
            shifted.add(new Coordinate(c.x + dir.x * d, c.y + dir.y * d));
        }
        for (int i = 1; i < shifted.size(); i++) {
            if (segmentHitsAvoid(shifted.get(i - 1), shifted.get(i), 0, true)) {
                return List.of();
            }
        }
        return shifted;
    }

    private double shiftClear(List<Coordinate> pts, Coordinate dir) {
        double need = 0;
        for (int i = 0; i < pts.size(); i++) {
            double s = escapeRoad(pts.get(i), dir);
            if (!Double.isFinite(s)) {
                return Double.POSITIVE_INFINITY;
            }
            need = Math.max(need, s);
            if (i == 0) {
                continue;
            }
            Coordinate mid = new Coordinate(
                    (pts.get(i - 1).x + pts.get(i).x) * 0.5,
                    (pts.get(i - 1).y + pts.get(i).y) * 0.5);
            s = escapeRoad(mid, dir);
            if (!Double.isFinite(s)) {
                return Double.POSITIVE_INFINITY;
            }
            need = Math.max(need, s);
        }
        return need;
    }

    private double escapeRoad(Coordinate c, Coordinate dir) {
        if (c == null) {
            return Double.POSITIVE_INFINITY;
        }
        if (!inRoad(c) && !blocked(c)) {
            return 0;
        }
        for (double d = 0.8; d <= 16.0; d += 0.65) {
            Coordinate q = new Coordinate(c.x + dir.x * d, c.y + dir.y * d);
            if (!inRoad(q) && !blocked(q)) {
                return d;
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    private Facade parallelFacade(Coordinate a, Coordinate b) {
        if (a == null || b == null || footprints.isEmpty()) {
            return null;
        }
        double len = a.distance(b);
        if (len < 18) {
            return null;
        }
        Facade best = null;
        double bestD = Double.POSITIVE_INFINITY;
        for (Polygon poly : footprints) {
            if (poly == null || poly.getExteriorRing() == null) {
                continue;
            }
            Coordinate[] ring = poly.getExteriorRing().getCoordinates();
            for (int i = 0; i < ring.length - 1; i++) {
                Coordinate p = ring[i];
                Coordinate q = ring[i + 1];
                if (p == null || q == null || p.distance(q) < 16) {
                    continue;
                }
                double ang = SpecialLayer.crossingAngleDeg(a, b, new Coordinate(q.x - p.x, q.y - p.y));
                if (ang > 16) {
                    continue;
                }
                double da = distPointSeg(a, p, q);
                double db = distPointSeg(b, p, q);
                if (Math.abs(da - db) > 6) {
                    continue;
                }
                double near = Math.min(da, db);
                if (near > 28 || near >= bestD) {
                    continue;
                }
                bestD = near;
                best = new Facade(p, q);
            }
        }
        return best;
    }

    private static double distPointSeg(Coordinate c, Coordinate p, Coordinate q) {
        double vx = q.x - p.x;
        double vy = q.y - p.y;
        double len2 = vx * vx + vy * vy;
        if (len2 < 1e-9) {
            return c.distance(p);
        }
        double t = ((c.x - p.x) * vx + (c.y - p.y) * vy) / len2;
        t = Math.max(0, Math.min(1, t));
        return c.distance(new Coordinate(p.x + t * vx, p.y + t * vy));
    }

    private static final class Facade {
        final Coordinate p;
        final Coordinate q;

        Facade(Coordinate p, Coordinate q) {
            this.p = p;
            this.q = q;
        }

        Coordinate toward(Coordinate mid) {
            double vx = q.x - p.x;
            double vy = q.y - p.y;
            double n = Math.hypot(vx, vy);
            Coordinate normal = new Coordinate(-vy / n, vx / n);
            double side = (mid.x - p.x) * normal.x + (mid.y - p.y) * normal.y;
            if (side < 0) {
                normal.x = -normal.x;
                normal.y = -normal.y;
            }
            return normal;
        }
    }

    private static final class Side {
        Coordinate hit;
        Coordinate dir;
        Coordinate along;
        double wall;
        double edge;
    }

    /**
     * Два выхода: ⊥ одной стене и ⊥ другой, каждый — короткий луч до фасада.
     * Порт на каркасе выбирает тот, с которого есть путь до сети.
     */
    public List<Coordinate> wallPerpExits(Coordinate origin, double extraOut) {
        List<Coordinate> sides = sideExits(origin, extraOut);
        if (!sides.isEmpty()) {
            return sides;
        }
        List<Coordinate> out = new ArrayList<>();
        if (origin == null) {
            return out;
        }
        if (!blocked(origin)) {
            Coordinate onFacade = facadeExit(origin, extraOut);
            out.add(onFacade != null ? onFacade : new Coordinate(origin));
            return out;
        }
        Prepared host = containing(origin);
        if (host == null) {
            Coordinate free = nearestFree(origin, 80);
            if (free != null) {
                out.add(free);
            }
            return out;
        }
        Coordinate interior;
        try {
            interior = host.geom.getInteriorPoint().getCoordinate();
        } catch (RuntimeException e) {
            interior = host.geom.getCentroid().getCoordinate();
        }
        Coordinate[] axes = wallAxes(host.geom, origin);
        for (Coordinate axis : axes) {
            if (axis == null) {
                continue;
            }
            Coordinate best = null;
            double bestD = Double.POSITIVE_INFINITY;
            for (int sign : new int[]{1, -1}) {
                Coordinate dir = new Coordinate(sign * axis.x, sign * axis.y);
                Coordinate hit = rayHitBoundary(origin, dir, host);
                if (hit == null) {
                    continue;
                }
                double indoor = origin.distance(hit);
                if (indoor < 0.35 || indoor > 80 || indoor >= bestD) {
                    continue;
                }
                Coordinate q = pushOut(hit, interior, extraOut);
                if (q == null || blocked(q)) {
                    q = pushOut(hit, interior, extraOut + 1.1);
                }
                if (q == null || blocked(q) || !onOuterFacade(host.geom, q)) {
                    continue;
                }
                bestD = indoor;
                best = q;
            }
            if (best != null) {
                out.add(best);
            }
        }
        if (out.isEmpty()) {
            Coordinate picked = pickWallPerpExit(host, origin, null, extraOut);
            if (picked != null) {
                out.add(picked);
            }
        }
        return out;
    }
    public Coordinate facadeExit(Coordinate origin, double extraOut) {
        if (origin == null) {
            return null;
        }
        List<Coordinate> candidates = new ArrayList<>();
        addStreetExits(origin, extraOut, candidates);
        Coordinate best = null;
        double bestS = Double.POSITIVE_INFINITY;
        for (Coordinate q : candidates) {
            if (q == null || blocked(q) || inRoad(q)) {
                continue;
            }
            double d = origin.distance(q);
            if (d < 0.4) {
                return new Coordinate(origin);
            }
            if (d > 36) {
                continue;
            }
            double s = exitScore(origin, q, null);
            if (s < bestS) {
                bestS = s;
                best = q;
            }
        }
        if (best != null && origin.distance(best) <= 8) {
            return new Coordinate(origin);
        }
        return best;
    }

    private void addStreetExits(Coordinate origin, double extraOut, List<Coordinate> candidates) {
        if (origin == null || special == null) {
            return;
        }
        SpecialLayer.Corridor cor = special.nearestCorridor(origin, 90);
        if (cor == null || cor.axis == null) {
            return;
        }
        double n = Math.hypot(cor.axis.x, cor.axis.y);
        if (n < 1e-9) {
            return;
        }
        Coordinate u = new Coordinate(cor.axis.x / n, cor.axis.y / n);
        Coordinate v = new Coordinate(-u.y, u.x);
        for (int s : new int[]{1, -1}) {
            for (double d = 0.6; d <= 28; d += 0.45) {
                Coordinate q = new Coordinate(origin.x + s * d * v.x, origin.y + s * d * v.y);
                if (!blocked(q) && !inRoad(q)) {
                    candidates.add(q);
                    break;
                }
            }
            for (double d = 0.6; d <= 16; d += 0.45) {
                Coordinate q = new Coordinate(origin.x + s * d * u.x, origin.y + s * d * u.y);
                if (!blocked(q) && !inRoad(q)) {
                    candidates.add(q);
                    break;
                }
            }
        }
    }

    /**
     * Если отрезок режет корпус — обойти по смещённой границе, не через угол двора.
     */
    public List<Coordinate> hugAround(Coordinate a, Coordinate b) {
        return hugAround(a, b, true);
    }

    /**
     * @param interiorOnly true — только протыкание корпуса (каркас/Дейкстра);
     *                     false — ещё и касание границы (выдача трубы).
     */
    public List<Coordinate> hugAround(Coordinate a, Coordinate b, boolean interiorOnly) {
        if (a == null || b == null) {
            return null;
        }
        if (!segmentHitsAvoid(a, b, 0, interiorOnly)) {
            return null;
        }
        LineString ls = gf.createLineString(new Coordinate[]{new Coordinate(a), new Coordinate(b)});
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        double cap = interiorOnly
                ? Math.min(220, a.distance(b) * 4 + 48)
                : Math.min(720, a.distance(b) * 5 + 80);
        for (Prepared p : queryAvoids(ls.getEnvelopeInternal())) {
            if (p == null || p.geom == null || !p.prepared.intersects(ls)) {
                continue;
            }
            List<Coordinate> hug = walkRing(p.geom, a, b, !interiorOnly);
            if (hug == null || hug.size() < 2) {
                continue;
            }
            double len = 0;
            boolean ok = true;
            for (int i = 1; i < hug.size(); i++) {
                Coordinate u = hug.get(i - 1);
                Coordinate v = hug.get(i);
                len += u.distance(v);
                if (len > cap || (u.distance(v) > 8 && segmentHitsAvoid(u, v, 0, interiorOnly))) {
                    ok = false;
                    break;
                }
            }
            if (ok && len < bestLen) {
                bestLen = len;
                best = hug;
            }
        }
        return best;
    }

    private List<Coordinate> walkRing(Geometry geom, Coordinate a, Coordinate b, boolean streetApprox) {
        if (geom == null || geom.isEmpty()) {
            return null;
        }
        Coordinate interior;
        try {
            interior = geom.getInteriorPoint().getCoordinate();
        } catch (RuntimeException e) {
            interior = geom.getCentroid().getCoordinate();
        }
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        if (streetApprox) {
            for (double off : new double[]{1.6, 2.4, 3.2}) {
                Polygon obb = streetAlignedObb(geom, off, a, b);
                if (obb == null || obb.isEmpty()) {
                    continue;
                }
                List<Coordinate> cand = traceRing(obb.getExteriorRing().getCoordinates(), interior, a, b,
                        80, true);
                if (cand != null) {
                    double len = pathLen(cand);
                    if (len < bestLen) {
                        bestLen = len;
                        best = cand;
                    }
                }
            }
            if (best != null) {
                return best;
            }
        }
        Geometry buf;
        try {
            buf = geom.buffer(streetApprox ? 1.6 : 1.25, 2);
        } catch (RuntimeException e) {
            buf = geom;
        }
        List<Polygon> polys = new ArrayList<>();
        collectPolygons(buf, polys);
        for (Polygon poly : polys) {
            if (poly == null || poly.isEmpty()) {
                continue;
            }
            List<Coordinate> cand = traceRing(poly.getExteriorRing().getCoordinates(), interior, a, b,
                    4, streetApprox);
            if (cand == null) {
                continue;
            }
            double len = pathLen(cand);
            if (len < bestLen) {
                bestLen = len;
                best = cand;
            }
        }
        return best;
    }

    /**
     * Улично-ориентированный прямоугольник корпуса: на выдаче труба идёт по
     * четырём сторонам OBB, а не по зубчатому кадастру.
     */
    private Polygon streetAlignedObb(Geometry geom, double offset, Coordinate alongA, Coordinate alongB) {
        if (geom == null || geom.isEmpty()) {
            return null;
        }
        Coordinate o;
        try {
            o = geom.getCentroid().getCoordinate();
        } catch (RuntimeException e) {
            return null;
        }
        Coordinate hint = o;
        if (alongA != null && alongB != null) {
            hint = new Coordinate((alongA.x + alongB.x) * 0.5, (alongA.y + alongB.y) * 0.5);
        }
        Coordinate u = new Coordinate(1, 0);
        if (special != null) {
            double bestArea = Double.POSITIVE_INFINITY;
            for (Coordinate axis : special.dominantAxes()) {
                if (axis == null) {
                    continue;
                }
                double n = Math.hypot(axis.x, axis.y);
                if (n < 1e-9) {
                    continue;
                }
                Coordinate cand = new Coordinate(axis.x / n, axis.y / n);
                double area = obbArea(geom, cand);
                if (area < bestArea) {
                    bestArea = area;
                    u = cand;
                }
            }
            if (bestArea == Double.POSITIVE_INFINITY) {
                SpecialLayer.Corridor cor = special.nearestCorridor(hint, 140);
                if (cor == null || cor.axis == null) {
                    cor = special.nearestCorridor(o, 140);
                }
                if (cor != null && cor.axis != null) {
                    double n = Math.hypot(cor.axis.x, cor.axis.y);
                    if (n > 1e-9) {
                        u = new Coordinate(cor.axis.x / n, cor.axis.y / n);
                    }
                }
            }
        }
        Coordinate v = new Coordinate(-u.y, u.x);
        Coordinate[] pts = geom.getCoordinates();
        boolean any = false;
        double t0 = 0;
        double t1 = 0;
        double s0 = 0;
        double s1 = 0;
        for (Coordinate p : pts) {
            if (p == null) {
                continue;
            }
            double t = (p.x - o.x) * u.x + (p.y - o.y) * u.y;
            double s = (p.x - o.x) * v.x + (p.y - o.y) * v.y;
            if (!any) {
                t0 = t1 = t;
                s0 = s1 = s;
                any = true;
            } else {
                t0 = Math.min(t0, t);
                t1 = Math.max(t1, t);
                s0 = Math.min(s0, s);
                s1 = Math.max(s1, s);
            }
        }
        if (!any) {
            return null;
        }
        t0 -= offset;
        t1 += offset;
        s0 -= offset;
        s1 += offset;
        Coordinate aa = new Coordinate(o.x + t0 * u.x + s0 * v.x, o.y + t0 * u.y + s0 * v.y);
        Coordinate bb = new Coordinate(o.x + t1 * u.x + s0 * v.x, o.y + t1 * u.y + s0 * v.y);
        Coordinate cc = new Coordinate(o.x + t1 * u.x + s1 * v.x, o.y + t1 * u.y + s1 * v.y);
        Coordinate dd = new Coordinate(o.x + t0 * u.x + s1 * v.x, o.y + t0 * u.y + s1 * v.y);
        try {
            return gf.createPolygon(new Coordinate[]{aa, bb, cc, dd, new Coordinate(aa)});
        } catch (RuntimeException e) {
            return null;
        }
    }

    private static double obbArea(Geometry geom, Coordinate u) {
        if (geom == null || u == null) {
            return Double.POSITIVE_INFINITY;
        }
        Coordinate o;
        try {
            o = geom.getCentroid().getCoordinate();
        } catch (RuntimeException e) {
            return Double.POSITIVE_INFINITY;
        }
        Coordinate v = new Coordinate(-u.y, u.x);
        boolean any = false;
        double t0 = 0;
        double t1 = 0;
        double s0 = 0;
        double s1 = 0;
        for (Coordinate p : geom.getCoordinates()) {
            if (p == null) {
                continue;
            }
            double t = (p.x - o.x) * u.x + (p.y - o.y) * u.y;
            double s = (p.x - o.x) * v.x + (p.y - o.y) * v.y;
            if (!any) {
                t0 = t1 = t;
                s0 = s1 = s;
                any = true;
            } else {
                t0 = Math.min(t0, t);
                t1 = Math.max(t1, t);
                s0 = Math.min(s0, s);
                s1 = Math.max(s1, s);
            }
        }
        if (!any) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.max(1e-3, t1 - t0) * Math.max(1e-3, s1 - s0);
    }

    private List<Coordinate> traceRing(Coordinate[] ringCoords, Coordinate interior,
                                       Coordinate a, Coordinate b, double step, boolean rejectTouch) {
        List<Coordinate> ring = densifyRing(ringCoords, step);
        if (ring.size() < 4) {
            return null;
        }
        List<Coordinate> walkable = new ArrayList<>();
        for (Coordinate raw : ring) {
            Coordinate q = pushOut(raw, interior, 0.35);
            if (q == null || blocked(q)) {
                q = pushOut(raw, interior, 1.1);
            }
            if (q != null && !blocked(q) && (walkable.isEmpty()
                    || walkable.get(walkable.size() - 1).distance(q) >= 0.8)) {
                walkable.add(q);
            }
        }
        if (walkable.size() < 4) {
            return null;
        }
        int ia = nearestIndex(walkable, a);
        int ib = nearestIndex(walkable, b);
        if (ia < 0 || ib < 0) {
            return null;
        }
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        boolean interiorOnly = !rejectTouch;
        for (int dir : new int[]{1, -1}) {
            List<Coordinate> arc = arc(walkable, ia, ib, dir);
            if (arc == null || arc.size() < 2) {
                continue;
            }
            if (rejectTouch && ringHits(arc, true)) {
                continue;
            }
            List<Coordinate> path = new ArrayList<>();
            path.add(new Coordinate(a));
            if (!appendJoin(path, arc.get(0), rejectTouch)) {
                continue;
            }
            for (int i = 1; i < arc.size(); i++) {
                Coordinate q = arc.get(i);
                if (path.get(path.size() - 1).distance(q) >= 0.45) {
                    path.add(new Coordinate(q));
                }
            }
            if (!appendJoin(path, b, rejectTouch)) {
                continue;
            }
            if (path.size() < 2) {
                continue;
            }
            if (!rejectTouch) {
                Coordinate first = path.get(1);
                Coordinate last = path.get(path.size() - 2);
                if (a.distance(first) > 10 && segmentHitsAvoid(a, first, 0, interiorOnly)) {
                    continue;
                }
                if (last.distance(b) > 10 && segmentHitsAvoid(last, b, 0, interiorOnly)) {
                    continue;
                }
            }
            double len = pathLen(path);
            if (len < bestLen) {
                bestLen = len;
                best = path;
            }
        }
        return best;
    }

    private boolean appendJoin(List<Coordinate> path, Coordinate to, boolean rejectTouch) {
        if (path == null || path.isEmpty() || to == null) {
            return false;
        }
        Coordinate from = path.get(path.size() - 1);
        if (from.distance(to) < 0.45) {
            return true;
        }
        if (!rejectTouch) {
            if (from.distance(to) > 10 && segmentHitsAvoid(from, to, 0, true)) {
                return false;
            }
            path.add(new Coordinate(to));
            return true;
        }
        List<Coordinate> join = streetJoin(from, to, true);
        if (join == null || join.size() < 2) {
            if (from.distance(to) > 10) {
                return false;
            }
            path.add(new Coordinate(to));
            return true;
        }
        for (int i = 1; i < join.size(); i++) {
            Coordinate q = join.get(i);
            if (q != null && path.get(path.size() - 1).distance(q) >= 0.45) {
                path.add(new Coordinate(q));
            }
        }
        return path.get(path.size() - 1).distance(to) < 0.5;
    }

    private List<Coordinate> streetJoin(Coordinate a, Coordinate b, boolean rejectTouch) {
        if (a == null || b == null) {
            return null;
        }
        boolean interiorOnly = !rejectTouch;
        Coordinate u = new Coordinate(1, 0);
        if (special != null) {
            Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
            SpecialLayer.Corridor c = special.nearestCorridor(mid, 120);
            if (c == null || c.axis == null) {
                c = special.nearestCorridor(a, 120);
            }
            if (c != null && c.axis != null) {
                double n = Math.hypot(c.axis.x, c.axis.y);
                if (n > 1e-9) {
                    u = new Coordinate(c.axis.x / n, c.axis.y / n);
                }
            }
        }
        double ang = SpecialLayer.crossingAngleDeg(a, b, u);
        if ((ang <= 14 || ang >= 76) && !segmentHitsAvoid(a, b, 0, interiorOnly)) {
            List<Coordinate> direct = new ArrayList<>(2);
            direct.add(new Coordinate(a));
            direct.add(new Coordinate(b));
            return direct;
        }
        Coordinate v = new Coordinate(-u.y, u.x);
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        Coordinate c1 = new Coordinate(a.x + (dx * u.x + dy * u.y) * u.x, a.y + (dx * u.x + dy * u.y) * u.y);
        Coordinate c2 = new Coordinate(a.x + (dx * v.x + dy * v.y) * v.x, a.y + (dx * v.x + dy * v.y) * v.y);
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        for (Coordinate corner : new Coordinate[]{c1, c2}) {
            if (corner.distance(a) < 0.4 || corner.distance(b) < 0.4) {
                continue;
            }
            if (segmentHitsAvoid(a, corner, 0, interiorOnly) || segmentHitsAvoid(corner, b, 0, interiorOnly)) {
                continue;
            }
            double len = a.distance(corner) + corner.distance(b);
            if (len < bestLen) {
                bestLen = len;
                List<Coordinate> p = new ArrayList<>(3);
                p.add(new Coordinate(a));
                p.add(new Coordinate(corner));
                p.add(new Coordinate(b));
                best = p;
            }
        }
        return best;
    }

    private static double pathLen(List<Coordinate> path) {
        double len = 0;
        for (int i = 1; i < path.size(); i++) {
            len += path.get(i - 1).distance(path.get(i));
        }
        return len;
    }

    private boolean ringHits(List<Coordinate> path, boolean rejectTouch) {
        if (path == null || path.size() < 2) {
            return true;
        }
        boolean interiorOnly = !rejectTouch;
        for (int i = 1; i < path.size(); i++) {
            Coordinate u = path.get(i - 1);
            Coordinate v = path.get(i);
            if (u.distance(v) > 2.5 && segmentHitsAvoid(u, v, 0, interiorOnly)) {
                return true;
            }
        }
        return false;
    }

    private static List<Coordinate> densifyRing(Coordinate[] ring, double step) {
        List<Coordinate> out = new ArrayList<>();
        if (ring == null || ring.length < 2) {
            return out;
        }
        int n = ring.length;
        if (n >= 2 && ring[0].distance(ring[n - 1]) < 1e-6) {
            n--;
        }
        for (int i = 0; i < n; i++) {
            Coordinate a = ring[i];
            Coordinate b = ring[(i + 1) % n];
            if (a == null || b == null) {
                continue;
            }
            out.add(new Coordinate(a));
            double d = a.distance(b);
            int k = Math.max(1, (int) Math.floor(d / Math.max(1.5, step)));
            for (int t = 1; t < k; t++) {
                double f = t / (double) k;
                out.add(new Coordinate(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y)));
            }
        }
        return out;
    }

    private static int nearestIndex(List<Coordinate> pts, Coordinate c) {
        int best = -1;
        double bestD = Double.POSITIVE_INFINITY;
        for (int i = 0; i < pts.size(); i++) {
            double d = pts.get(i).distance(c);
            if (d < bestD) {
                bestD = d;
                best = i;
            }
        }
        return best;
    }

    private static List<Coordinate> arc(List<Coordinate> ring, int from, int to, int dir) {
        List<Coordinate> out = new ArrayList<>();
        int n = ring.size();
        int cur = from;
        int guard = 0;
        out.add(ring.get(from));
        while (cur != to && guard++ <= n + 2) {
            cur = Math.floorMod(cur + dir, n);
            out.add(ring.get(cur));
        }
        return out.size() >= 2 ? out : null;
    }

    /**
     * Короткий выход: перпендикуляр к улице (лицо фасада), не торец/гибл.
     * Направление toward только как сторона света, без тяги к далёкой сети.
     */
    private double exitScore(Coordinate origin, Coordinate q, Coordinate toward) {
        double s = origin.distance(q);
        if (inRoad(q)) {
            s += 40;
        }
        SpecialLayer.Corridor near = special.nearestCorridor(q, sidewalkM() + 16);
        if (near == null) {
            s += 30;
        } else if (near.axis != null) {
            double ang = SpecialLayer.crossingAngleDeg(origin, q, near.axis);
            if (ang <= 16) {
                s += 18;
            } else if (ang < 74) {
                s += 22;
            }
        }
        if (toward != null) {
            double dx = toward.x - origin.x;
            double dy = toward.y - origin.y;
            double qx = q.x - origin.x;
            double qy = q.y - origin.y;
            double tn = Math.hypot(dx, dy);
            double qn = Math.hypot(qx, qy);
            if (tn > 1e-6 && qn > 1e-6 && (dx * qx + dy * qy) / (tn * qn) < 0.2) {
                s += 16;
            }
        }
        return s;
    }

    public List<Polygon> avoidPolygons() {
        List<Polygon> list = new ArrayList<>();
        for (Prepared a : avoids) {
            collectPolygons(a.geom, list);
        }
        return list;
    }

    /** Слитые кварталы (корпуса), без парков и прочих запретов. */
    public List<Polygon> blockPolygons() {
        return new ArrayList<>(blocks);
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
