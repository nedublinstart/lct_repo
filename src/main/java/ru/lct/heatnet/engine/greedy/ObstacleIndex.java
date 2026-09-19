package ru.lct.heatnet.engine.greedy;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
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
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.HeatSource;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

public final class ObstacleIndex {

    private final GeometryFactory gf = GeoJsonGeometries.GF;
    private final List<Prepared> avoids = new ArrayList<>();
    private final List<Prepared> costlies = new ArrayList<>();
    private final List<PreparedGeometry> allows = new ArrayList<>();
    private final STRtree avoidTree = new STRtree();

    public static ObstacleIndex build(Scene scene) {
        ObstacleIndex index = new ObstacleIndex();
        for (SpatialConstraint c : scene.constraints) {
            if (c.geometry == null || c.rule == null) {
                continue;
            }
            Geometry g = c.geometry;
            if (c.rule.bufferM > 0 && c.rule.avoid()) {
                try {
                    Geometry buffered = g.buffer(c.rule.bufferM);
                    if (buffered != null) {
                        g = buffered;
                    }
                } catch (RuntimeException ignored) {
                    // keep original
                }
            }
            Prepared p = new Prepared();
            p.raw = c;
            p.geom = g;
            p.prepared = PreparedGeometryFactory.prepare(g);
            if (c.rule.avoid()) {
                index.avoids.add(p);
                index.avoidTree.insert(g.getEnvelopeInternal(), p);
            } else if (c.rule.cross() || c.rule.special()) {
                index.costlies.add(p);
            }
        }
        index.avoidTree.build();
        for (ExistingSegment seg : scene.segments) {
            if (seg.line != null) {
                index.allowGeometry(seg.line.buffer(6.0));
            }
        }
        for (Chamber ch : scene.chambers) {
            if (ch.point != null) {
                index.allowGeometry(ch.point.buffer(4.0));
            }
        }
        for (HeatSource src : scene.sources) {
            if (src.point != null) {
                index.allowGeometry(src.point.buffer(4.0));
            }
        }
        return index;
    }

    public void allowCoordinates(Collection<Coordinate> coordinates, double radius) {
        for (Coordinate c : coordinates) {
            if (c != null) {
                allowGeometry(gf.createPoint(c).buffer(Math.max(1.5, radius)));
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
        Point p = gf.createPoint(c);
        if (allowed(p)) {
            return false;
        }
        return hitsAvoid(p);
    }

    public int extra(Coordinate c) {
        Point p = gf.createPoint(c);
        int sum = 0;
        for (Prepared a : costlies) {
            if (a.prepared.intersects(p) || a.geom.distance(p) < 0.5) {
                sum += Math.max(1, a.raw.rule.extraGridCost);
            }
        }
        return sum;
    }

    public boolean segmentHitsAvoid(Coordinate a, Coordinate b) {
        LineString ls = gf.createLineString(new Coordinate[]{new Coordinate(a), new Coordinate(b)});
        return hitsAvoid(ls);
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
        List<Coordinate> free = sampleFree(c, maxRadius);
        if (free.isEmpty()) {
            return nearestFree(c, maxRadius);
        }
        Coordinate best = free.get(0);
        double bestS = Double.POSITIVE_INFINITY;
        for (Coordinate p : free) {
            double toTarget = target == null ? 0 : p.distance(target);
            double s = toTarget + 0.25 * c.distance(p);
            if (s < bestS) {
                bestS = s;
                best = p;
            }
        }
        return best;
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
            Geometry ring;
            try {
                ring = a.geom.buffer(2.0);
            } catch (RuntimeException e) {
                ring = a.geom;
            }
            Coordinate interior;
            try {
                interior = a.geom.getInteriorPoint().getCoordinate();
            } catch (RuntimeException e) {
                interior = a.geom.getCentroid().getCoordinate();
            }
            Coordinate[] pts = ring.getCoordinates();
            int step = Math.max(1, pts.length / 120);
            for (int i = 0; i < pts.length; i += step) {
                Coordinate q = pts[i];
                double vx = q.x - interior.x;
                double vy = q.y - interior.y;
                double n = Math.hypot(vx, vy);
                if (n < 1e-6) {
                    continue;
                }
                for (double extra : new double[]{0.5, 2.5, 5.0, 9.0, 14.0}) {
                    Coordinate o = new Coordinate(q.x + extra * vx / n, q.y + extra * vy / n);
                    if (!blocked(o) && c.distance(o) <= maxRadius + 20) {
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

    public List<Polygon> avoidPolygons() {
        List<Polygon> list = new ArrayList<>();
        for (Prepared a : avoids) {
            if (a.geom instanceof Polygon) {
                list.add((Polygon) a.geom);
            }
        }
        return list;
    }

    private boolean allowed(Geometry g) {
        for (PreparedGeometry a : allows) {
            if (a.intersects(g)) {
                return true;
            }
        }
        return false;
    }

    private boolean hitsAvoid(Geometry g) {
        Envelope env = g.getEnvelopeInternal();
        for (Prepared a : queryAvoids(env)) {
            if (a.prepared.intersects(g)) {
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

    private static final class Prepared {
        SpatialConstraint raw;
        Geometry geom;
        PreparedGeometry prepared;
    }
}
