package ru.lct.heatnet.engine.greedy;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.geom.prep.PreparedGeometry;
import org.locationtech.jts.geom.prep.PreparedGeometryFactory;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

public final class ObstacleIndex {

    private final GeometryFactory gf = GeoJsonGeometries.GF;
    private final List<Prepared> avoids = new ArrayList<>();
    private final List<Prepared> costlies = new ArrayList<>();

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
            } else if (c.rule.cross() || c.rule.special()) {
                index.costlies.add(p);
            }
        }
        return index;
    }

    public boolean blocked(Coordinate c) {
        Point p = gf.createPoint(c);
        for (Prepared a : avoids) {
            if (a.prepared.intersects(p)) {
                return true;
            }
        }
        return false;
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
        for (Prepared av : avoids) {
            if (av.prepared.intersects(ls)) {
                return true;
            }
        }
        return false;
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

    public List<Polygon> avoidPolygons() {
        List<Polygon> list = new ArrayList<>();
        for (Prepared a : avoids) {
            if (a.geom instanceof Polygon) {
                list.add((Polygon) a.geom);
            }
        }
        return list;
    }

    private static final class Prepared {
        SpatialConstraint raw;
        Geometry geom;
        PreparedGeometry prepared;
    }
}
