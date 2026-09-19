package ru.lct.heatnet.engine.greedy;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.linearref.LinearLocation;
import org.locationtech.jts.linearref.LocationIndexedLine;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.HeatSource;
import ru.lct.heatnet.scene.Scene;

public final class NetworkSnapper {

    private final Scene scene;
    private final double chamberSnapM;

    public NetworkSnapper(Scene scene, AppendixModel appendix) {
        this.scene = scene;
        this.chamberSnapM = appendix.getRouting().chamberSnapM;
    }

    public Snap snap(Point from, boolean chambersOnly) {
        Snap best = null;
        double bestD = Double.POSITIVE_INFINITY;
        if (!chambersOnly) {
            for (ExistingSegment seg : scene.segments) {
                LocationIndexedLine lil = new LocationIndexedLine(seg.line);
                LinearLocation loc = lil.project(from.getCoordinate());
                Coordinate c = lil.extractPoint(loc);
                double d = from.getCoordinate().distance(c);
                if (d < bestD) {
                    bestD = d;
                    best = Snap.segment(seg.id, c, seg.dn);
                }
            }
        }
        for (Chamber ch : scene.chambers) {
            double d = from.getCoordinate().distance(ch.point.getCoordinate());
                if (d < bestD) {
                    bestD = d;
                    best = Snap.chamber(ch.id, ch.point.getCoordinate(), ch.dn);
                }
        }
        for (HeatSource src : scene.sources) {
            double d = from.getCoordinate().distance(src.point.getCoordinate());
            if (d < bestD) {
                bestD = d;
                best = Snap.source(src.id, src.point.getCoordinate());
            }
        }
        if (best == null) {
            return null;
        }
        if (!"chamber".equals(best.kind) && !"source".equals(best.kind)) {
            for (Chamber ch : scene.chambers) {
                if (ch.point.getCoordinate().distance(best.coordinate) <= chamberSnapM) {
                    return Snap.chamber(ch.id, ch.point.getCoordinate(), ch.dn);
                }
            }
        }
        return best;
    }

    public static final class Snap {
        public final String objectId;
        public final String kind;
        public final Coordinate coordinate;
        public final Point point;
        public final int dn;

        private Snap(String objectId, String kind, Coordinate coordinate, int dn) {
            this.objectId = objectId;
            this.kind = kind;
            this.coordinate = coordinate;
            this.point = GeoJsonGeometries.GF.createPoint(coordinate);
            this.dn = dn;
        }

        static Snap segment(String id, Coordinate c, int dn) {
            return new Snap(id, "segment", c, dn);
        }

        static Snap chamber(String id, Coordinate c) {
            return chamber(id, c, 0);
        }

        static Snap chamber(String id, Coordinate c, int dn) {
            return new Snap(id, "chamber", c, dn);
        }

        static Snap source(String id, Coordinate c) {
            return new Snap(id, "source", c, 0);
        }
    }
}
