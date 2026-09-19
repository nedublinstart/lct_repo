package ru.lct.heatnet.geo;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.CoordinateFilter;
import org.locationtech.jts.geom.Geometry;

/**
 * Расчёты длин и буферов — в EPSG:32637 (UTM zone 37N), как требует приложение.
 * Вход/выход GeoJSON остаются WGS 84.
 */
public final class CrsProjector {

    private static final double A = 6_378_137.0;
    private static final double F = 1.0 / 298.257223563;
    private static final double K0 = 0.9996;
    private static final double LON0 = Math.toRadians(39.0);
    private static final double E0 = 500_000.0;
    private static final double E2 = F * (2.0 - F);
    private static final double EP2 = E2 / (1.0 - E2);

    private final boolean geographic;

    private CrsProjector(boolean geographic) {
        this.geographic = geographic;
    }

    public static CrsProjector detect(Iterable<Geometry> geometries) {
        double minX = Double.POSITIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;
        int n = 0;
        for (Geometry g : geometries) {
            if (g == null || g.isEmpty()) {
                continue;
            }
            minX = Math.min(minX, g.getEnvelopeInternal().getMinX());
            minY = Math.min(minY, g.getEnvelopeInternal().getMinY());
            maxX = Math.max(maxX, g.getEnvelopeInternal().getMaxX());
            maxY = Math.max(maxY, g.getEnvelopeInternal().getMaxY());
            n++;
        }
        if (n == 0) {
            return new CrsProjector(false);
        }
        boolean geo = Math.abs(minX) <= 180 && Math.abs(maxX) <= 180
                && Math.abs(minY) <= 90 && Math.abs(maxY) <= 90
                && (maxX - minX) < 15 && (maxY - minY) < 15;
        return new CrsProjector(geo);
    }

    public boolean isGeographic() {
        return geographic;
    }

    public Geometry toMeters(Geometry geometry) {
        if (geometry == null) {
            return null;
        }
        Geometry copy = geometry.copy();
        if (!geographic) {
            return copy;
        }
        copy.apply((CoordinateFilter) c -> {
            Coordinate u = toUtm(c.x, c.y);
            c.setX(u.x);
            c.setY(u.y);
        });
        copy.geometryChanged();
        return copy;
    }

    public Geometry toLonLat(Geometry geometry) {
        if (geometry == null) {
            return null;
        }
        Geometry copy = geometry.copy();
        if (!geographic) {
            return copy;
        }
        copy.apply((CoordinateFilter) c -> {
            Coordinate w = toWgs(c.x, c.y);
            c.setX(w.x);
            c.setY(w.y);
        });
        copy.geometryChanged();
        return copy;
    }

    public Coordinate toMeters(Coordinate c) {
        if (!geographic) {
            return new Coordinate(c);
        }
        return toUtm(c.x, c.y);
    }

    static Coordinate toUtm(double lonDeg, double latDeg) {
        double lat = Math.toRadians(latDeg);
        double lon = Math.toRadians(lonDeg);
        double n = A / Math.sqrt(1 - E2 * Math.sin(lat) * Math.sin(lat));
        double t = Math.tan(lat);
        double c = EP2 * Math.cos(lat) * Math.cos(lat);
        double a = Math.cos(lat) * (lon - LON0);
        double m = meridional(lat);
        double t2 = t * t;
        double a2 = a * a;
        double a3 = a2 * a;
        double a4 = a2 * a2;
        double a5 = a4 * a;
        double a6 = a4 * a2;
        double east = K0 * n * (a + (1 - t2 + c) * a3 / 6.0
                + (5 - 18 * t2 + t2 * t2 + 72 * c - 58 * EP2) * a5 / 120.0) + E0;
        double north = K0 * (m + n * t * (a2 / 2.0 + (5 - t2 + 9 * c + 4 * c * c) * a4 / 24.0
                + (61 - 58 * t2 + t2 * t2 + 600 * c - 330 * EP2) * a6 / 720.0));
        return new Coordinate(east, north);
    }

    static Coordinate toWgs(double east, double north) {
        double x = east - E0;
        double mu = north / (K0 * A * (1 - E2 / 4 - 3 * E2 * E2 / 64 - 5 * E2 * E2 * E2 / 256));
        double e1 = (1 - Math.sqrt(1 - E2)) / (1 + Math.sqrt(1 - E2));
        double phi1 = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * Math.sin(2 * mu)
                + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * Math.sin(4 * mu)
                + (151 * e1 * e1 * e1 / 96) * Math.sin(6 * mu);
        double n = A / Math.sqrt(1 - E2 * Math.sin(phi1) * Math.sin(phi1));
        double r = A * (1 - E2) / Math.pow(1 - E2 * Math.sin(phi1) * Math.sin(phi1), 1.5);
        double t = Math.tan(phi1);
        double c = EP2 * Math.cos(phi1) * Math.cos(phi1);
        double d = x / (n * K0);
        double d2 = d * d;
        double d3 = d2 * d;
        double d4 = d2 * d2;
        double d5 = d4 * d;
        double d6 = d4 * d2;
        double lat = phi1 - (n * Math.tan(phi1) / r) * (d2 / 2
                - (5 + 3 * t * t + 10 * c - 4 * c * c - 9 * EP2) * d4 / 24
                + (61 + 90 * t * t + 298 * c + 45 * t * t * t * t - 252 * EP2 - 3 * c * c) * d6 / 720);
        double lon = LON0 + (d - (1 + 2 * t * t + c) * d3 / 6
                + (5 - 2 * c + 28 * t * t - 3 * c * c + 8 * EP2 + 24 * t * t * t * t) * d5 / 120) / Math.cos(phi1);
        return new Coordinate(Math.toDegrees(lon), Math.toDegrees(lat));
    }

    private static double meridional(double lat) {
        double e4 = E2 * E2;
        double e6 = e4 * E2;
        return A * ((1 - E2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * lat
                - (3 * E2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * Math.sin(2 * lat)
                + (15 * e4 / 256 + 45 * e6 / 1024) * Math.sin(4 * lat)
                - (35 * e6 / 3072) * Math.sin(6 * lat));
    }
}
