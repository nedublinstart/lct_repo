package ru.lct.heatnet.geo;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.CoordinateFilter;
import org.locationtech.jts.geom.Geometry;

/**
 * Если координаты похожи на lon/lat — переводим в местные метры для расчёта
 * и обратно при выгрузке. Иначе считаем, что уже метры.
 */
public final class CrsProjector {

    private final boolean geographic;
    private final double lon0;
    private final double lat0;
    private final double metersPerLon;
    private final double metersPerLat = 111_320.0;

    private CrsProjector(boolean geographic, double lon0, double lat0) {
        this.geographic = geographic;
        this.lon0 = lon0;
        this.lat0 = lat0;
        this.metersPerLon = 111_320.0 * Math.cos(Math.toRadians(lat0 == 0 ? 55.75 : lat0));
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
            return new CrsProjector(false, 0, 0);
        }
        boolean geo = Math.abs(minX) <= 180 && Math.abs(maxX) <= 180
                && Math.abs(minY) <= 90 && Math.abs(maxY) <= 90
                && (maxX - minX) < 10 && (maxY - minY) < 10;
        return new CrsProjector(geo, (minX + maxX) / 2.0, (minY + maxY) / 2.0);
    }

    public boolean isGeographic() {
        return geographic;
    }

    public Geometry toMeters(Geometry geometry) {
        if (geometry == null || !geographic) {
            return geometry == null ? null : geometry.copy();
        }
        Geometry copy = geometry.copy();
        copy.apply((CoordinateFilter) c -> {
            double x = (c.x - lon0) * metersPerLon;
            double y = (c.y - lat0) * metersPerLat;
            c.setX(x);
            c.setY(y);
        });
        copy.geometryChanged();
        return copy;
    }

    public Geometry toLonLat(Geometry geometry) {
        if (geometry == null || !geographic) {
            return geometry == null ? null : geometry.copy();
        }
        Geometry copy = geometry.copy();
        copy.apply((CoordinateFilter) c -> {
            double lon = lon0 + c.x / metersPerLon;
            double lat = lat0 + c.y / metersPerLat;
            c.setX(lon);
            c.setY(lat);
        });
        copy.geometryChanged();
        return copy;
    }

    public Coordinate toMeters(Coordinate c) {
        if (!geographic) {
            return new Coordinate(c);
        }
        return new Coordinate((c.x - lon0) * metersPerLon, (c.y - lat0) * metersPerLat);
    }
}
