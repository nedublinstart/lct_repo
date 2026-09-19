package ru.lct.heatnet.geo;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.PrecisionModel;
import org.wololo.jts2geojson.GeoJSONReader;
import org.wololo.jts2geojson.GeoJSONWriter;

public final class GeoJsonGeometries {

    public static final GeometryFactory GF = new GeometryFactory(new PrecisionModel(), 0);
    private static final GeoJSONReader READER = new GeoJSONReader();
    private static final GeoJSONWriter WRITER = new GeoJSONWriter();
    private static final ObjectMapper MAPPER = new ObjectMapper();

    private GeoJsonGeometries() {
    }

    public static Geometry read(String geometryJson) {
        if (geometryJson == null || geometryJson.isBlank() || "null".equals(geometryJson)) {
            return null;
        }
        try {
            return READER.read(geometryJson);
        } catch (RuntimeException e) {
            throw new IllegalArgumentException("Некорректная GeoJSON-геометрия: " + e.getMessage(), e);
        }
    }

    public static String write(Geometry geometry) {
        if (geometry == null || geometry.isEmpty()) {
            return null;
        }
        return WRITER.write(geometry).toString();
    }

    public static Envelope envelope(JsonNode geometryNode) {
        Geometry g = read(geometryNode == null ? null : geometryNode.toString());
        return g == null ? null : g.getEnvelopeInternal();
    }

    public static ObjectMapper mapper() {
        return MAPPER;
    }
}
