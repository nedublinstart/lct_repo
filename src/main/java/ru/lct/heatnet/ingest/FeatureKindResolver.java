package ru.lct.heatnet.ingest;

import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import ru.lct.heatnet.appendix.AppendixModel;

public class FeatureKindResolver {

    private final Map<String, FeatureKind> byToken = new HashMap<>();

    public FeatureKindResolver(AppendixModel appendix) {
        appendix.getFeatureKinds().forEach((key, spec) -> {
            FeatureKind kind = fromKey(key);
            if (kind == FeatureKind.UNKNOWN || spec == null || spec.typeValues == null) {
                return;
            }
            for (String token : spec.typeValues) {
                byToken.put(PropertyReader.norm(token), kind);
            }
            byToken.put(PropertyReader.norm(key), kind);
        });
    }

    public FeatureKind resolve(String typeValue, org.locationtech.jts.geom.Geometry geometry,
                               Double flow, String nextId, String oksId) {
        return resolve(typeValue, geometry == null ? "" : geometry.getGeometryType(), flow, nextId, oksId);
    }

    public FeatureKind resolve(String typeValue, String geometryType, Double flow, String nextId, String oksId) {
        FeatureKind mapped = byToken.get(PropertyReader.norm(typeValue));
        if (mapped != null && mapped != FeatureKind.UNKNOWN) {
            return mapped;
        }
        String g = geometryType == null ? "" : geometryType;
        boolean line = g.toLowerCase(Locale.ROOT).contains("line");
        boolean point = g.equalsIgnoreCase("Point") || g.equalsIgnoreCase("MultiPoint");
        boolean poly = g.toLowerCase(Locale.ROOT).contains("polygon");
        if (line && nextId != null) {
            return FeatureKind.EXISTING_SEGMENT;
        }
        if (point && oksId != null) {
            return FeatureKind.CONNECTION_POINT;
        }
        if (poly && flow != null) {
            return FeatureKind.OKS_PROSPECTIVE;
        }
        String t = PropertyReader.norm(typeValue);
        if (t.contains("камер") || t.contains("chamber") || t.equals("tk")) {
            return FeatureKind.CHAMBER;
        }
        if (t.contains("источ") || t.contains("source") || t.contains("тэц")) {
            return FeatureKind.SOURCE;
        }
        if (t.contains("парк") || t.contains("дорог") || t.contains("река") || t.contains("метро")
                || t.contains("road") || t.contains("park") || t.contains("river")) {
            return FeatureKind.CONSTRAINT;
        }
        if (poly) {
            return FeatureKind.CONSTRAINT;
        }
        return FeatureKind.UNKNOWN;
    }

    private static FeatureKind fromKey(String key) {
        switch (PropertyReader.norm(key).replace('-', '_')) {
            case "existing_segment":
            case "heat_network":
                return FeatureKind.EXISTING_SEGMENT;
            case "chamber":
            case "heat_chamber":
                return FeatureKind.CHAMBER;
            case "source":
                return FeatureKind.SOURCE;
            case "oks_prospective":
            case "oks_future":
                return FeatureKind.OKS_PROSPECTIVE;
            case "oks_existing":
                return FeatureKind.OKS_EXISTING;
            case "connection_point":
            case "oks_connection_point":
                return FeatureKind.CONNECTION_POINT;
            case "constraint":
            case "restriction":
                return FeatureKind.CONSTRAINT;
            default:
                return FeatureKind.UNKNOWN;
        }
    }

    public static FeatureKind fromKeyPublic(String key) {
        return new FeatureKindResolver(empty()).resolve(key, "", null, null, null);
    }

    private static AppendixModel empty() {
        AppendixModel model = new AppendixModel();
        Map<String, AppendixModel.KindSpec> kinds = new HashMap<>();
        kinds.put("existing_segment", spec(List.of("existing_segment")));
        model.setFeatureKinds(kinds);
        return model;
    }

    private static AppendixModel.KindSpec spec(List<String> values) {
        AppendixModel.KindSpec spec = new AppendixModel.KindSpec();
        spec.typeValues = values;
        return spec;
    }
}
