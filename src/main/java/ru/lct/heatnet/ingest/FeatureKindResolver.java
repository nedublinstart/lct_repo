package ru.lct.heatnet.ingest;

import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.locationtech.jts.geom.Geometry;
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

    public FeatureKind resolve(String typeValue, Geometry geometry, Double flow, String nextId, String oksId) {
        FeatureKind mapped = byToken.get(PropertyReader.norm(typeValue));
        if (mapped != null && mapped != FeatureKind.UNKNOWN) {
            if (mapped == FeatureKind.OKS_EXISTING) {
                return FeatureKind.CONSTRAINT;
            }
            return mapped;
        }
        String g = geometry == null ? "" : geometry.getGeometryType();
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
                return FeatureKind.EXISTING_SEGMENT;
            case "chamber":
                return FeatureKind.CHAMBER;
            case "source":
                return FeatureKind.SOURCE;
            case "oks_prospective":
                return FeatureKind.OKS_PROSPECTIVE;
            case "oks_existing":
                return FeatureKind.OKS_EXISTING;
            case "connection_point":
                return FeatureKind.CONNECTION_POINT;
            case "constraint":
                return FeatureKind.CONSTRAINT;
            default:
                return FeatureKind.UNKNOWN;
        }
    }

    public static FeatureKind fromKeyPublic(String key) {
        return new FeatureKindResolver(empty()).resolve(key, null, null, null, null);
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
