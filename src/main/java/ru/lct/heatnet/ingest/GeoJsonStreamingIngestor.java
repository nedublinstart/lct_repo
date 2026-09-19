package ru.lct.heatnet.ingest;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonToken;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.function.Consumer;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.persist.IngestedFeature;

@Component
public class GeoJsonStreamingIngestor {

    private static final Logger log = LoggerFactory.getLogger(GeoJsonStreamingIngestor.class);
    private static final int BATCH = 400;
    private final ObjectMapper mapper = GeoJsonGeometries.mapper();
    private final JsonFactory factory = mapper.getFactory();

    public ParseStats parse(Path file, UUID datasetId, AppendixModel appendix, Consumer<List<IngestedFeature>> sink)
            throws IOException {
        FeatureKindResolver resolver = new FeatureKindResolver(appendix);
        ParseStats stats = new ParseStats();
        List<IngestedFeature> batch = new ArrayList<>(BATCH);
        try (InputStream in = Files.newInputStream(file);
             JsonParser parser = factory.createParser(in)) {
            JsonToken token;
            boolean inFeatures = false;
            while ((token = parser.nextToken()) != null) {
                if (!inFeatures && token == JsonToken.FIELD_NAME && "features".equals(parser.currentName())) {
                    parser.nextToken();
                    inFeatures = true;
                    continue;
                }
                if (inFeatures && token == JsonToken.START_OBJECT) {
                    JsonNode feature = mapper.readTree(parser);
                    IngestedFeature ingested = toFeature(feature, datasetId, appendix, resolver, stats);
                    if (ingested != null) {
                        batch.add(ingested);
                        stats.total++;
                        stats.byKind.merge(ingested.getKind(), 1, Integer::sum);
                    }
                    if (batch.size() >= BATCH) {
                        sink.accept(new ArrayList<>(batch));
                        batch.clear();
                    }
                }
                if (inFeatures && token == JsonToken.END_ARRAY) {
                    break;
                }
            }
        }
        if (!batch.isEmpty()) {
            sink.accept(batch);
        }
        log.info("Разобрано объектов: {} ({})", stats.total, stats.byKind);
        return stats;
    }

    private IngestedFeature toFeature(JsonNode feature, UUID datasetId, AppendixModel appendix,
                                      FeatureKindResolver resolver, ParseStats stats) {
        if (feature == null || !feature.isObject()) {
            return null;
        }
        JsonNode geomNode = feature.get("geometry");
        JsonNode propsNode = feature.get("properties");
        PropertyReader reader = new PropertyReader(propsNode);
        Geometry geometry = geomNode == null || geomNode.isNull() ? null : GeoJsonGeometries.read(geomNode.toString());
        String typeValue = reader.firstNonBlank(appendix.aliases("kind"));
        if (typeValue == null && feature.hasNonNull("id") && feature.get("id").isTextual()) {
            typeValue = reader.str("feature_type", "type");
        }
        Double flow = reader.firstNum(appendix.aliases("flow"));
        String nextId = reader.firstNonBlank(appendix.aliases("next_id"));
        String oksId = reader.firstNonBlank(appendix.aliases("oks_id"));
        FeatureKind kind = resolver.resolve(typeValue, geometry, flow, nextId, oksId);
        if (kind == FeatureKind.UNKNOWN) {
            stats.unknown++;
        }
        IngestedFeature entity = new IngestedFeature();
        entity.setDatasetId(datasetId);
        String id = reader.firstNonBlank(appendix.aliases("id"));
        if (id == null && feature.hasNonNull("id")) {
            id = feature.get("id").asText();
        }
        entity.setExternalId(id);
        entity.setKind(kind);
        entity.setPropertiesJson(propsNode == null || propsNode.isNull() ? "{}" : propsNode.toString());
        entity.setGeometryJson(geomNode == null || geomNode.isNull() ? null : geomNode.toString());
        if (geometry != null && !geometry.isEmpty()) {
            Envelope env = geometry.getEnvelopeInternal();
            entity.setMinX(env.getMinX());
            entity.setMinY(env.getMinY());
            entity.setMaxX(env.getMaxX());
            entity.setMaxY(env.getMaxY());
        }
        return entity;
    }

    public static final class ParseStats {
        public int total;
        public int unknown;
        public Map<FeatureKind, Integer> byKind = new EnumMap<>(FeatureKind.class);
    }
}
