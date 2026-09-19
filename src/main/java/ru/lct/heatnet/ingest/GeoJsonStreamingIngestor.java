package ru.lct.heatnet.ingest;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonToken;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.BufferedInputStream;
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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.persist.IngestedFeature;

/**
 * Читает FeatureCollection токенами Jackson: в памяти только текущий Feature, без дерева всего файла.
 */
@Component
public class GeoJsonStreamingIngestor {

    private static final Logger log = LoggerFactory.getLogger(GeoJsonStreamingIngestor.class);
    static final int BATCH = 250;
    private static final int BUFFER_BYTES = 1 << 20;
    private final ObjectMapper mapper = GeoJsonGeometries.mapper();
    private final JsonFactory factory = mapper.getFactory();

    public ParseStats parse(Path file, UUID datasetId, AppendixModel appendix, Consumer<List<IngestedFeature>> sink)
            throws IOException {
        try (InputStream in = new BufferedInputStream(Files.newInputStream(file), BUFFER_BYTES)) {
            return parse(in, datasetId, appendix, sink);
        }
    }

    public ParseStats parse(InputStream in, UUID datasetId, AppendixModel appendix, Consumer<List<IngestedFeature>> sink)
            throws IOException {
        FeatureKindResolver resolver = new FeatureKindResolver(appendix);
        ParseStats stats = new ParseStats();
        List<IngestedFeature> batch = new ArrayList<>(BATCH);
        try (JsonParser parser = factory.createParser(in instanceof BufferedInputStream
                ? in
                : new BufferedInputStream(in, BUFFER_BYTES))) {
            boolean inFeatures = false;
            JsonToken token;
            while ((token = parser.nextToken()) != null) {
                if (!inFeatures && token == JsonToken.FIELD_NAME && "features".equals(parser.currentName())) {
                    parser.nextToken();
                    inFeatures = true;
                    continue;
                }
                if (inFeatures && token == JsonToken.START_OBJECT) {
                    IngestedFeature ingested = readFeature(parser, datasetId, appendix, resolver, stats);
                    if (ingested != null) {
                        batch.add(ingested);
                        stats.total++;
                        stats.byKind.merge(ingested.getKind(), 1, Integer::sum);
                        expand(stats, ingested);
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

    private IngestedFeature readFeature(JsonParser parser, UUID datasetId, AppendixModel appendix,
                                        FeatureKindResolver resolver, ParseStats stats) throws IOException {
        String featureId = null;
        JsonNode propsNode = null;
        String geometryJson = null;
        String geometryType = "";
        Envelope envelope = null;
        while (parser.nextToken() != JsonToken.END_OBJECT) {
            if (parser.currentToken() != JsonToken.FIELD_NAME) {
                continue;
            }
            String field = parser.currentName();
            JsonToken value = parser.nextToken();
            if ("id".equals(field)) {
                featureId = parser.getValueAsString();
            } else if ("properties".equals(field)) {
                if (value == JsonToken.VALUE_NULL) {
                    propsNode = null;
                } else {
                    propsNode = mapper.readTree(parser);
                }
            } else if ("geometry".equals(field)) {
                GeometryTokenCopier.Copied copied = GeometryTokenCopier.copy(parser, factory);
                geometryJson = copied.json;
                geometryType = copied.type == null ? "" : copied.type;
                if (copied.envelope != null && !copied.envelope.isNull()) {
                    envelope = copied.envelope;
                }
            } else {
                parser.skipChildren();
            }
        }
        PropertyReader reader = new PropertyReader(propsNode);
        String typeValue = reader.firstNonBlank(appendix.aliases("kind"));
        Double flow = reader.firstNum(appendix.aliases("flow"));
        String nextId = reader.firstNonBlank(appendix.aliases("next_id"));
        String oksId = reader.firstNonBlank(appendix.aliases("oks_id"));
        FeatureKind kind = resolver.resolve(typeValue, geometryType, flow, nextId, oksId);
        if (kind == FeatureKind.UNKNOWN) {
            stats.unknown++;
        }
        IngestedFeature entity = new IngestedFeature();
        entity.setId(UUID.randomUUID());
        entity.setDatasetId(datasetId);
        String id = reader.firstNonBlank(appendix.aliases("id"));
        if (id == null) {
            id = featureId;
        }
        entity.setExternalId(id);
        entity.setKind(kind);
        entity.setPropertiesJson(propsNode == null || propsNode.isNull() ? "{}" : propsNode.toString());
        entity.setGeometryJson("null".equals(geometryJson) ? null : geometryJson);
        if (envelope != null && !envelope.isNull()) {
            entity.setMinX(envelope.getMinX());
            entity.setMinY(envelope.getMinY());
            entity.setMaxX(envelope.getMaxX());
            entity.setMaxY(envelope.getMaxY());
        }
        return entity;
    }

    private static void expand(ParseStats stats, IngestedFeature feature) {
        if (feature.getGeometryJson() == null) {
            return;
        }
        stats.envelope.expandToInclude(feature.getMinX(), feature.getMinY());
        stats.envelope.expandToInclude(feature.getMaxX(), feature.getMaxY());
    }

    public static final class ParseStats {
        public int total;
        public int unknown;
        public Map<FeatureKind, Integer> byKind = new EnumMap<>(FeatureKind.class);
        public final Envelope envelope = new Envelope();
    }
}
