package ru.lct.heatnet.export;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.List;
import org.locationtech.jts.geom.Geometry;
import org.springframework.stereotype.Component;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.geo.CrsProjector;
import ru.lct.heatnet.geo.GeoJsonGeometries;

/**
 * Выгрузка сдачи: один FeatureCollection, до трёх вариантов.
 * <p>
 * Состав совпадает с разделом 7 актуального приложения: участки, новые камеры, технические узлы
 * и сводка. Отдельной врезки и реконструкции в файле нет. Геометрия — WGS 84.
 * Пустая геометрия не выгружается, иначе на карте появляется маркер без линии.
 */
@Component
public class ResultGeoJsonExporter {

    private final ObjectMapper mapper = GeoJsonGeometries.mapper();

    public String export(Variant variant, AppendixModel appendix, CrsProjector projector) {
        return export(variant, "1", appendix, projector);
    }

    public String export(Variant variant, String variantId, AppendixModel appendix, CrsProjector projector) {
        ObjectNode root = mapper.createObjectNode();
        root.put("type", "FeatureCollection");
        root.put("name", appendix.getExport().collectionName);
        ArrayNode features = root.putArray("features");
        append(features, variant, variantId, appendix, projector);
        return write(root);
    }

    public String exportAll(List<Variant> variants, AppendixModel appendix, CrsProjector projector) {
        ObjectNode root = mapper.createObjectNode();
        root.put("type", "FeatureCollection");
        root.put("name", appendix.getExport().collectionName);
        ArrayNode features = root.putArray("features");
        int rank = 1;
        for (Variant variant : variants) {
            if (rank > 3) {
                break;
            }
            append(features, variant, String.valueOf(rank), appendix, projector);
            rank++;
        }
        return write(root);
    }

    private void append(ArrayNode features, Variant variant, String variantId, AppendixModel appendix, CrsProjector projector) {
        for (NewSegment seg : variant.segments) {
            if (seg == null || seg.geometryMeters == null || seg.geometryMeters.getNumPoints() < 2) {
                continue;
            }
            Geometry fromSource = seg.geometryMeters.reverse();
            features.add(feature("heat_network", seg.id, projector.toLonLat(fromSource), node -> {
                node.put("object_type", "heat_network");
                putVariantId(node, variantId);
                putId(node, "start_node_id", seg.toId);
                putId(node, "end_node_id", seg.fromId);
                node.put("flow_tph", round(seg.flowTph));
                node.put("diameter", seg.dn);
                node.put("length", round(seg.lengthM));
                node.put("laying_method", seg.layingMethod == null ? "base" : seg.layingMethod);
                // Геометрия в файле развёрнута: начало выгрузки — toId, конец — fromId.
                Double depthStart = seg.depthTo != null ? seg.depthTo : seg.depthM;
                Double depthEnd = seg.depthFrom != null ? seg.depthFrom : seg.depthM;
                if (depthStart == null && depthEnd == null) {
                    node.putNull("depth_start");
                    node.putNull("depth_end");
                } else {
                    double start = depthStart != null ? depthStart : depthEnd;
                    double end = depthEnd != null ? depthEnd : depthStart;
                    node.put("depth_start", roundDepth(start));
                    node.put("depth_end", roundDepth(end));
                }
                node.put("cost", round(seg.cost));
            }));
        }
        for (NewChamber ch : variant.chambers) {
            features.add(feature("heat_chamber", ch.id, projector.toLonLat(ch.geometryMeters), node -> {
                node.put("object_type", "heat_chamber");
                putVariantId(node, variantId);
                node.put("diameter", ch.dn);
                node.put("cost", round(ch.cost));
            }));
        }
        for (TechnicalNode n : variant.technicalNodes) {
            features.add(feature("technical_node", n.id, projector.toLonLat(n.geometryMeters), node -> {
                node.put("object_type", "technical_node");
                putVariantId(node, variantId);
            }));
        }
        ObjectNode summary = mapper.createObjectNode();
        summary.put("type", "Feature");
        summary.putNull("geometry");
        ObjectNode p = summary.putObject("properties");
        p.put("id", "summary_" + variantId);
        p.put("object_type", "variant_summary");
        putVariantId(p, variantId);
        p.put("rank", Integer.parseInt(variantId.replaceAll("[^0-9]", "1")));
        p.put("construction_cost", round(num(variant, "construction_cost", variant.constructionCost)));
        p.put("chamber_construction_cost", round(num(variant, "chamber_construction_cost", 0)));
        p.put("existing_chamber_tie_in_count", (int) Math.round(num(variant, "existing_chamber_tie_in_count",
                variant.taps == null ? 0 : variant.taps.size())));
        p.put("existing_chamber_tie_in_cost", round(num(variant, "existing_chamber_tie_in_cost",
                num(variant, "tie_in_cost", 0))));
        p.put("unconnected_penalty", round(variant.penalty));
        p.put("calculated_cost", round(variant.totalCost));
        p.put("new_network_length", round(variant.newLengthM));
        p.put("score", Math.round(variant.score * 1000.0) / 1000.0);
        ArrayNode un = p.putArray("unconnected_oks_ids");
        for (String id : variant.unconnectedOks) {
            putId(un, id);
        }
        features.add(summary);
    }

    private ObjectNode feature(String type, String id, Geometry geom, PropertySink sink) {
        ObjectNode f = mapper.createObjectNode();
        f.put("type", "Feature");
        if (id != null) {
            f.put("id", id);
        }
        try {
            if (geom == null) {
                f.putNull("geometry");
            } else {
                f.set("geometry", mapper.readTree(GeoJsonGeometries.write(geom)));
            }
        } catch (Exception e) {
            f.putNull("geometry");
        }
        ObjectNode p = f.putObject("properties");
        p.put("id", id);
        sink.put(p);
        return f;
    }

    /** Идентификатор варианта. Строка допустима приложением наравне с числом. */
    private static void putVariantId(ObjectNode node, String variantId) {
        node.put("variant_id", variantId);
    }

    /** Числовой id входных данных пишется числом, остальные — строкой. */
    private static void putId(ObjectNode node, String key, String id) {
        if (id != null && id.matches("-?\\d{1,18}")) {
            node.put(key, Long.parseLong(id));
        } else {
            node.put(key, id);
        }
    }

    private static void putId(ArrayNode array, String id) {
        if (id != null && id.matches("-?\\d{1,18}")) {
            array.add(Long.parseLong(id));
        } else {
            array.add(id);
        }
    }

    private static double num(Variant v, String key, double fallback) {
        Double d = v.costBreakdown.get(key);
        return d == null ? fallback : d;
    }

    private static double round(double v) {
        return Math.round(v * 100.0) / 100.0;
    }

    /** 0,1 мм: уклон 0,10 не должен поплыть из-за округления отметки. */
    private static double roundDepth(double v) {
        return Math.round(v * 10000.0) / 10000.0;
    }

    private String write(ObjectNode root) {
        try {
            return mapper.writerWithDefaultPrettyPrinter().writeValueAsString(root);
        } catch (Exception e) {
            throw new IllegalStateException(e);
        }
    }

    @FunctionalInterface
    private interface PropertySink {
        void put(ObjectNode node);
    }
}
