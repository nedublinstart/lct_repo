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
import ru.lct.heatnet.engine.ReconstructionChamber;
import ru.lct.heatnet.engine.ReconstructionSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.geo.CrsProjector;
import ru.lct.heatnet.geo.GeoJsonGeometries;

/**
 * Выгрузка сдачи: один FeatureCollection, до трёх вариантов.
 * <p>
 * Поля совпадают с техническим приложением. Геометрия — WGS 84.
 * Врезка ({@code tie_in}) пишется только если она осталась после
 * {@link ru.lct.heatnet.engine.steiner.SubmissionHygiene}: у неё есть труба
 * и точка лежит на существующей сети. Пустая геометрия не выгружается,
 * иначе на карте появляется маркер без линии.
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
                node.put("start_node_id", seg.toId);
                node.put("end_node_id", seg.fromId);
                node.put("flow_tph", round(seg.flowTph));
                node.put("diameter", seg.dn);
                node.put("length", round(seg.lengthM));
                node.put("laying_method", seg.layingMethod == null ? "base" : seg.layingMethod);
                if (seg.depthM == null) {
                    node.putNull("depth_start");
                    node.putNull("depth_end");
                } else {
                    node.put("depth_start", seg.depthM);
                    node.put("depth_end", seg.depthM);
                }
                node.put("cost", round(seg.cost));
            }));
        }
        for (TapPoint tap : variant.taps) {
            if (tap == null || tap.geometryMeters == null) {
                continue;
            }
            features.add(feature("tie_in", tap.id, projector.toLonLat(tap.geometryMeters), node -> {
                node.put("object_type", "tie_in");
                putVariantId(node, variantId);
                node.put("existing_object_id", tap.existingObjectId);
                node.put("existing_object_type", officialKind(tap.existingObjectKind));
                node.put("existing_diameter", tap.existingDiameter);
                node.put("required_diameter", tap.requiredDiameter);
                node.put("cost", round(tap.cost));
            }));
        }
        for (ReconstructionSegment r : variant.reconstructionSegments) {
            features.add(feature("heat_network_reconstruction", r.id, projector.toLonLat(r.geometryMeters), node -> {
                node.put("object_type", "heat_network_reconstruction");
                putVariantId(node, variantId);
                node.put("existing_object_id", r.existingObjectId != null ? r.existingObjectId
                        : r.id.startsWith("RE-") ? r.id.substring(3) : r.id);
                node.put("existing_flow_tph", round(r.existingFlowTph));
                node.put("added_flow_tph", round(r.extraFlowTph));
                node.put("calculated_flow_tph", round(r.calculatedFlowTph));
                node.put("existing_diameter", r.existingDn);
                node.put("required_diameter", r.requiredDn);
                node.put("length", round(r.lengthM));
                node.put("cost", round(r.cost));
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
        for (ReconstructionChamber r : variant.reconstructionChambers) {
            features.add(feature("heat_chamber_reconstruction", r.id, projector.toLonLat(r.geometryMeters), node -> {
                node.put("object_type", "heat_chamber_reconstruction");
                putVariantId(node, variantId);
                node.put("existing_object_id", r.existingObjectId != null ? r.existingObjectId
                        : r.id.startsWith("RC-") ? r.id.substring(3) : r.id);
                node.put("existing_diameter", r.existingDn);
                node.put("required_diameter", r.requiredDn);
                node.put("cost", round(r.cost));
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
        p.put("tie_in_cost", round(num(variant, "tie_in_cost", 0)));
        p.put("reconstruction_cost", round(num(variant, "reconstruction_cost", 0)));
        p.put("chamber_reconstruction_cost", round(num(variant, "chamber_reconstruction_cost", 0)));
        p.put("unconnected_penalty", round(variant.penalty));
        p.put("calculated_cost", round(variant.totalCost));
        p.put("new_network_length", round(variant.newLengthM));
        p.put("reconstruction_length", round(variant.reconLengthM));
        p.put("length", round(variant.newLengthM + variant.reconLengthM));
        p.put("score", Math.round(variant.score * 1000.0) / 1000.0);
        ArrayNode un = p.putArray("unconnected_oks_ids");
        variant.unconnectedOks.forEach(un::add);
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

    /** Раздел 10: variant_id — строка ("1", "2", "3"). */
    private static void putVariantId(ObjectNode node, String variantId) {
        node.put("variant_id", variantId);
    }

    private static String officialKind(String kind) {
        if ("chamber".equals(kind) || "heat_chamber".equals(kind) || "source".equals(kind)) {
            return "heat_chamber";
        }
        return "heat_network";
    }

    private static double num(Variant v, String key, double fallback) {
        Double d = v.costBreakdown.get(key);
        return d == null ? fallback : d;
    }

    private static double round(double v) {
        return Math.round(v * 100.0) / 100.0;
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
