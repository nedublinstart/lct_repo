package ru.lct.heatnet.export;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
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

@Component
public class ResultGeoJsonExporter {

    private final ObjectMapper mapper = GeoJsonGeometries.mapper();

    public String export(Variant variant, AppendixModel appendix, CrsProjector projector) {
        ObjectNode root = mapper.createObjectNode();
        root.put("type", "FeatureCollection");
        root.put("name", appendix.getExport().collectionName);
        ObjectNode props = root.putObject("properties");
        props.put("variant", variant.code);
        props.put("title", variant.title);
        props.put("description", variant.description);
        props.put("total_cost", variant.totalCost);
        props.put("score", variant.score);
        props.put("new_length_m", variant.newLengthM);
        props.put("recon_length_m", variant.reconLengthM);
        ArrayNode unconnected = props.putArray("unconnected_oks");
        variant.unconnectedOks.forEach(unconnected::add);
        ArrayNode features = root.putArray("features");

        for (NewSegment seg : variant.segments) {
            features.add(feature(type(appendix, "new_segment", "new_segment"), seg.id, projector.toLonLat(seg.geometryMeters), node -> {
                node.put("flow_tph", seg.flowTph);
                node.put("dn", seg.dn);
                node.put("length_m", round(seg.lengthM));
                node.put("laying_method", seg.layingMethod);
                node.put("cost", round(seg.cost));
                node.put("from_id", seg.fromId);
                node.put("to_id", seg.toId);
                if (seg.depthM != null) {
                    node.put("depth_m", seg.depthM);
                }
                if (seg.specialReason != null) {
                    node.put("special_reason", seg.specialReason);
                }
            }));
        }
        for (NewChamber ch : variant.chambers) {
            features.add(feature(type(appendix, "new_chamber", "new_chamber"), ch.id, projector.toLonLat(ch.geometryMeters), node -> {
                node.put("dn", ch.dn);
                node.put("cost", round(ch.cost));
                node.put("at_tap", ch.atTap);
            }));
        }
        for (TapPoint tap : variant.taps) {
            features.add(feature(type(appendix, "tap", "tap_point"), tap.id, projector.toLonLat(tap.geometryMeters), node -> {
                node.put("existing_object_id", tap.existingObjectId);
                node.put("existing_object_kind", tap.existingObjectKind);
                node.put("extra_flow_tph", tap.extraFlowTph);
                node.put("cost", round(tap.cost));
            }));
        }
        for (TechnicalNode n : variant.technicalNodes) {
            features.add(feature(type(appendix, "technical_node", "technical_node"), n.id, projector.toLonLat(n.geometryMeters), node -> {
                node.put("reason", n.reason);
            }));
        }
        for (ReconstructionSegment r : variant.reconstructionSegments) {
            features.add(feature(type(appendix, "reconstruction_segment", "reconstruction_segment"), r.id, projector.toLonLat(r.geometryMeters), node -> {
                node.put("existing_dn", r.existingDn);
                node.put("required_dn", r.requiredDn);
                node.put("extra_flow_tph", r.extraFlowTph);
                node.put("length_m", round(r.lengthM));
                node.put("cost", round(r.cost));
            }));
        }
        for (ReconstructionChamber r : variant.reconstructionChambers) {
            features.add(feature(type(appendix, "reconstruction_chamber", "reconstruction_chamber"), r.id, projector.toLonLat(r.geometryMeters), node -> {
                node.put("existing_dn", r.existingDn);
                node.put("required_dn", r.requiredDn);
                node.put("cost", round(r.cost));
            }));
        }
        for (String oksId : variant.unconnectedOks) {
            ObjectNode f = mapper.createObjectNode();
            f.put("type", "Feature");
            f.putNull("geometry");
            ObjectNode p = f.putObject("properties");
            p.put("id", oksId);
            p.put("feature_type", type(appendix, "unconnected_oks", "unconnected_oks"));
            features.add(f);
        }
        try {
            return mapper.writerWithDefaultPrettyPrinter().writeValueAsString(root);
        } catch (Exception e) {
            throw new IllegalStateException(e);
        }
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
        p.put("feature_type", type);
        sink.put(p);
        return f;
    }

    private static String type(AppendixModel appendix, String key, String fallback) {
        if (appendix.getExport() != null && appendix.getExport().types != null) {
            return appendix.getExport().types.getOrDefault(key, fallback);
        }
        return fallback;
    }

    private static double round(double v) {
        return Math.round(v * 100.0) / 100.0;
    }

    @FunctionalInterface
    private interface PropertySink {
        void put(ObjectNode node);
    }
}
