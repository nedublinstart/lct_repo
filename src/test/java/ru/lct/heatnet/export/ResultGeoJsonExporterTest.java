package ru.lct.heatnet.export;

import static org.assertj.core.api.Assertions.assertThat;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.geo.CrsProjector;
import ru.lct.heatnet.geo.GeoJsonGeometries;

class ResultGeoJsonExporterTest {

    @Test
    void idsStayUniqueAcrossVariantsAndInputIdsAreKept() throws Exception {
        AppendixModel appendix = new AppendixModel();
        CrsProjector projector = CrsProjector.detect(List.of(
                GeoJsonGeometries.GF.createPoint(new Coordinate(500_000, 6_170_000))));
        String json = new ResultGeoJsonExporter().exportAll(
                List.of(variant(), variant()), appendix, projector);

        JsonNode root = new ObjectMapper().readTree(json);
        Set<String> ids = new HashSet<>();
        int networks = 0;
        int chambers = 0;
        for (JsonNode feature : root.get("features")) {
            JsonNode props = feature.get("properties");
            String id = props.get("id").asText();
            assertThat(ids.add(id)).isTrue();
            if (feature.hasNonNull("id")) {
                assertThat(feature.get("id").asText()).isEqualTo(id);
            }
            String type = props.get("object_type").asText();
            if ("heat_network".equals(type)) {
                networks++;
                assertThat(props.get("end_node_id").isNumber()).isTrue();
                assertThat(props.get("end_node_id").asInt()).isEqualTo(7);
                assertThat(props.get("start_node_id").asText()).endsWith("-CH-1");
                assertThat(props.get("start_node_id").asText()).startsWith(props.get("variant_id").asText());
                assertThat(props.has("length")).isTrue();
                assertThat(props.get("depth_start").isNull()).isTrue();
                assertThat(props.get("depth_end").isNull()).isTrue();
            } else if ("heat_chamber".equals(type)) {
                chambers++;
                assertThat(id).endsWith("-CH-1");
            } else if ("technical_node".equals(type)) {
                assertThat(id).endsWith("-TN-1");
            }
        }
        assertThat(networks).isEqualTo(2);
        assertThat(chambers).isEqualTo(2);
        assertThat(root.get("features")).hasSize(8);
    }

    private static Variant variant() {
        Variant variant = new Variant();
        NewSegment seg = new NewSegment();
        seg.id = "NS-1";
        seg.fromId = "7";
        seg.toId = "CH-1";
        seg.flowTph = 20;
        seg.dn = 100;
        seg.lengthM = 10;
        seg.layingMethod = "base";
        seg.cost = 897_480;
        LineString line = GeoJsonGeometries.GF.createLineString(new Coordinate[]{
                new Coordinate(500_000, 6_170_000),
                new Coordinate(500_010, 6_170_000)
        });
        seg.geometryMeters = line;
        variant.segments.add(seg);

        NewChamber chamber = new NewChamber();
        chamber.id = "CH-1";
        chamber.dn = 100;
        chamber.cost = 3_000_000;
        chamber.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(500_010, 6_170_000));
        variant.chambers.add(chamber);

        TechnicalNode node = new TechnicalNode();
        node.id = "TN-1";
        node.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(500_005, 6_170_000));
        variant.technicalNodes.add(node);
        return variant;
    }
}
