package ru.lct.heatnet.engine.depth;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.fasterxml.jackson.databind.JsonNode;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.costing.CostCalculator;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.export.ResultGeoJsonExporter;
import ru.lct.heatnet.geo.CrsProjector;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

class DepthPostProcessorTest {

    private final AppendixModel appendix = appendix();
    private final DepthPostProcessor depth = new DepthPostProcessor();

    @Test
    void gasCrossingStaysAboveAndKeepsSlope() {
        Variant variant = pipe(500_000, 6_170_000, 500_100, 6_170_000, 100, "OKS-1", "CH-1");
        Scene scene = new Scene();
        scene.constraints.add(constraint("g", "gas_pipeline", line(500_050, 6_169_980, 500_050, 6_170_020)));
        depth.apply(variant, scene, appendix);

        assertThat(depthAt(variant, 500_000)).isCloseTo(3.0, within(1e-3));
        assertThat(depthAt(variant, 500_100)).isCloseTo(3.0, within(1e-3));
        assertThat(depthAt(variant, 500_050)).isCloseTo(2.42, within(0.03));
        assertProfile(variant);
        assertThat(variant.technicalNodes).isNotEmpty();
        double pipes = cost(variant);
        assertThat(pipes).isCloseTo(100 * 89_748.0, within(50.0));
    }

    @Test
    void tieInDoesNotDodgeTheNetworkItJoins() {
        Variant variant = pipe(50, 40, 50, 0, 100, "OKS-1", "CH-1");
        NewChamber chamber = new NewChamber();
        chamber.id = "CH-1";
        chamber.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(50, 0));
        variant.chambers.add(chamber);
        Scene scene = new Scene();
        ExistingSegment network = new ExistingSegment();
        network.id = "net";
        network.dn = 200;
        network.line = line(0, 0, 100, 0);
        scene.segments.add(network);
        depth.apply(variant, scene, appendix);

        assertThat(variant.segments).hasSize(1);
        assertThat(variant.segments.get(0).depthFrom).isCloseTo(3.0, within(1e-6));
        assertThat(variant.segments.get(0).depthTo).isCloseTo(3.0, within(1e-6));
        assertThat(variant.technicalNodes).isEmpty();
    }

    @Test
    void crossingExistingNetworkGoesAboveWithoutExtraCost() {
        Variant variant = pipe(0, 20, 80, 20, 100, "OKS-1", "CH-1");
        NewChamber chamber = new NewChamber();
        chamber.id = "CH-1";
        chamber.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(80, 20));
        variant.chambers.add(chamber);
        Scene scene = new Scene();
        ExistingSegment network = new ExistingSegment();
        network.id = "net";
        network.dn = 200;
        network.line = line(40, 0, 40, 40);
        scene.segments.add(network);
        depth.apply(variant, scene, appendix);

        assertThat(depthAt(variant, 40)).isCloseTo(2.32, within(0.03));
        assertThat(depthAt(variant, 0)).isCloseTo(3.0, within(1e-3));
        assertProfile(variant);
        assertThat(cost(variant)).isCloseTo(80 * 89_748.0, within(50.0));
    }

    @Test
    void tallPipeCrossesCableFromBelow() {
        Variant variant = pipe(0, 0, 80, 0, 1400, "OKS-1", "CH-1");
        Scene scene = new Scene();
        scene.constraints.add(constraint("c", "power_cable", line(40, -10, 40, 10)));
        depth.apply(variant, scene, appendix);

        assertThat(depthAt(variant, 40)).isCloseTo(3.4, within(0.05));
        assertThat(depthAt(variant, 0)).isCloseTo(3.0, within(1e-2));
        assertProfile(variant);
        double flat = 80 * 683_417;
        assertThat(cost(variant)).isGreaterThan(flat * 1.001);
        boolean splitAtThree = false;
        for (NewSegment seg : variant.segments) {
            double lo = Math.min(seg.depthFrom, seg.depthTo);
            double hi = Math.max(seg.depthFrom, seg.depthTo);
            if (Math.abs(lo - 3.0) < 0.02 && hi > 3.2) {
                splitAtThree = true;
            }
            assertThat(lo < 2.999 && hi > 3.001).isFalse();
        }
        assertThat(splitAtThree).isTrue();
    }

    @Test
    void roadCoverDoesNotDeepenOrdinaryLaying() {
        Variant variant = pipe(0, 20, 100, 20, 100, "OKS-1", "CH-1");
        Scene scene = new Scene();
        Polygon road = GeoJsonGeometries.GF.createPolygon(new Coordinate[]{
                new Coordinate(40, 10), new Coordinate(60, 10), new Coordinate(60, 30),
                new Coordinate(40, 30), new Coordinate(40, 10)
        });
        scene.constraints.add(constraint("r", "road", road));
        depth.apply(variant, scene, appendix);

        assertThat(variant.segments).hasSize(1);
        assertThat(variant.segments.get(0).depthFrom).isCloseTo(3.0, within(1e-6));
        assertThat(variant.segments.get(0).depthTo).isCloseTo(3.0, within(1e-6));
        assertThat(variant.technicalNodes).isEmpty();
    }

    @Test
    void exportWritesEndsAndPlanModeStaysNull() throws Exception {
        Variant variant = pipe(500_000, 6_170_000, 500_100, 6_170_000, 100, "OKS-1", "CH-1");
        Scene scene = new Scene();
        scene.constraints.add(constraint("g", "gas_pipeline", line(500_050, 6_169_980, 500_050, 6_170_020)));
        depth.apply(variant, scene, appendix);
        new CostCalculator().apply(variant, appendix);
        CrsProjector projector = CrsProjector.detect(List.of(variant.segments.get(0).geometryMeters));
        String json = new ResultGeoJsonExporter().export(variant, "1", appendix, projector);
        JsonNode features = GeoJsonGeometries.mapper().readTree(json).get("features");
        boolean ramp = false;
        for (JsonNode feature : features) {
            JsonNode props = feature.get("properties");
            if (!"heat_network".equals(props.get("object_type").asText())) {
                continue;
            }
            assertThat(props.get("depth_start").isNull()).isFalse();
            assertThat(props.get("depth_end").isNull()).isFalse();
            NewSegment seg = find(variant, props.get("id").asText());
            assertThat(props.get("depth_start").asDouble()).isCloseTo(seg.depthTo, within(1e-4));
            assertThat(props.get("depth_end").asDouble()).isCloseTo(seg.depthFrom, within(1e-4));
            if (Math.abs(seg.depthFrom - seg.depthTo) > 0.05) {
                ramp = true;
            }
        }
        assertThat(ramp).isTrue();

        Variant flat = pipe(500_000, 6_170_000, 500_100, 6_170_000, 100, "OKS-1", "CH-1");
        new CostCalculator().apply(flat, appendix);
        String plan = new ResultGeoJsonExporter().export(flat, "1", appendix, projector);
        JsonNode planNet = GeoJsonGeometries.mapper().readTree(plan).get("features").get(0).get("properties");
        assertThat(planNet.get("depth_start").isNull()).isTrue();
        assertThat(planNet.get("depth_end").isNull()).isTrue();
    }

    @Test
    void kindNamesFollowTheAppendix() {
        assertThat(DepthPostProcessor.kind("railway")).isEqualTo("railway");
        assertThat(DepthPostProcessor.kind("железнодорожный")).isEqualTo("railway");
        assertThat(DepthPostProcessor.kind("road")).isEqualTo("road");
        assertThat(DepthPostProcessor.kind("tdtp")).isEqualTo("road");
        assertThat(DepthPostProcessor.kind("tram_tracks")).isEqualTo("tram");
        assertThat(DepthPostProcessor.kind("gas_pipeline")).isEqualTo("gas");
        assertThat(DepthPostProcessor.kind("power_cable")).isEqualTo("cable");
        assertThat(DepthPostProcessor.kind("heat_network")).isEqualTo("heat");
        assertThat(DepthPostProcessor.kind("oks")).isEmpty();
    }

    private static void assertProfile(Variant variant) {
        double length = 0;
        for (NewSegment seg : variant.segments) {
            assertThat(seg.depthFrom).isNotNull();
            assertThat(seg.depthTo).isNotNull();
            assertThat(seg.depthFrom).isGreaterThanOrEqualTo(0.7 - 1e-6);
            assertThat(seg.depthTo).isGreaterThanOrEqualTo(0.7 - 1e-6);
            if (seg.lengthM > 1e-3) {
                double grade = Math.abs(seg.depthTo - seg.depthFrom) / seg.lengthM;
                assertThat(grade).isLessThanOrEqualTo(0.1005);
            }
            double lo = Math.min(seg.depthFrom, seg.depthTo);
            double hi = Math.max(seg.depthFrom, seg.depthTo);
            assertThat(lo < 2.999 && hi > 3.001).isFalse();
            length += seg.lengthM;
            assertThat(seg.depthM).isNull();
        }
        assertThat(length).isGreaterThan(1);
        for (TechnicalNode node : variant.technicalNodes) {
            boolean used = false;
            for (NewSegment seg : variant.segments) {
                if (node.id.equals(seg.fromId) || node.id.equals(seg.toId)) {
                    used = true;
                }
            }
            assertThat(used).isTrue();
        }
    }

    private double cost(Variant variant) {
        new CostCalculator().apply(variant, appendix);
        return variant.costBreakdown.get("pipe_cost");
    }

    private static double depthAt(Variant variant, double x) {
        for (NewSegment seg : variant.segments) {
            Coordinate a = seg.geometryMeters.getCoordinateN(0);
            Coordinate b = seg.geometryMeters.getCoordinateN(seg.geometryMeters.getNumPoints() - 1);
            double min = Math.min(a.x, b.x) - 1e-6;
            double max = Math.max(a.x, b.x) + 1e-6;
            if (x < min || x > max) {
                continue;
            }
            double span = b.x - a.x;
            double t = Math.abs(span) < 1e-9 ? 0 : (x - a.x) / span;
            return seg.depthFrom + t * (seg.depthTo - seg.depthFrom);
        }
        throw new AssertionError("нет участка на x=" + x);
    }

    private static NewSegment find(Variant variant, String id) {
        for (NewSegment seg : variant.segments) {
            if (id.equals(seg.id) || id.equals("1-" + seg.id)) {
                return seg;
            }
        }
        throw new AssertionError(id);
    }

    private static Variant pipe(double x1, double y1, double x2, double y2, int dn, String from, String to) {
        NewSegment seg = new NewSegment();
        seg.id = "NS-1";
        seg.geometryMeters = line(x1, y1, x2, y2);
        seg.lengthM = seg.geometryMeters.getLength();
        seg.dn = dn;
        seg.flowTph = 10;
        seg.fromId = from;
        seg.toId = to;
        seg.layingMethod = "base";
        seg.kSpec = 1;
        Variant variant = new Variant();
        variant.segments.add(seg);
        return variant;
    }

    private static SpatialConstraint constraint(String id, String type, org.locationtech.jts.geom.Geometry geometry) {
        SpatialConstraint constraint = new SpatialConstraint();
        constraint.id = id;
        constraint.type = type;
        constraint.geometry = geometry;
        return constraint;
    }

    private static LineString line(double x1, double y1, double x2, double y2) {
        return GeoJsonGeometries.GF.createLineString(new Coordinate[]{
                new Coordinate(x1, y1), new Coordinate(x2, y2)
        });
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
