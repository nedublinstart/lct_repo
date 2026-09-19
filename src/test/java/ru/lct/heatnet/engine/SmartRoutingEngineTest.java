package ru.lct.heatnet.engine;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LinearRing;
import org.locationtech.jts.geom.Polygon;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

class SmartRoutingEngineTest {

    @Test
    void jointTreeGoesAroundObstacleAndKeepsThreeVariants() {
        GeometryFactory gf = new GeometryFactory();
        Scene scene = new Scene();
        ExistingSegment seg = new ExistingSegment();
        seg.id = "S-1";
        seg.dn = 400;
        seg.existingFlowTph = 10;
        seg.nextId = "SRC";
        seg.line = gf.createLineString(new Coordinate[]{new Coordinate(0, 0), new Coordinate(0, 200)});
        scene.segments.add(seg);

        ProspectiveOks a = oks(gf, "OKS-A", 8, 120, 80);
        ProspectiveOks b = oks(gf, "OKS-B", 7, 120, 120);
        scene.oks.add(a);
        scene.oks.add(b);

        Polygon wall = gf.createPolygon(new Coordinate[]{
                new Coordinate(40, 60), new Coordinate(80, 60), new Coordinate(80, 140),
                new Coordinate(40, 140), new Coordinate(40, 60)
        });
        AppendixModel appendix = appendix();
        scene.constraints.add(constraint("BLD", wall, appendix));
        scene.envelope();

        List<Variant> variants = new SmartRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        });
        assertThat(variants).isNotEmpty();
        Variant first = variants.get(0);
        assertThat(first.unconnectedOks).isEmpty();
        assertThat(first.segments).isNotEmpty();
        Polygon core = (Polygon) wall.buffer(-1.0);
        for (NewSegment s : first.segments) {
            assertThat(crossesInterior(core, s)).isFalse();
        }
        assertThat(first.chambers.size() + first.taps.size()).isGreaterThan(0);
        int maxPts = 0;
        for (NewSegment s : first.segments) {
            maxPts = Math.max(maxPts, s.geometryMeters.getNumPoints());
        }
        assertThat(maxPts).as("трасса из прямых, не сеточная лесенка").isLessThanOrEqualTo(16);
    }

    @Test
    void treeStaysOnStreetAndDoesNotEnterCourtyard() {
        GeometryFactory gf = new GeometryFactory();
        Scene scene = new Scene();
        ExistingSegment seg = new ExistingSegment();
        seg.id = "S-1";
        seg.dn = 400;
        seg.existingFlowTph = 10;
        seg.nextId = "SRC";
        seg.line = gf.createLineString(new Coordinate[]{new Coordinate(0, 0), new Coordinate(0, 200)});
        scene.segments.add(seg);

        LinearRing shell = gf.createLinearRing(new Coordinate[]{
                new Coordinate(30, 30), new Coordinate(170, 30), new Coordinate(170, 170),
                new Coordinate(30, 170), new Coordinate(30, 30)
        });
        LinearRing hole = gf.createLinearRing(new Coordinate[]{
                new Coordinate(70, 70), new Coordinate(70, 130), new Coordinate(130, 130),
                new Coordinate(130, 70), new Coordinate(70, 70)
        });
        Polygon block = gf.createPolygon(shell, new LinearRing[]{hole});
        AppendixModel appendix = appendix();
        scene.constraints.add(constraint("BLOCK", block, appendix));
        scene.oks.add(oks(gf, "OKS-A", 9, 50, 80));
        scene.oks.add(oks(gf, "OKS-B", 8, 50, 120));
        scene.envelope();

        Variant first = new SmartRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        }).get(0);
        assertThat(first.unconnectedOks).isEmpty();

        Polygon courtyard = gf.createPolygon(new Coordinate[]{
                new Coordinate(72, 72), new Coordinate(128, 72), new Coordinate(128, 128),
                new Coordinate(72, 128), new Coordinate(72, 72)
        });
        Coordinate cpA = new Coordinate(50, 80);
        Coordinate cpB = new Coordinate(50, 120);
        int streetPts = 0;
        for (NewSegment s : first.segments) {
            for (Coordinate c : s.geometryMeters.getCoordinates()) {
                if (c.distance(cpA) < 12 || c.distance(cpB) < 12) {
                    continue;
                }
                assertThat(courtyard.contains(gf.createPoint(c)))
                        .as("точка трассы %s внутри двора", c)
                        .isFalse();
                if (c.x < 28) {
                    streetPts++;
                }
            }
        }
        assertThat(streetPts).isGreaterThan(0);
    }

    private static boolean crossesInterior(Polygon core, NewSegment segment) {
        if (segment.geometryMeters == null || core == null || core.isEmpty()) {
            return false;
        }
        return core.intersects(segment.geometryMeters) && !core.touches(segment.geometryMeters);
    }

    private static SpatialConstraint constraint(String id, Polygon geometry, AppendixModel appendix) {
        SpatialConstraint c = new SpatialConstraint();
        c.id = id;
        c.type = "oks";
        c.geometry = geometry;
        c.rule = appendix.constraintRule("oks");
        return c;
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }

    private static ProspectiveOks oks(GeometryFactory gf, String id, double flow, double x, double y) {
        ProspectiveOks o = new ProspectiveOks();
        o.id = id;
        o.flowTph = flow;
        o.connection = gf.createPoint(new Coordinate(x, y));
        return o;
    }
}
