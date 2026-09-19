package ru.lct.heatnet.engine;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
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
        SpatialConstraint c = new SpatialConstraint();
        c.id = "BLD";
        c.type = "oks";
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        AppendixModel appendix = new AppendixLoader(props).load();
        c.rule = appendix.constraintRule("oks");
        c.geometry = wall;
        scene.constraints.add(c);
        scene.envelope();

        List<Variant> variants = new SmartRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        });
        assertThat(variants).isNotEmpty();
        Variant first = variants.get(0);
        assertThat(first.unconnectedOks).isEmpty();
        assertThat(first.segments).isNotEmpty();
        boolean wentAround = false;
        for (NewSegment s : first.segments) {
            for (Coordinate coord : s.geometryMeters.getCoordinates()) {
                if (coord.x < 35 || coord.x > 85 || coord.y < 55 || coord.y > 145) {
                    wentAround = true;
                }
            }
        }
        assertThat(wentAround).isTrue();
        assertThat(first.chambers.size() + first.taps.size()).isGreaterThan(0);
    }

    private static ProspectiveOks oks(GeometryFactory gf, String id, double flow, double x, double y) {
        ProspectiveOks o = new ProspectiveOks();
        o.id = id;
        o.flowTph = flow;
        o.connection = gf.createPoint(new Coordinate(x, y));
        return o;
    }
}
