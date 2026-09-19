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
import ru.lct.heatnet.engine.greedy.GreedyRoutingEngine;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

class GreedyRoutingEngineTest {

    @Test
    void buildsThreeVariantsAndGoesAroundObstacle() {
        GeometryFactory gf = new GeometryFactory();
        Scene scene = new Scene();
        ExistingSegment seg = new ExistingSegment();
        seg.id = "S-1";
        seg.dn = 200;
        seg.existingFlowTph = 10;
        seg.nextId = "SRC";
        seg.line = gf.createLineString(new Coordinate[]{new Coordinate(0, 0), new Coordinate(0, 200)});
        scene.segments.add(seg);

        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "OKS-A";
        oks.flowTph = 8;
        oks.connection = gf.createPoint(new Coordinate(120, 100));
        scene.oks.add(oks);

        Polygon wall = gf.createPolygon(new Coordinate[]{
                new Coordinate(40, 60), new Coordinate(80, 60), new Coordinate(80, 140),
                new Coordinate(40, 140), new Coordinate(40, 60)
        });
        SpatialConstraint c = new SpatialConstraint();
        c.id = "BLD";
        c.type = "existing_building";
        c.geometry = wall;
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        AppendixModel appendix = new AppendixLoader(props).load();
        c.rule = appendix.constraintRule("existing_building");
        scene.constraints.add(c);
        scene.envelope();

        List<Variant> variants = new GreedyRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        });
        assertThat(variants).hasSize(3);
        assertThat(variants.get(0).unconnectedOks).isEmpty();
        assertThat(variants.get(0).segments).isNotEmpty();
        Coordinate[] coords = variants.get(0).segments.get(0).geometryMeters.getCoordinates();
        boolean wentAround = false;
        for (Coordinate coord : coords) {
            if (coord.x < 35 || coord.x > 85 || coord.y < 55 || coord.y > 145) {
                wentAround = true;
            }
        }
        assertThat(wentAround).isTrue();
    }
}
