package ru.lct.heatnet.engine;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.SpecialLayer;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

class RoadTdtpRoutingTest {

    private final GeometryFactory gf = new GeometryFactory();

    @Test
    void crossesRoadNearPerpendicularAndDoesNotRunAlongCarriageway() {
        AppendixModel appendix = appendix();
        Scene scene = streetScene(true, appendix);
        Variant first = new SmartRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        }).get(0);
        assertThat(first.unconnectedOks).isEmpty();
        assertThat(first.segments).isNotEmpty();

        Polygon road = roadPoly();
        Coordinate axis = new Coordinate(0, 1);
        double alongBad = 0;
        double specialLen = 0;
        boolean sawSpecial = false;
        for (NewSegment seg : first.segments) {
            LineString ls = seg.geometryMeters;
            double hit = SpecialLayer.hitLength(ls, road);
            if ("special".equals(seg.layingMethod)) {
                sawSpecial = true;
                specialLen += seg.lengthM;
                assertThat(seg.kSpec).isGreaterThan(1.01);
                assertThat(seg.lengthM).isLessThan(75);
            }
            if (hit < 4) {
                continue;
            }
            Coordinate[] c = ls.getCoordinates();
            double ang = SpecialLayer.crossingAngleDeg(c[0], c[c.length - 1], axis);
            if (ang < 45) {
                alongBad += hit;
            } else {
                assertThat(hit).isLessThan(road.getEnvelopeInternal().getWidth() * 1.8);
            }
        }
        assertThat(sawSpecial).as("спецпроход только на пересечении проезжей").isTrue();
        assertThat(specialLen).isLessThan(90);
        assertThat(alongBad).as("продольный ход по проезжей").isLessThan(8);
        assertThat(first.technicalNodes).isNotEmpty();
    }

    @Test
    void inferredStreetWithoutRoadPolygonStillBlocksLongDiagonal() {
        AppendixModel appendix = appendix();
        Scene scene = streetScene(false, appendix);
        Variant first = new SmartRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        }).get(0);
        assertThat(first.unconnectedOks).isEmpty();

        Polygon gap = gf.createPolygon(new Coordinate[]{
                new Coordinate(28, 20), new Coordinate(76, 20),
                new Coordinate(76, 100), new Coordinate(28, 100),
                new Coordinate(28, 20)
        });
        Coordinate axis = new Coordinate(0, 1);
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);
        for (NewSegment seg : first.segments) {
            Coordinate[] c = seg.geometryMeters.getCoordinates();
            for (int i = 1; i < c.length; i++) {
                org.locationtech.jts.geom.LineString edge = gf.createLineString(new Coordinate[]{c[i - 1], c[i]});
                double hit = SpecialLayer.hitLength(edge, gap);
                if (hit < 12) {
                    continue;
                }
                double ang = SpecialLayer.crossingAngleDeg(c[i - 1], c[i], axis);
                if (ang < 40) {
                    assertThat(obstacles.alongAvoid(c[i - 1], c[i], 8))
                            .as("продольный ход по проезжей %s", seg.id)
                            .isTrue();
                } else {
                    assertThat(ang)
                            .as("диагональ через улицу %s", seg.id)
                            .isGreaterThanOrEqualTo(65.0);
                }
                assertThat(hit).isLessThan(70);
            }
        }
    }

    @Test
    void tdtpAliasUsesSpecialRoadRule() {
        AppendixModel appendix = appendix();
        AppendixModel.ConstraintSpec spec = appendix.constraintRule("ТДТП");
        assertThat(spec.special()).isTrue();
        assertThat(spec.minAngleDeg).isEqualTo(45);
        assertThat(appendix.constraintRule("проезжая часть").special()).isTrue();
        assertThat(appendix.constraintRule("tram").special()).isTrue();
    }

    private Scene streetScene(boolean explicitRoad, AppendixModel appendix) {
        Scene scene = new Scene();
        ExistingSegment seg = new ExistingSegment();
        seg.id = "S-1";
        seg.dn = 400;
        seg.existingFlowTph = 10;
        seg.nextId = "SRC";
        seg.line = gf.createLineString(new Coordinate[]{new Coordinate(0, 0), new Coordinate(0, 120)});
        scene.segments.add(seg);

        scene.constraints.add(building("L", 6, 15, 26, 105, appendix));
        scene.constraints.add(building("R", 78, 15, 98, 105, appendix));
        if (explicitRoad) {
            SpatialConstraint road = new SpatialConstraint();
            road.id = "ROAD";
            road.type = "tdtp";
            road.geometry = roadPoly();
            road.rule = appendix.constraintRule("tdtp");
            scene.constraints.add(road);
        }

        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "OKS-A";
        oks.flowTph = 12;
        oks.connection = gf.createPoint(new Coordinate(88, 60));
        scene.oks.add(oks);
        scene.envelope();
        return scene;
    }

    private SpatialConstraint building(String id, double x0, double y0, double x1, double y1, AppendixModel appendix) {
        SpatialConstraint c = new SpatialConstraint();
        c.id = id;
        c.type = "oks";
        c.geometry = gf.createPolygon(new Coordinate[]{
                new Coordinate(x0, y0), new Coordinate(x1, y0),
                new Coordinate(x1, y1), new Coordinate(x0, y1),
                new Coordinate(x0, y0)
        });
        c.rule = appendix.constraintRule("oks");
        return c;
    }

    private Polygon roadPoly() {
        return gf.createPolygon(new Coordinate[]{
                new Coordinate(32, -220), new Coordinate(72, -220),
                new Coordinate(72, 340), new Coordinate(32, 340),
                new Coordinate(32, -220)
        });
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
