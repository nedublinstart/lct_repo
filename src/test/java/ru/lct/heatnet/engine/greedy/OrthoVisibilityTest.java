package ru.lct.heatnet.engine.greedy;

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
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.SmartRoutingEngine;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

class OrthoVisibilityTest {

    private final GeometryFactory gf = new GeometryFactory();

    @Test
    void usefulChordRejectsLongParkDiagonal() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        assertThat(OrthoPaths.usefulChord(obstacles, new Coordinate(0, 0), new Coordinate(80, 70))).isFalse();
        assertThat(OrthoPaths.longOpenDiagonal(new Coordinate(0, 0), new Coordinate(80, 70))).isTrue();
        List<Coordinate> elbow = OrthoPaths.bestElbow(obstacles, new Coordinate(0, 0), new Coordinate(80, 70));
        assertThat(elbow).hasSize(3);
        assertThat(OrthoPaths.rightAngle(elbow.get(0), elbow.get(1), elbow.get(2))).isTrue();
        assertThat(OrthoPaths.usefulElbow(obstacles, new Coordinate(0, 0), new Coordinate(80, 70))).isNull();
    }

    @Test
    void visibilityGoesAroundBuildingNotThroughIt() {
        AppendixModel appendix = appendix();
        Scene scene = new Scene();
        scene.constraints.add(building("BLD", 40, 40, 80, 100, appendix));
        scene.envelope();
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);
        VisibilityPathfinder vis = VisibilityPathfinder.build(obstacles);
        Coordinate start = new Coordinate(20, 70);
        Coordinate goal = new Coordinate(110, 70);
        List<Coordinate> path = vis.find(start, goal);
        assertThat(path).isNotNull();
        assertThat(path.size()).isGreaterThanOrEqualTo(2);
        LineString ls = gf.createLineString(path.toArray(new Coordinate[0]));
        Polygon core = (Polygon) scene.constraints.get(0).geometry.buffer(-1.0);
        assertThat(core.intersects(ls) && !core.touches(ls)).isFalse();
        for (int i = 1; i < path.size(); i++) {
            assertThat(OrthoPaths.longOpenDiagonal(path.get(i - 1), path.get(i))
                    && !obstacles.alongAvoid(path.get(i - 1), path.get(i), OrthoPaths.FACADE_M))
                    .as("диагональ %s→%s", path.get(i - 1), path.get(i))
                    .isFalse();
        }
    }

    @Test
    void steinerCrossesAvenueAtRightAngleAndDoesNotClipHouses() {
        AppendixModel appendix = appendix();
        Scene scene = avenueScene(appendix);
        Variant first = new SmartRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        }).get(0);
        assertThat(first.unconnectedOks).isEmpty();
        assertThat(first.segments).isNotEmpty();

        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);
        Polygon north = (Polygon) buildingPoly(10, 70, 90, 110);
        Polygon south = (Polygon) buildingPoly(10, -10, 50, 20);
        Polygon road = roadPoly();
        Coordinate roadAxis = new Coordinate(1, 0);
        double alongBad = 0;
        boolean sawSpecial = false;
        for (NewSegment seg : first.segments) {
            LineString ls = seg.geometryMeters;
            Polygon northCore = (Polygon) north.buffer(-1.0);
            Polygon southCore = (Polygon) south.buffer(-1.0);
            assertThat(northCore.intersects(ls) && !northCore.touches(ls)).as("север дома %s", seg.id).isFalse();
            assertThat(southCore.intersects(ls) && !southCore.touches(ls)).as("юг дома %s", seg.id).isFalse();
            for (int i = 1; i < ls.getNumPoints(); i++) {
                Coordinate a = ls.getCoordinateN(i - 1);
                Coordinate b = ls.getCoordinateN(i);
                assertThat(OrthoPaths.longOpenDiagonal(a, b)
                        && !obstacles.alongAvoid(a, b, OrthoPaths.FACADE_M))
                        .as("открытая диагональ на %s", seg.id)
                        .isFalse();
            }
            double hit = SpecialLayer.hitLength(ls, road);
            if ("special".equals(seg.layingMethod)) {
                sawSpecial = true;
                Coordinate[] c = ls.getCoordinates();
                double ang = SpecialLayer.crossingAngleDeg(c[0], c[c.length - 1], roadAxis);
                assertThat(ang).as("угол пересечения проспекта %s", seg.id).isGreaterThanOrEqualTo(65.0);
                assertThat(seg.lengthM).isLessThan(80);
            }
            if (hit >= 8) {
                Coordinate[] c = ls.getCoordinates();
                double ang = SpecialLayer.crossingAngleDeg(c[0], c[c.length - 1], roadAxis);
                if (ang < 65) {
                    alongBad += hit;
                }
            }
        }
        assertThat(sawSpecial).isTrue();
        assertThat(alongBad).isLessThan(10);
    }

    private Scene avenueScene(AppendixModel appendix) {
        Scene scene = new Scene();
        ExistingSegment seg = new ExistingSegment();
        seg.id = "S-1";
        seg.dn = 400;
        seg.existingFlowTph = 10;
        seg.nextId = "SRC";
        seg.line = gf.createLineString(new Coordinate[]{new Coordinate(-20, 90), new Coordinate(-20, 40)});
        scene.segments.add(seg);
        scene.constraints.add(building("N", 10, 70, 90, 110, appendix));
        scene.constraints.add(building("S", 10, -10, 50, 20, appendix));
        SpatialConstraint road = new SpatialConstraint();
        road.id = "AVENUE";
        road.type = "tdtp";
        road.geometry = roadPoly();
        road.rule = appendix.constraintRule("tdtp");
        scene.constraints.add(road);
        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "OKS-A";
        oks.flowTph = 12;
        oks.connection = gf.createPoint(new Coordinate(30, 5));
        scene.oks.add(oks);
        scene.envelope();
        return scene;
    }

    private SpatialConstraint building(String id, double x0, double y0, double x1, double y1, AppendixModel appendix) {
        SpatialConstraint c = new SpatialConstraint();
        c.id = id;
        c.type = "oks";
        c.geometry = buildingPoly(x0, y0, x1, y1);
        c.rule = appendix.constraintRule("oks");
        return c;
    }

    private Polygon buildingPoly(double x0, double y0, double x1, double y1) {
        return gf.createPolygon(new Coordinate[]{
                new Coordinate(x0, y0), new Coordinate(x1, y0),
                new Coordinate(x1, y1), new Coordinate(x0, y1),
                new Coordinate(x0, y0)
        });
    }

    private Polygon roadPoly() {
        return gf.createPolygon(new Coordinate[]{
                new Coordinate(-40, 28), new Coordinate(140, 28),
                new Coordinate(140, 58), new Coordinate(-40, 58),
                new Coordinate(-40, 28)
        });
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
