package ru.lct.heatnet.engine.steiner;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

class LiquidRoutesTest {

    private final GeometryFactory gf = new GeometryFactory();

    @Test
    void throughHouseChordBecomesPerpFacadeThenOutside() {
        ObstacleIndex obstacles = longHouse();
        Variant variant = new Variant();
        variant.segments.add(seg("1", "TN-1",
                new Coordinate(50, 10), new Coordinate(50, -30)));
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-1";
        node.geometryMeters = gf.createPoint(new Coordinate(50, -30));
        variant.technicalNodes.add(node);
        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "1";
        oks.flowTph = 10;
        oks.connection = gf.createPoint(new Coordinate(50, 10));
        OksPort port = new OksPort(oks, new Coordinate(50, 10), new Coordinate(50, -2));

        LiquidRoutes.apply(variant, obstacles, null, new AtomicInteger(20), List.of(port));

        NewSegment stub = variant.segments.get(0);
        Coordinate[] pts = stub.geometryMeters.getCoordinates();
        assertThat(pts[0].distance(new Coordinate(50, 10))).isLessThan(0.3);
        assertThat(pts[0].distance(pts[1]))
                .as("короткий ⊥ к ближайшей стене, не сквозь корпус: %s", java.util.Arrays.toString(pts))
                .isLessThan(16);
        assertThat(pts[1].y).isLessThan(1.5);
        for (int i = 2; i < pts.length; i++) {
            assertThat(obstacles.segmentHitsAvoid(pts[i - 1], pts[i], 0, true))
                    .as("после фасада труба не режет дом %s→%s", pts[i - 1], pts[i])
                    .isFalse();
        }
    }

    @Test
    void alongHouseInteriorIsNotKept() {
        ObstacleIndex obstacles = longHouse();
        Variant variant = new Variant();
        variant.segments.add(seg("1", "TN-9",
                new Coordinate(10, 10), new Coordinate(90, 10)));
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-9";
        node.geometryMeters = gf.createPoint(new Coordinate(90, 10));
        variant.technicalNodes.add(node);
        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "1";
        oks.flowTph = 10;
        oks.connection = gf.createPoint(new Coordinate(10, 10));

        LiquidRoutes.apply(variant, obstacles, null, new AtomicInteger(30),
                List.of(new OksPort(oks, new Coordinate(10, 10), new Coordinate(10, -2))));

        boolean dug = false;
        for (NewSegment s : variant.segments) {
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            for (int i = 1; i < pts.length; i++) {
                if (obstacles.blocked(pts[i - 1]) && obstacles.blocked(pts[i]) && pts[i - 1].distance(pts[i]) > 20) {
                    dug = true;
                }
            }
        }
        assertThat(dug).isFalse();
        Coordinate tn = variant.technicalNodes.get(0).geometryMeters.getCoordinate();
        assertThat(obstacles.blocked(tn)).as("узел не остаётся внутри корпуса: %s", tn).isFalse();
    }

    @Test
    void kinkOutsideIsCutWhenTheChordIsLegal() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        Variant variant = new Variant();
        variant.segments.add(seg("TN-A", "TN-B",
                new Coordinate(0, 0), new Coordinate(0, 40), new Coordinate(30, 40)));
        LiquidRoutes.apply(variant, obstacles, null, new AtomicInteger(1), List.of());
        Coordinate[] pts = variant.segments.get(0).geometryMeters.getCoordinates();
        assertThat(pts.length).isLessThanOrEqualTo(3);
        assertThat(variant.segments.get(0).lengthM).isLessThan(55);
    }

    private ObstacleIndex longHouse() {
        Scene scene = new Scene();
        Polygon wall = gf.createPolygon(new Coordinate[]{
                new Coordinate(0, 0), new Coordinate(100, 0), new Coordinate(100, 20),
                new Coordinate(0, 20), new Coordinate(0, 0)
        });
        SpatialConstraint c = new SpatialConstraint();
        c.id = "BLD";
        c.type = "oks";
        AppendixModel appendix = appendix();
        c.rule = appendix.constraintRule("oks");
        c.geometry = wall;
        scene.constraints.add(c);
        scene.envelope();
        return ObstacleIndex.build(scene, appendix);
    }

    private NewSegment seg(String from, String to, Coordinate... pts) {
        NewSegment s = new NewSegment();
        s.fromId = from;
        s.toId = to;
        s.flowTph = 10;
        s.geometryMeters = gf.createLineString(pts);
        s.lengthM = s.geometryMeters.getLength();
        return s;
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
