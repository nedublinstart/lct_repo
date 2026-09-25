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
    void nearestWallNotTheGableEvenIfTheRouteRunsAlongTheFacade() {
        ObstacleIndex obstacles = longHouse();
        Variant variant = new Variant();
        variant.segments.add(seg("1", "TN-2",
                new Coordinate(8, 13), new Coordinate(70, 13)));
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-2";
        node.geometryMeters = gf.createPoint(new Coordinate(70, 13));
        variant.technicalNodes.add(node);
        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "1";
        oks.flowTph = 10;
        oks.connection = gf.createPoint(new Coordinate(8, 13));

        LiquidRoutes.apply(variant, obstacles, null, new AtomicInteger(40),
                List.of(new OksPort(oks, new Coordinate(8, 13), new Coordinate(70, 13))));

        Coordinate[] pts = variant.segments.get(0).geometryMeters.getCoordinates();
        double stub = pts[0].distance(pts[1]);
        double dx = pts[1].x - pts[0].x;
        double dy = pts[1].y - pts[0].y;
        double ang = Math.toDegrees(Math.atan2(Math.abs(dy), Math.abs(dx)));
        assertThat(stub)
                .as("до ближней стены, не в торец: %s", java.util.Arrays.toString(pts))
                .isLessThan(12);
        assertThat(ang)
                .as("перпендикуляр длинной стене, не вдоль неё: %s", java.util.Arrays.toString(pts))
                .isGreaterThan(70);
        assertThat(pts[1].y).isGreaterThan(15);
    }

    @Test
    void longitudinalGapRunIsLiftedOffTheCarriageAndNotSpecial() {
        ObstacleIndex obstacles = streetGap();
        Variant variant = new Variant();
        variant.segments.add(seg("TN-A", "TN-B",
                new Coordinate(24, 12), new Coordinate(24, 72)));
        LiquidRoutes.apply(variant, obstacles, null, new AtomicInteger(50), List.of());
        for (NewSegment s : variant.segments) {
            if ("special".equals(s.layingMethod)) {
                assertThat(s.lengthM)
                        .as("спецметод только на коротком выходе из проезжей, не на всём ходе: %s", s.id)
                        .isLessThan(16);
            }
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            for (int i = 1; i < pts.length; i++) {
                if (pts[i - 1].distance(pts[i]) < 18) {
                    continue;
                }
                double mx = (pts[i - 1].x + pts[i].x) * 0.5;
                assertThat(Math.abs(mx - 24))
                        .as("длинный кусок не остаётся в середине проезжей: %s", java.util.Arrays.toString(pts))
                        .isGreaterThan(3);
            }
        }
    }

    @Test
    void perpendicularGapCrossingStaysSpecial() {
        ObstacleIndex obstacles = streetGap();
        Variant variant = new Variant();
        variant.segments.add(seg("TN-A", "TN-B",
                new Coordinate(4, 45), new Coordinate(44, 45)));
        LiquidRoutes.apply(variant, obstacles, null, new AtomicInteger(60), List.of());
        boolean saw = false;
        for (NewSegment s : variant.segments) {
            if ("special".equals(s.layingMethod)) {
                saw = true;
                assertThat(s.lengthM).isLessThan(30);
            }
        }
        assertThat(saw).as("поперечный проход через щель помечается спецметодом").isTrue();
    }

    @Test
    void nearestFacadeStaysPerpWhenTheGapIsInsideTheClosedBlock() {
        ObstacleIndex obstacles = gapBesideLongHouse();
        Variant variant = new Variant();
        variant.segments.add(seg("1", "TN-3",
                new Coordinate(30, 17), new Coordinate(30, 70)));
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-3";
        node.geometryMeters = gf.createPoint(new Coordinate(30, 70));
        variant.technicalNodes.add(node);
        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "1";
        oks.flowTph = 10;
        oks.connection = gf.createPoint(new Coordinate(30, 17));

        LiquidRoutes.apply(variant, obstacles, null, new AtomicInteger(80),
                List.of(new OksPort(oks, new Coordinate(30, 17), new Coordinate(30, 70))));

        Coordinate[] pts = variant.segments.get(0).geometryMeters.getCoordinates();
        double stub = pts[0].distance(pts[1]);
        assertThat(stub)
                .as("к ближней стене, не вдоль дома до торца: %s", java.util.Arrays.toString(pts))
                .isLessThan(8);
        assertThat(pts[1].y).isGreaterThan(20.4);
        assertThat(pts[1].y).isLessThan(24);
        assertThat(Math.abs(pts[1].x - 30)).isLessThan(3);
        for (int i = 2; i < pts.length; i++) {
            assertThat(obstacles.footprintCutM(pts[i - 1], pts[i]))
                    .as("после фасада ребро не входит в дом")
                    .isLessThan(1.2);
        }
    }

    @Test
    void cornerClipIsSkirtedAroundTheFootprint() {
        ObstacleIndex obstacles = longHouse();
        Variant variant = new Variant();
        variant.segments.add(seg("TN-A", "TN-B",
                new Coordinate(-2, 4), new Coordinate(10, -3)));
        LiquidRoutes.apply(variant, obstacles, null, new AtomicInteger(70), List.of());
        double cut = 0;
        double len = 0;
        for (NewSegment s : variant.segments) {
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            len += s.lengthM;
            for (int i = 1; i < pts.length; i++) {
                cut += obstacles.footprintCutM(pts[i - 1], pts[i]);
            }
        }
        assertThat(cut)
                .as("хорда угла не остаётся внутри корпуса, длина %.1f", len)
                .isLessThan(1.2);
        assertThat(len).isLessThan(70);
    }

    @Test
    void degreeTwoCornerStraightensOntoTheChord() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        Variant variant = new Variant();
        variant.segments.add(seg("TN-A", "TN-K", new Coordinate(0, 0), new Coordinate(0, 40)));
        variant.segments.add(seg("TN-K", "TN-B", new Coordinate(0, 40), new Coordinate(30, 40)));
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-K";
        node.geometryMeters = gf.createPoint(new Coordinate(0, 40));
        variant.technicalNodes.add(node);

        LiquidRoutes.apply(variant, obstacles, null, new AtomicInteger(90), List.of());

        double len = 0;
        for (NewSegment s : variant.segments) {
            len += s.lengthM;
        }
        Coordinate k = variant.technicalNodes.get(0).geometryMeters.getCoordinate();
        assertThat(len)
                .as("угол на стыке схлопывается в хорду, узел %s", k)
                .isLessThan(55);
        assertThat(k.distance(new Coordinate(0, 40))).isGreaterThan(5);
    }

    @Test
    void degreeTwoCornerStaysWhenTheChordCutsTheHouse() {
        ObstacleIndex obstacles = longHouse();
        Variant variant = new Variant();
        variant.segments.add(seg("TN-A", "TN-K", new Coordinate(50, -8), new Coordinate(50, 30)));
        variant.segments.add(seg("TN-K", "TN-B", new Coordinate(50, 30), new Coordinate(90, 30)));
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-K";
        node.geometryMeters = gf.createPoint(new Coordinate(50, 30));
        variant.technicalNodes.add(node);

        LiquidRoutes.apply(variant, obstacles, null, new AtomicInteger(91), List.of());

        double cut = 0;
        for (NewSegment s : variant.segments) {
            Coordinate[] pts = s.geometryMeters.getCoordinates();
            for (int i = 1; i < pts.length; i++) {
                cut += obstacles.footprintCutM(pts[i - 1], pts[i]);
            }
        }
        assertThat(cut).as("спрямление не ведёт трубу сквозь дом").isLessThan(1.2);
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

    private ObstacleIndex streetGap() {
        Scene scene = new Scene();
        AppendixModel appendix = appendix();
        scene.constraints.add(block(appendix, "L", 0, 0, 12, 90));
        scene.constraints.add(block(appendix, "R", 36, 0, 48, 90));
        scene.envelope();
        return ObstacleIndex.build(scene, appendix);
    }

    private SpatialConstraint block(AppendixModel appendix, String id, double x0, double y0, double x1, double y1) {
        SpatialConstraint c = new SpatialConstraint();
        c.id = id;
        c.type = "oks";
        c.rule = appendix.constraintRule("oks");
        c.geometry = gf.createPolygon(new Coordinate[]{
                new Coordinate(x0, y0), new Coordinate(x1, y0), new Coordinate(x1, y1),
                new Coordinate(x0, y1), new Coordinate(x0, y0)
        });
        return c;
    }

    private ObstacleIndex gapBesideLongHouse() {
        Scene scene = new Scene();
        AppendixModel appendix = appendix();
        scene.constraints.add(block(appendix, "A", 0, 0, 100, 20));
        scene.constraints.add(block(appendix, "B", 0, 24, 100, 44));
        scene.envelope();
        return ObstacleIndex.build(scene, appendix);
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
