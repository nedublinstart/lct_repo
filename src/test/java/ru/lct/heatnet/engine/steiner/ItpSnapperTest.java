package ru.lct.heatnet.engine.steiner;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

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
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

class ItpSnapperTest {

    private final GeometryFactory gf = new GeometryFactory();

    @Test
    void joinCutsAroundTheCornerAndHitsThePathInFront() {
        ObstacleIndex obstacles = house();
        Coordinate origin = new Coordinate(40, 10);
        List<Coordinate> around = List.of(
                new Coordinate(62, 10),
                new Coordinate(62, 4),
                new Coordinate(0, 4));
        List<Coordinate> joined = ItpSnapper.join(obstacles, origin, around);
        assertThat(joined.get(0).distance(origin)).isLessThan(0.2);
        assertThat(joined.get(1).x).isLessThan(50.0);
        boolean hitsFront = false;
        for (Coordinate c : joined) {
            if (Math.abs(c.x - 40) <= 8 && Math.abs(c.y - 4) <= 4) {
                hitsFront = true;
            }
        }
        assertThat(hitsFront).as("ввод должен попасть на путь перед фасадом, а не за угол").isTrue();
        double aroundLen = 22 + 6 + 62;
        double newLen = 0;
        for (int i = 1; i < joined.size(); i++) {
            newLen += joined.get(i - 1).distance(joined.get(i));
        }
        assertThat(newLen).isLessThan(aroundLen * 0.55);
    }

    @Test
    void snapReplacesAroundCornerSpurWithStraightStub() {
        ObstacleIndex obstacles = house();
        Variant variant = new Variant();
        variant.segments.add(seg("TN-J", "TI-1", 10, new Coordinate(62, 4), new Coordinate(80, 4)));
        variant.segments.add(seg("TN-J", "TN-W", 10, new Coordinate(62, 4), new Coordinate(0, 4)));
        variant.segments.add(seg("OKS-A", "TN-C", 10, new Coordinate(40, 10), new Coordinate(62, 10)));
        variant.segments.add(seg("TN-C", "TN-J", 10, new Coordinate(62, 10), new Coordinate(62, 4)));
        TapPoint tap = new TapPoint();
        tap.id = "TI-1";
        tap.nodeId = "TI-1";
        tap.geometryMeters = gf.createPoint(new Coordinate(80, 4));
        variant.taps.add(tap);

        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "OKS-A";
        oks.flowTph = 10;
        oks.connection = gf.createPoint(new Coordinate(40, 10));
        OksPort port = new OksPort(oks, new Coordinate(40, 10), new Coordinate(62, 10));

        ItpSnapper.snap(variant, obstacles, new AtomicInteger(20), List.of(port));

        NewSegment stub = null;
        for (NewSegment s : variant.segments) {
            if ("OKS-A".equals(s.fromId)) {
                stub = s;
                break;
            }
        }
        assertThat(stub).isNotNull();
        Coordinate[] pts = stub.geometryMeters.getCoordinates();
        assertThat(pts[0].distance(new Coordinate(40, 10))).isLessThan(0.3);
        assertThat(pts[pts.length - 1].x).isCloseTo(40, within(12.0));
        assertThat(pts[pts.length - 1].y).isCloseTo(4, within(6.0));
        assertThat(pts[1].x).isLessThan(52.0);
    }

    private ObstacleIndex house() {
        Scene scene = new Scene();
        Polygon wall = gf.createPolygon(new Coordinate[]{
                new Coordinate(20, 10), new Coordinate(60, 10), new Coordinate(60, 50),
                new Coordinate(20, 50), new Coordinate(20, 10)
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

    private NewSegment seg(String from, String to, double flow, Coordinate... pts) {
        NewSegment s = new NewSegment();
        s.fromId = from;
        s.toId = to;
        s.flowTph = flow;
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
