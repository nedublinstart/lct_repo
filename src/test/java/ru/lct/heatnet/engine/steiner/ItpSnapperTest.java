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

    @Test
    void joinFromInsideNearCornerGoesStraightToTheFrontNotAroundGable() {
        ObstacleIndex obstacles = house();
        Coordinate origin = new Coordinate(58, 12);
        List<Coordinate> around = List.of(
                new Coordinate(62, 12),
                new Coordinate(62, 4),
                new Coordinate(0, 4));
        List<Coordinate> joined = ItpSnapper.join(obstacles, origin, around);
        assertThat(joined.get(0).distance(origin)).isLessThan(0.2);
        Coordinate hit = joined.get(Math.min(1, joined.size() - 1));
        for (Coordinate c : joined) {
            if (Math.abs(c.y - 4) <= 3) {
                hit = c;
                break;
            }
        }
        assertThat(hit.x)
                .as("ввод к пути перед фасадом, не за восточный угол: %s", joined)
                .isLessThan(61.0);
        assertThat(hit.x).isGreaterThan(50.0);
        assertThat(Math.abs(hit.y - 4)).isLessThan(6.0);
        assertThat(joined.stream().noneMatch(c -> c.x >= 61.5)).isTrue();
    }

    @Test
    void cutStubKeepsItpAsLeafWithoutTheTrunk() {
        ObstacleIndex obstacles = house();
        Coordinate origin = new Coordinate(40, 10);
        List<Coordinate> around = List.of(
                new Coordinate(62, 10),
                new Coordinate(62, 4),
                new Coordinate(0, 4));
        ItpSnapper.Cut cut = ItpSnapper.cutStub(obstacles, origin, around);
        assertThat(cut.stub.get(0).distance(origin)).isLessThan(0.2);
        assertThat(cut.stub.size()).isGreaterThanOrEqualTo(2);
        double stubLen = 0;
        for (int i = 1; i < cut.stub.size(); i++) {
            stubLen += cut.stub.get(i - 1).distance(cut.stub.get(i));
        }
        assertThat(stubLen).isLessThan(20);
        assertThat(cut.rest).isNotEmpty();
        assertThat(cut.rest.get(cut.rest.size() - 1).distance(new Coordinate(0, 4))).isLessThan(0.5);
    }

    @Test
    void longExclusiveSpurSnapsToNearbyTap() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        Variant variant = new Variant();
        variant.segments.add(seg("OKS-A", "TN-J", 10, new Coordinate(0, 10), new Coordinate(200, 10)));
        variant.segments.add(seg("TN-J", "TI-FAR", 20, new Coordinate(200, 10), new Coordinate(200, 80)));
        variant.segments.add(seg("OKS-B", "TN-J", 10, new Coordinate(200, 40), new Coordinate(200, 10)));
        TapPoint far = new TapPoint();
        far.id = "TI-FAR";
        far.nodeId = "TI-FAR";
        far.geometryMeters = gf.createPoint(new Coordinate(200, 80));
        variant.taps.add(far);
        TapPoint near = new TapPoint();
        near.id = "TI-NEAR";
        near.nodeId = "TI-NEAR";
        near.geometryMeters = gf.createPoint(new Coordinate(100, 10));
        variant.taps.add(near);

        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "OKS-A";
        oks.flowTph = 10;
        oks.connection = gf.createPoint(new Coordinate(0, 10));
        OksPort port = new OksPort(oks, new Coordinate(0, 10), new Coordinate(8, 10));

        ItpSnapper.snap(variant, obstacles, new AtomicInteger(20), List.of(port));

        NewSegment stub = null;
        for (NewSegment s : variant.segments) {
            if ("OKS-A".equals(s.fromId)) {
                stub = s;
                break;
            }
        }
        assertThat(stub).isNotNull();
        assertThat(stub.lengthM).isLessThan(120);
        assertThat(stub.toId).isEqualTo("TI-NEAR");
    }

    @Test
    void retapLongSpurOntoNearbyExistingChamber() {
        Scene scene = new Scene();
        ru.lct.heatnet.scene.Chamber ch = new ru.lct.heatnet.scene.Chamber();
        ch.id = "107";
        ch.dn = 400;
        ch.incidentCount = 2;
        ch.nextId = "S-1";
        ch.point = gf.createPoint(new Coordinate(80, 10));
        scene.chambers.add(ch);
        ru.lct.heatnet.scene.ExistingSegment net = new ru.lct.heatnet.scene.ExistingSegment();
        net.id = "S-1";
        net.dn = 400;
        net.existingFlowTph = 10;
        net.nextId = "SRC";
        net.line = gf.createLineString(new Coordinate[]{new Coordinate(80, 10), new Coordinate(80, 200)});
        scene.segments.add(net);
        ru.lct.heatnet.scene.HeatSource src = new ru.lct.heatnet.scene.HeatSource();
        src.id = "SRC";
        src.point = gf.createPoint(new Coordinate(80, 200));
        scene.sources.add(src);
        scene.envelope();
        AppendixModel appendix = appendix();
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);
        TapCatalog catalog = TapCatalog.build(scene, appendix, obstacles);

        Variant variant = new Variant();
        variant.segments.add(seg("OKS-A", "TI-FAR", 80, new Coordinate(0, 10), new Coordinate(200, 10)));
        TapPoint far = new TapPoint();
        far.id = "TI-FAR";
        far.nodeId = "TI-FAR";
        far.existingObjectId = "S-FAR";
        far.existingObjectKind = "heat_network";
        far.cost = 5_000_000;
        far.geometryMeters = gf.createPoint(new Coordinate(200, 10));
        variant.taps.add(far);

        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "OKS-A";
        oks.flowTph = 80;
        oks.connection = gf.createPoint(new Coordinate(0, 10));
        OksPort port = new OksPort(oks, new Coordinate(0, 10), new Coordinate(8, 10));

        ItpSnapper.retapIfCheaper(variant, obstacles, new AtomicInteger(30), List.of(port),
                null, appendix, catalog);

        assertThat(variant.taps).isNotEmpty();
        assertThat(variant.taps).extracting(t -> t.existingObjectId).contains("107");
        NewSegment stub = null;
        for (NewSegment s : variant.segments) {
            if ("OKS-A".equals(s.fromId)) {
                stub = s;
                break;
            }
        }
        assertThat(stub).isNotNull();
        assertThat(stub.lengthM).isLessThan(100);
        assertThat(stub.toId).isEqualTo("107");
    }

    @Test
    void consolidateDropsPipeTapWhenTreeIsCheaper() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        Variant variant = new Variant();
        variant.segments.add(seg("OKS-A", "CH-1", 10, new Coordinate(10, 10), new Coordinate(90, 10)));
        variant.segments.add(seg("OKS-B", "TI-MAIN", 10, new Coordinate(10, 22), new Coordinate(200, 22)));
        TapPoint pipe = new TapPoint();
        pipe.id = "TI-PIPE";
        pipe.nodeId = "CH-1";
        pipe.cost = 5_000_000;
        pipe.geometryMeters = gf.createPoint(new Coordinate(90, 10));
        variant.taps.add(pipe);
        ru.lct.heatnet.engine.NewChamber ch = new ru.lct.heatnet.engine.NewChamber();
        ch.id = "CH-1";
        ch.atTap = true;
        ch.cost = 3_000_000;
        ch.geometryMeters = gf.createPoint(new Coordinate(90, 10));
        variant.chambers.add(ch);
        TapPoint main = new TapPoint();
        main.id = "TI-MAIN";
        main.nodeId = "TI-MAIN";
        main.cost = 5_000_000;
        main.geometryMeters = gf.createPoint(new Coordinate(200, 22));
        variant.taps.add(main);

        ProspectiveOks a = new ProspectiveOks();
        a.id = "OKS-A";
        a.flowTph = 10;
        a.connection = gf.createPoint(new Coordinate(10, 10));
        ProspectiveOks b = new ProspectiveOks();
        b.id = "OKS-B";
        b.flowTph = 10;
        b.connection = gf.createPoint(new Coordinate(10, 22));

        ItpSnapper.consolidate(variant, obstacles, new AtomicInteger(40),
                List.of(new OksPort(a, new Coordinate(10, 10), new Coordinate(10, 12)),
                        new OksPort(b, new Coordinate(10, 22), new Coordinate(10, 24))),
                null, appendix());

        assertThat(variant.taps).extracting(t -> t.id).contains("TI-MAIN").doesNotContain("TI-PIPE");
        assertThat(variant.chambers).isEmpty();
        NewSegment stub = null;
        for (NewSegment s : variant.segments) {
            if ("OKS-A".equals(s.fromId)) {
                stub = s;
                break;
            }
        }
        assertThat(stub).isNotNull();
        assertThat(stub.lengthM).isLessThan(40);
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
