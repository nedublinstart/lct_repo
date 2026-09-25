package ru.lct.heatnet.engine.steiner;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.Scene;

class SubmissionHygieneTest {

    private final GeometryFactory gf = new GeometryFactory();

    @Test
    void orphanTapWithoutPipeIsDropped() {
        Variant variant = new Variant();
        variant.segments.add(seg("1", "106", new Coordinate(0, 0), new Coordinate(0, 40)));
        TapPoint live = tap("TI-1", "106", new Coordinate(0, 40));
        TapPoint ghost = tap("TI-9", "999", new Coordinate(80, 80));
        variant.taps.add(live);
        variant.taps.add(ghost);
        Scene scene = sceneWithPipe(new Coordinate(0, 40), new Coordinate(10, 40));

        SubmissionHygiene.prepare(variant, scene);

        assertThat(variant.taps).extracting(t -> t.id).containsExactly("TI-1");
    }

    @Test
    void pipeEndWithinFourMetresMeetsTheNode() {
        Variant variant = new Variant();
        variant.segments.add(seg("1", "TN-1", new Coordinate(0, 0), new Coordinate(0, 20)));
        variant.segments.add(seg("TN-1", "106", new Coordinate(2.5, 20), new Coordinate(2.5, 40)));
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-1";
        node.geometryMeters = gf.createPoint(new Coordinate(0, 20));
        variant.technicalNodes.add(node);
        variant.taps.add(tap("TI-1", "106", new Coordinate(2.5, 40)));
        Scene scene = sceneWithPipe(new Coordinate(2.5, 40), new Coordinate(12, 40));

        SubmissionHygiene.prepare(variant, scene);

        NewSegment second = variant.segments.get(1);
        Coordinate start = second.geometryMeters.getCoordinateN(0);
        assertThat(start.distance(new Coordinate(0, 20))).isLessThan(0.2);
        assertThat(second.lengthM).isGreaterThan(20);
    }

    @Test
    void pipeEndSevenMetresFromItsDeclaredNodeIsPulled() {
        Variant variant = new Variant();
        variant.segments.add(seg("14", "TN-1", new Coordinate(0, 0), new Coordinate(3.7, 0)));
        variant.segments.add(seg("TN-1", "106", new Coordinate(11, 0), new Coordinate(11, 30)));
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-1";
        node.geometryMeters = gf.createPoint(new Coordinate(11, 0));
        variant.technicalNodes.add(node);
        variant.taps.add(tap("TI-1", "106", new Coordinate(11, 30)));
        Scene scene = sceneWithPipe(new Coordinate(11, 30), new Coordinate(21, 30));

        SubmissionHygiene.prepare(variant, scene);

        NewSegment stub = variant.segments.get(0);
        Coordinate end = stub.geometryMeters.getCoordinateN(1);
        assertThat(end.distance(new Coordinate(11, 0))).isLessThan(0.2);
        assertThat(stub.lengthM).isGreaterThan(10.0);
    }

    @Test
    void pipeEndFifteenMetresFromNodeIsLeftAlone() {
        Variant variant = new Variant();
        variant.segments.add(seg("14", "TN-1", new Coordinate(0, 0), new Coordinate(15, 0)));
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-1";
        node.geometryMeters = gf.createPoint(new Coordinate(0, 0));
        variant.technicalNodes.add(node);

        SubmissionHygiene.prepare(variant, null);

        Coordinate end = variant.segments.get(0).geometryMeters.getCoordinateN(1);
        assertThat(end.distance(new Coordinate(15, 0))).isLessThan(0.2);
    }

    @Test
    void tapDiameterIgnoresPipesThatDoNotReachIt() {
        Variant variant = new Variant();
        NewSegment local = seg("1", "106", new Coordinate(0, 0), new Coordinate(0, 10));
        local.dn = 100;
        NewSegment other = seg("2", "107", new Coordinate(100, 0), new Coordinate(100, 10));
        other.dn = 300;
        variant.segments.add(local);
        variant.segments.add(other);
        variant.taps.add(tap("TI-1", "106", new Coordinate(0, 10)));

        SubmissionHygiene.assignTapDiameters(variant);

        assertThat(variant.taps.get(0).requiredDiameter).isEqualTo(100);
    }

    private Scene sceneWithPipe(Coordinate a, Coordinate b) {
        Scene scene = new Scene();
        ExistingSegment seg = new ExistingSegment();
        seg.id = "S-1";
        seg.line = gf.createLineString(new Coordinate[]{a, b});
        scene.segments.add(seg);
        Chamber ch = new Chamber();
        ch.id = "106";
        ch.point = gf.createPoint(a);
        scene.chambers.add(ch);
        return scene;
    }

    private NewSegment seg(String from, String to, Coordinate a, Coordinate b) {
        NewSegment s = new NewSegment();
        s.fromId = from;
        s.toId = to;
        s.geometryMeters = gf.createLineString(new Coordinate[]{a, b});
        s.lengthM = s.geometryMeters.getLength();
        return s;
    }

    private TapPoint tap(String id, String node, Coordinate at) {
        TapPoint t = new TapPoint();
        t.id = id;
        t.nodeId = node;
        t.existingObjectId = node;
        t.geometryMeters = gf.createPoint(at);
        return t;
    }
}
