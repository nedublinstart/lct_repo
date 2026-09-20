package ru.lct.heatnet.costing;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.Variant;

class DiameterSelectorTest {

    private final DiameterSelector selector = new DiameterSelector();

    @Test
    void picksMinimalDiameterThatFitsFlow() {
        AppendixModel appendix = load();
        assertThat(selector.select(10, appendix)).isEqualTo(80);
        assertThat(selector.select(12, appendix)).isEqualTo(80);
        assertThat(selector.select(13, appendix)).isEqualTo(80);
        assertThat(selector.select(13.3, appendix)).isEqualTo(100);
        assertThat(selector.select(500, appendix)).isEqualTo(400);
    }

    @Test
    void upgradesDiameterWhenRunIsTooLong() {
        AppendixModel appendix = load();
        NewSegment seg = new NewSegment();
        seg.flowTph = 10;
        seg.lengthM = 10_000;
        selector.apply(List.of(seg), appendix);
        assertThat(seg.dn).isGreaterThan(80);
    }

    @Test
    void applyDoesNotMutateAppendixOrderPermanentlyAcrossCalls() {
        AppendixModel appendix = load();
        List<NewSegment> segs = new ArrayList<>();
        NewSegment a = new NewSegment();
        a.flowTph = 50;
        a.lengthM = 10;
        segs.add(a);
        selector.apply(segs, appendix);
        assertThat(a.dn).isEqualTo(150);
    }

    @Test
    void consecutiveSameDiameterAlongTreeBumpsTowardRoot() {
        AppendixModel appendix = load();
        org.locationtech.jts.geom.GeometryFactory gf = new org.locationtech.jts.geom.GeometryFactory();
        Variant variant = new Variant();
        NewSegment leaf = new NewSegment();
        leaf.id = "NS-1";
        leaf.fromId = "OKS-A";
        leaf.toId = "CH-1";
        leaf.flowTph = 10;
        leaf.lengthM = 200;
        leaf.geometryMeters = gf.createLineString(new org.locationtech.jts.geom.Coordinate[]{
                new org.locationtech.jts.geom.Coordinate(200, 0),
                new org.locationtech.jts.geom.Coordinate(0, 0)
        });
        NewSegment trunk = new NewSegment();
        trunk.id = "NS-2";
        trunk.fromId = "CH-1";
        trunk.toId = "TAP";
        trunk.flowTph = 10;
        trunk.lengthM = 200;
        trunk.geometryMeters = gf.createLineString(new org.locationtech.jts.geom.Coordinate[]{
                new org.locationtech.jts.geom.Coordinate(0, 0),
                new org.locationtech.jts.geom.Coordinate(-200, 0)
        });
        variant.segments.add(leaf);
        variant.segments.add(trunk);
        ru.lct.heatnet.engine.TapPoint tap = new ru.lct.heatnet.engine.TapPoint();
        tap.id = "TI-1";
        tap.nodeId = "TAP";
        tap.existingObjectId = "S-1";
        variant.taps.add(tap);
        selector.applyTree(variant, appendix);
        assertThat(leaf.dn).isEqualTo(80);
        assertThat(trunk.dn).isGreaterThan(80);
    }

    @Test
    void weighByTreeSumsLeavesTowardTap() {
        AppendixModel appendix = load();
        org.locationtech.jts.geom.GeometryFactory gf = new org.locationtech.jts.geom.GeometryFactory();
        Variant variant = new Variant();
        NewSegment a = new NewSegment();
        a.id = "NS-1";
        a.fromId = "OKS-A";
        a.toId = "TN-1";
        a.flowTph = 10;
        a.lengthM = 10;
        a.geometryMeters = gf.createLineString(new org.locationtech.jts.geom.Coordinate[]{
                new org.locationtech.jts.geom.Coordinate(0, 10),
                new org.locationtech.jts.geom.Coordinate(0, 0)
        });
        NewSegment b = new NewSegment();
        b.id = "NS-2";
        b.fromId = "OKS-B";
        b.toId = "TN-1";
        b.flowTph = 20;
        b.lengthM = 10;
        b.geometryMeters = gf.createLineString(new org.locationtech.jts.geom.Coordinate[]{
                new org.locationtech.jts.geom.Coordinate(10, 0),
                new org.locationtech.jts.geom.Coordinate(0, 0)
        });
        NewSegment trunk = new NewSegment();
        trunk.id = "NS-3";
        trunk.fromId = "TN-1";
        trunk.toId = "TAP";
        trunk.flowTph = 1;
        trunk.lengthM = 10;
        trunk.geometryMeters = gf.createLineString(new org.locationtech.jts.geom.Coordinate[]{
                new org.locationtech.jts.geom.Coordinate(0, 0),
                new org.locationtech.jts.geom.Coordinate(0, -10)
        });
        variant.segments.add(a);
        variant.segments.add(b);
        variant.segments.add(trunk);
        ru.lct.heatnet.engine.TapPoint tap = new ru.lct.heatnet.engine.TapPoint();
        tap.id = "TI-1";
        tap.nodeId = "TAP";
        variant.taps.add(tap);
        selector.applyTree(variant, appendix);
        assertThat(a.flowTph).isEqualTo(10);
        assertThat(b.flowTph).isEqualTo(20);
        assertThat(trunk.flowTph).isEqualTo(30);
        assertThat(trunk.dn).isGreaterThanOrEqualTo(a.dn);
        assertThat(trunk.dn).isGreaterThanOrEqualTo(b.dn);
    }

    private static AppendixModel load() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
