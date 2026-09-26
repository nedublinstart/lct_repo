package ru.lct.heatnet.costing;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.HeatSource;
import ru.lct.heatnet.scene.Scene;

class ReconstructionCalculatorTest {

    @Test
    void marksSegmentWhenExtraFlowExceedsCapacity() {
        AppendixModel appendix = load();
        GeometryFactory gf = new GeometryFactory();
        Scene scene = new Scene();
        ExistingSegment seg = new ExistingSegment();
        seg.id = "S-1";
        seg.dn = 80;
        seg.existingFlowTph = 10;
        seg.nextId = "SRC-1";
        seg.line = gf.createLineString(new Coordinate[]{new Coordinate(0, 0), new Coordinate(100, 0)});
        scene.segments.add(seg);
        HeatSource src = new HeatSource();
        src.id = "SRC-1";
        src.point = gf.createPoint(new Coordinate(0, 0));
        scene.sources.add(src);

        Variant v = new Variant();
        TapPoint tap = new TapPoint();
        tap.existingObjectId = "S-1";
        tap.existingObjectKind = "heat_network";
        tap.extraFlowTph = 200;
        tap.requiredDiameter = 250;
        tap.geometryMeters = gf.createPoint(new Coordinate(25, 0));
        v.taps.add(tap);

        new ReconstructionCalculator().apply(v, scene, appendix);
        assertThat(v.reconstructionSegments).hasSize(1);
        assertThat(v.reconstructionSegments.get(0).requiredDn).isEqualTo(250);
        assertThat(v.reconstructionSegments.get(0).lengthM).isBetween(24.0, 26.0);
        assertThat(v.reconstructionSegments.get(0).existingObjectId).isEqualTo("S-1");
    }

    private static AppendixModel load() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
