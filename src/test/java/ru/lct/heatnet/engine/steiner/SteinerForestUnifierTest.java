package ru.lct.heatnet.engine.steiner;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.scene.Scene;

class SteinerForestUnifierTest {

    private final GeometryFactory gf = new GeometryFactory();

    @Test
    void overlappingPipesCollapseToOneTree() {
        Variant variant = new Variant();
        variant.segments.add(seg("OKS-A", "TI-1", 12,
                new Coordinate(0, 0), new Coordinate(0, 40), new Coordinate(0, 80)));
        variant.segments.add(seg("OKS-B", "TI-1", 8,
                new Coordinate(30, 40), new Coordinate(0, 40), new Coordinate(0, 80)));
        TapPoint tap = new TapPoint();
        tap.id = "TI-1";
        tap.nodeId = "TI-1";
        tap.geometryMeters = gf.createPoint(new Coordinate(0, 80));
        variant.taps.add(tap);

        Map<String, Double> flow = new HashMap<>();
        flow.put("OKS-A", 12.0);
        flow.put("OKS-B", 8.0);
        Scene scene = new Scene();
        scene.envelope();
        ObstacleIndex obstacles = ObstacleIndex.build(scene);
        SteinerForestUnifier.unify(variant, obstacles, new AtomicInteger(1), Set.of("OKS-A", "OKS-B"), flow);

        double len = 0;
        for (NewSegment s : variant.segments) {
            len += s.lengthM;
        }
        assertThat(variant.taps).hasSize(1);
        assertThat(len).isLessThan(130);
        assertThat(len).isGreaterThan(90);
    }

    @Test
    void parallelRailsCollapseToOneSpine() {
        Variant variant = new Variant();
        variant.segments.add(seg("OKS-A", "TI-1", 10,
                new Coordinate(0, 0), new Coordinate(0, 80)));
        variant.segments.add(seg("OKS-B", "TI-1", 10,
                new Coordinate(12, 0), new Coordinate(12, 80)));
        TapPoint tap = new TapPoint();
        tap.id = "TI-1";
        tap.nodeId = "TI-1";
        tap.geometryMeters = gf.createPoint(new Coordinate(0, 80));
        variant.taps.add(tap);

        Map<String, Double> flow = new HashMap<>();
        flow.put("OKS-A", 10.0);
        flow.put("OKS-B", 10.0);
        Scene scene = new Scene();
        scene.envelope();
        ObstacleIndex obstacles = ObstacleIndex.build(scene);
        SteinerForestUnifier.unify(variant, obstacles, new AtomicInteger(1), Set.of("OKS-A", "OKS-B"), flow);

        double len = 0;
        for (NewSegment s : variant.segments) {
            len += s.lengthM;
        }
        assertThat(variant.taps).hasSize(1);
        assertThat(len).isLessThan(120);
        assertThat(len).isGreaterThan(80);
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
}
