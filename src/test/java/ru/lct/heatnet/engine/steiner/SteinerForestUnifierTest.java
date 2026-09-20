package ru.lct.heatnet.engine.steiner;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
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
import ru.lct.heatnet.scene.ProspectiveOks;
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

    @Test
    void disconnectedTreesKeepBothOksAndTaps() {
        Variant variant = new Variant();
        variant.segments.add(seg("OKS-A", "TI-1", 10,
                new Coordinate(0, 0), new Coordinate(0, 40)));
        variant.segments.add(seg("OKS-B", "TI-2", 10,
                new Coordinate(200, 0), new Coordinate(200, 40)));
        TapPoint tap1 = new TapPoint();
        tap1.id = "TI-1";
        tap1.nodeId = "TI-1";
        tap1.geometryMeters = gf.createPoint(new Coordinate(0, 40));
        TapPoint tap2 = new TapPoint();
        tap2.id = "TI-2";
        tap2.nodeId = "TI-2";
        tap2.geometryMeters = gf.createPoint(new Coordinate(200, 40));
        variant.taps.add(tap1);
        variant.taps.add(tap2);

        Map<String, Double> flow = new HashMap<>();
        flow.put("OKS-A", 10.0);
        flow.put("OKS-B", 10.0);
        Scene scene = new Scene();
        scene.envelope();
        ObstacleIndex obstacles = ObstacleIndex.build(scene);
        SteinerForestUnifier.unify(variant, obstacles, new AtomicInteger(1), Set.of("OKS-A", "OKS-B"), flow);

        Set<String> from = new HashSet<>();
        for (NewSegment s : variant.segments) {
            from.add(s.fromId);
        }
        assertThat(from).contains("OKS-A", "OKS-B");
        assertThat(variant.taps).hasSize(2);
    }

    @Test
    void islandNearTreeIsStitchedWithoutNewTap() {
        Variant variant = new Variant();
        variant.segments.add(seg("OKS-A", "TI-1", 10,
                new Coordinate(0, 0), new Coordinate(0, 40)));
        variant.segments.add(seg("OKS-B", "TN-X", 8,
                new Coordinate(20, 0), new Coordinate(20, 20)));
        variant.taps.add(tap("TI-1", "TI-1", new Coordinate(0, 40)));

        Scene scene = new Scene();
        scene.envelope();
        ObstacleIndex obstacles = ObstacleIndex.build(scene);
        SteinerForestUnifier.stitchToExisting(variant, scene, obstacles, null, new AtomicInteger(1),
                List.of(port("OKS-A", 0, 0), port("OKS-B", 20, 0)));

        assertThat(reachesTap(variant, "OKS-A")).isTrue();
        assertThat(reachesTap(variant, "OKS-B")).as("остров 20 м от дерева должен сесть на ту же врезку").isTrue();
        assertThat(variant.taps).hasSize(1);
    }

    @Test
    void stubSittingOnTapIsWiredById() {
        Variant variant = new Variant();
        variant.segments.add(seg("OKS-A", "TI-1", 10,
                new Coordinate(0, 0), new Coordinate(0, 40)));
        variant.segments.add(seg("OKS-B", "TN-X", 8,
                new Coordinate(3, 40), new Coordinate(3, 52)));
        variant.taps.add(tap("TI-1", "TI-1", new Coordinate(0, 40)));

        Scene scene = new Scene();
        scene.envelope();
        ObstacleIndex obstacles = ObstacleIndex.build(scene);
        SteinerForestUnifier.stitchToExisting(variant, scene, obstacles, null, new AtomicInteger(1),
                List.of(port("OKS-A", 0, 0), port("OKS-B", 3, 40)));

        assertThat(reachesTap(variant, "OKS-B")).isTrue();
        assertThat(variant.taps).hasSize(1);
    }

    @Test
    void farTappedTreesStaySeparate() {
        Variant variant = new Variant();
        variant.segments.add(seg("OKS-A", "TI-1", 10,
                new Coordinate(0, 0), new Coordinate(0, 40)));
        variant.segments.add(seg("OKS-B", "TI-2", 10,
                new Coordinate(200, 0), new Coordinate(200, 40)));
        variant.taps.add(tap("TI-1", "TI-1", new Coordinate(0, 40)));
        variant.taps.add(tap("TI-2", "TI-2", new Coordinate(200, 40)));

        Scene scene = new Scene();
        scene.envelope();
        ObstacleIndex obstacles = ObstacleIndex.build(scene);
        SteinerForestUnifier.stitchToExisting(variant, scene, obstacles, null, new AtomicInteger(1),
                List.of(port("OKS-A", 0, 0), port("OKS-B", 200, 0)));

        assertThat(variant.taps).hasSize(2);
        assertThat(reachesTap(variant, "OKS-A")).isTrue();
        assertThat(reachesTap(variant, "OKS-B")).isTrue();
        double extra = 0;
        for (NewSegment s : variant.segments) {
            extra += s.lengthM;
        }
        assertThat(extra).as("нельзя стягивать два валидных дерева сотнями метров").isLessThan(120);
    }

    @Test
    void islandHundredMetresIsStitchedWithoutNewTap() {
        Variant variant = new Variant();
        variant.segments.add(seg("OKS-A", "TI-1", 10,
                new Coordinate(0, 0), new Coordinate(0, 40)));
        variant.segments.add(seg("OKS-B", "TN-X", 8,
                new Coordinate(100, 0), new Coordinate(100, 20)));
        variant.taps.add(tap("TI-1", "TI-1", new Coordinate(0, 40)));

        Scene scene = new Scene();
        scene.envelope();
        ObstacleIndex obstacles = ObstacleIndex.build(scene);
        SteinerForestUnifier.stitchToExisting(variant, scene, obstacles, null, new AtomicInteger(1),
                List.of(port("OKS-A", 0, 0), port("OKS-B", 100, 0)));

        assertThat(reachesTap(variant, "OKS-B")).as("остров ~100 м стыкуем к дереву без третьей врезки").isTrue();
        assertThat(variant.taps).hasSize(1);
    }

    @Test
    void twoTapsOnOneTreeKeepOne() {
        Variant variant = new Variant();
        variant.segments.add(seg("OKS-A", "TI-1", 10,
                new Coordinate(0, 0), new Coordinate(0, 40)));
        variant.segments.add(seg("TI-1", "TI-2", 10,
                new Coordinate(0, 40), new Coordinate(0, 50)));
        variant.taps.add(tap("TI-1", "TI-1", new Coordinate(0, 40)));
        variant.taps.add(tap("TI-2", "TI-2", new Coordinate(0, 50)));
        SteinerForestUnifier.dropDuplicateTaps(variant);
        assertThat(variant.taps).hasSize(1);
        assertThat(reachesTap(variant, "OKS-A")).isTrue();
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

    private TapPoint tap(String id, String nodeId, Coordinate at) {
        TapPoint t = new TapPoint();
        t.id = id;
        t.nodeId = nodeId;
        t.geometryMeters = gf.createPoint(at);
        return t;
    }

    private static OksPort port(String id, double x, double y) {
        ProspectiveOks oks = new ProspectiveOks();
        oks.id = id;
        oks.flowTph = 10;
        Coordinate c = new Coordinate(x, y);
        return new OksPort(oks, c, c);
    }

    private static boolean reachesTap(Variant v, String oks) {
        Set<String> taps = new HashSet<>();
        for (TapPoint t : v.taps) {
            if (t.id != null) {
                taps.add(t.id);
            }
            if (t.nodeId != null) {
                taps.add(t.nodeId);
            }
        }
        Map<String, List<String>> adj = new HashMap<>();
        for (NewSegment s : v.segments) {
            if (s.fromId == null || s.toId == null) {
                continue;
            }
            adj.computeIfAbsent(s.fromId, k -> new ArrayList<>()).add(s.toId);
            adj.computeIfAbsent(s.toId, k -> new ArrayList<>()).add(s.fromId);
        }
        Set<String> seen = new HashSet<>();
        ArrayDeque<String> q = new ArrayDeque<>();
        q.add(oks);
        seen.add(oks);
        while (!q.isEmpty()) {
            String u = q.removeFirst();
            if (taps.contains(u)) {
                return true;
            }
            for (String n : adj.getOrDefault(u, List.of())) {
                if (seen.add(n)) {
                    q.add(n);
                }
            }
        }
        return false;
    }
}
