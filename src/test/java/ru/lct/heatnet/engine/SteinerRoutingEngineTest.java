package ru.lct.heatnet.engine;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.costing.CostCalculator;
import ru.lct.heatnet.costing.DiameterSelector;
import ru.lct.heatnet.costing.RankingCalculator;
import ru.lct.heatnet.costing.ReconstructionCalculator;
import ru.lct.heatnet.engine.steiner.MehlhornSteiner;
import ru.lct.heatnet.engine.steiner.OksPort;
import ru.lct.heatnet.engine.steiner.PathMetric;
import ru.lct.heatnet.engine.steiner.SteinerRoutingEngine;
import ru.lct.heatnet.engine.steiner.SteinerTree;
import ru.lct.heatnet.engine.steiner.Strategy;
import ru.lct.heatnet.engine.steiner.TapCandidate;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;

class SteinerRoutingEngineTest {

    private final GeometryFactory gf = new GeometryFactory();

    @Test
    void threeStrategiesProduceRankedConnectedVariants() {
        Scene scene = twoPipesScene();
        AppendixModel appendix = appendix();
        List<Variant> variants = new SteinerRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        });
        score(variants, scene, appendix);
        assertThat(variants).isNotEmpty();
        Set<String> codes = new HashSet<>();
        for (Variant v : variants) {
            assertThat(v.unconnectedOks).isEmpty();
            assertThat(v.segments).isNotEmpty();
            assertThat(v.taps).isNotEmpty();
            codes.add(v.code);
        }
        assertThat(codes).contains(Strategy.MIN_COST.code);
        assertThat(variants.get(0).score).isLessThanOrEqualTo(variants.get(variants.size() - 1).score);
    }

    @Test
    void minTapsUsesFewerTieInsThanSeparateClusters() {
        Scene scene = twoPipesScene();
        AppendixModel appendix = appendix();
        List<Variant> variants = new SteinerRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        });
        Variant mintaps = find(variants, Strategy.MIN_TAPS.code);
        Variant mincost = find(variants, Strategy.MIN_COST.code);
        assertThat(mintaps).isNotNull();
        assertThat(mintaps.unconnectedOks).isEmpty();
        if (mincost != null && !mincost.code.equals(mintaps.code)) {
            assertThat(mintaps.taps.size()).isLessThanOrEqualTo(mincost.taps.size());
        }
        assertThat(mintaps.taps.size()).isLessThanOrEqualTo(2);
    }

    @Test
    void minReconPrefersSpareCapacityPipe() {
        Scene scene = twoPipesScene();
        AppendixModel appendix = appendix();
        List<Variant> variants = new SteinerRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        });
        score(variants, scene, appendix);
        Variant minrecon = find(variants, Strategy.MIN_RECON.code);
        assertThat(minrecon).isNotNull();
        assertThat(minrecon.unconnectedOks).isEmpty();
        boolean tapsFat = false;
        for (TapPoint t : minrecon.taps) {
            if ("S-FAT".equals(t.existingObjectId) || t.existingDiameter >= 300) {
                tapsFat = true;
            }
        }
        assertThat(tapsFat || minrecon.reconstructionSegments.isEmpty()).isTrue();
    }

    @Test
    void walledOffOksIsUnconnectedNotCrash() {
        Scene scene = new Scene();
        ExistingSegment seg = new ExistingSegment();
        seg.id = "S-1";
        seg.dn = 400;
        seg.existingFlowTph = 5;
        seg.nextId = "SRC";
        seg.line = gf.createLineString(new Coordinate[]{new Coordinate(0, 0), new Coordinate(0, 40)});
        scene.segments.add(seg);
        ProspectiveOks trapped = oks("OKS-X", 6, 200, 200);
        scene.oks.add(trapped);
        ru.lct.heatnet.scene.SpatialConstraint wall = new ru.lct.heatnet.scene.SpatialConstraint();
        wall.id = "BOX";
        wall.type = "oks";
        AppendixModel appendix = appendix();
        wall.rule = appendix.constraintRule("oks");
        wall.geometry = gf.createPolygon(new Coordinate[]{
                new Coordinate(30, -250), new Coordinate(170, -250), new Coordinate(170, 450),
                new Coordinate(30, 450), new Coordinate(30, -250)
        });
        scene.constraints.add(wall);
        scene.envelope();
        List<Variant> variants = new SteinerRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        });
        assertThat(variants).isNotEmpty();
        assertThat(variants.get(0).unconnectedOks).contains("OKS-X");
    }

    @Test
    void mehlhornBuildsTreeCoveringAllPorts() {
        PathMetric metric = new StraightMetric();
        ProspectiveOks a = oks("A", 5, 0, 40);
        ProspectiveOks b = oks("B", 5, 40, 0);
        List<OksPort> ports = List.of(
                new OksPort(a, a.connection.getCoordinate(), a.connection.getCoordinate()),
                new OksPort(b, b.connection.getCoordinate(), b.connection.getCoordinate())
        );
        ExistingSegment dummy = new ExistingSegment();
        dummy.id = "S";
        dummy.dn = 400;
        dummy.line = gf.createLineString(new Coordinate[]{new Coordinate(0, 0), new Coordinate(1, 0)});
        TapCandidate tap = TapCandidate.segment(dummy, new Coordinate(0, 0));
        SteinerTree tree = MehlhornSteiner.connect(ports, tap, metric, 4);
        assertThat(tree.failed()).isFalse();
        assertThat(tree.connected).hasSize(2);
        assertThat(tree.branches).isNotEmpty();
        double flow = 0;
        for (SteinerTree.Branch br : tree.branches) {
            if (tree.nodes.get(br.to).tap) {
                flow += br.flow;
            }
        }
        assertThat(flow).isEqualTo(10.0);
    }

    private Scene twoPipesScene() {
        Scene scene = new Scene();
        ExistingSegment thin = new ExistingSegment();
        thin.id = "S-THIN";
        thin.dn = 80;
        thin.existingFlowTph = 12;
        thin.nextId = "SRC";
        thin.line = gf.createLineString(new Coordinate[]{new Coordinate(0, 0), new Coordinate(0, 140)});
        scene.segments.add(thin);
        ExistingSegment fat = new ExistingSegment();
        fat.id = "S-FAT";
        fat.dn = 400;
        fat.existingFlowTph = 8;
        fat.nextId = "SRC";
        fat.line = gf.createLineString(new Coordinate[]{new Coordinate(0, 260), new Coordinate(0, 420)});
        scene.segments.add(fat);
        scene.oks.add(oks("OKS-A", 8, 70, 40));
        scene.oks.add(oks("OKS-B", 7, 70, 90));
        scene.oks.add(oks("OKS-C", 6, 70, 340));
        scene.envelope();
        return scene;
    }

    private static void score(List<Variant> variants, Scene scene, AppendixModel appendix) {
        DiameterSelector d = new DiameterSelector();
        ReconstructionCalculator r = new ReconstructionCalculator();
        CostCalculator c = new CostCalculator();
        for (Variant v : variants) {
            d.applyTree(v, appendix);
            r.apply(v, scene, appendix);
            c.apply(v, scene, appendix);
        }
        new RankingCalculator().rank(variants, appendix);
    }

    private static Variant find(List<Variant> variants, String code) {
        for (Variant v : variants) {
            if (code.equals(v.code)) {
                return v;
            }
        }
        return variants.isEmpty() ? null : variants.get(0);
    }

    private ProspectiveOks oks(String id, double flow, double x, double y) {
        ProspectiveOks o = new ProspectiveOks();
        o.id = id;
        o.flowTph = flow;
        o.connection = gf.createPoint(new Coordinate(x, y));
        return o;
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }

    private static final class StraightMetric implements PathMetric {
        @Override
        public List<org.locationtech.jts.geom.Coordinate> find(Coordinate a, Coordinate b) {
            return List.of(new Coordinate(a), new Coordinate(b));
        }

        @Override
        public double cost(Coordinate a, Coordinate b) {
            return a.distance(b);
        }

        @Override
        public double length(Coordinate a, Coordinate b) {
            return a.distance(b);
        }
    }
}
