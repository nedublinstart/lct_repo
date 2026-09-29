package ru.lct.heatnet.engine.flow;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.costing.CostCalculator;
import ru.lct.heatnet.costing.DiameterSelector;
import ru.lct.heatnet.costing.RankingCalculator;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.ingest.GeoJsonStreamingIngestor;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.persist.IngestedFeature;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SceneAssembler;
import ru.lct.heatnet.service.DatasetService;

/**
 * Конкурсный набор на новом движке: все ОКС доходят до сети, смета совпадает с суммой частей,
 * ось не заходит в чужие корпуса, ввод ИТП идёт прямой на рассчитанный выход.
 */
class FlowRoutingEngineTest {

    @Test
    void contestMinCostRespectsClearanceAndConnectsAll() throws Exception {
        Path file = DatasetService.findContestGeoJson();
        assumeTrue(file != null && Files.isRegularFile(file), "конкурсный GeoJSON лежит в корне ветки");
        AppendixModel appendix = appendix();
        List<IngestedFeature> features = new ArrayList<>();
        new GeoJsonStreamingIngestor().parse(file, UUID.randomUUID(), appendix, features::addAll);
        Scene scene = new SceneAssembler().assemble(features, appendix);

        List<Variant> variants = new ru.lct.heatnet.engine.flow.FlowRoutingEngine(3_000L)
                .route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
                }, List.of("mincost"));
        assertThat(variants.size()).isBetween(1, 2);
        for (Variant v : variants) {
            new DiameterSelector().applyTree(v, appendix);
        }
        for (Variant v : variants) {
            new CostCalculator().apply(v, scene, appendix);
        }
        new RankingCalculator().rank(variants, appendix);

        for (Variant v : variants) {
            assertThat(v.unconnectedOks).isEmpty();
            assertThat(v.segments).isNotEmpty();
            assertThat(v.chambers.size() + v.taps.size()).isGreaterThan(0);
            assertThat(v.newLengthM).isBetween(1_600.0, 4_000.0);
            double pipes = v.costBreakdown.get("pipe_cost");
            double chambers = v.costBreakdown.get("chamber_construction_cost");
            double ties = v.costBreakdown.get("existing_chamber_tie_in_cost");
            double penalty = v.costBreakdown.get("unconnected_penalty");
            assertThat(v.costBreakdown.get("construction_cost")).isEqualTo(pipes + chambers + ties);
            assertThat(v.costBreakdown.get("reconstruction_cost")).isEqualTo(0.0);
            assertThat(v.totalCost).isEqualTo(pipes + chambers + ties + penalty);
            assertThat(v.totalCost).isGreaterThan(180_000_000);
            assertThat(v.score).isCloseTo(0.7 * v.totalCost / 25_000_000.0 + 0.3 * v.newLengthM / 100.0, within(0.01));
        }
        Prices prices = new Prices(appendix);
        double flow = 0;
        for (ProspectiveOks o : scene.oks) {
            if (o.connection != null) {
                flow += o.flowTph;
            }
        }
        Envelope roi = new Envelope(scene.envelope());
        roi.expandBy(100);
        FreeSpace space = new FreeSpace(scene, appendix, prices, roi, prices.dnFor(flow));
        Ports ports = new Ports(scene, space);
        Map<String, Ports.Terminal> byId = new HashMap<>();
        for (Ports.Terminal t : ports.terms) {
            byId.put(t.id, t);
        }
        for (Variant variant : variants) {
            for (NewSegment s : variant.segments) {
                assertThat(s.layingMethod).isEqualTo(s.kSpec > 1 + 1e-9 ? "special" : "base");
                Coordinate[] pts = s.geometryMeters.getCoordinates();
                Ports.Terminal term = byId.get(s.fromId);
                for (int i = 1; i < pts.length; i++) {
                    int skip = term != null && i == 1 ? term.host : -1;
                    double cut = space.violation(pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y, skip);
                    assertThat(cut)
                            .as("%s %s→%s ребро %d заходит в запрет на %.2f м", s.id, s.fromId, s.toId, i, cut)
                            .isLessThan(0.05);
                }
                if (term != null && pts.length >= 2) {
                    assertThat(exitsThroughPort(ports, term, pts[0], pts[1]))
                            .as("ввод %s идёт прямой на выход ИТП", s.fromId)
                            .isTrue();
                }
            }
        }
    }

    /** Прямая первого ребра проходит через один из выходов ИТП на фасад. */
    private static boolean exitsThroughPort(Ports ports, Ports.Terminal term, Coordinate a, Coordinate b) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double len = Math.hypot(dx, dy);
        if (len < 1e-6) {
            return false;
        }
        for (int i = 0; i < term.count; i++) {
            int p = term.first + i;
            double px = ports.px[p];
            double py = ports.py[p];
            double cross = Math.abs(dx * (py - a.y) - dy * (px - a.x)) / len;
            double along = (dx * (px - a.x) + dy * (py - a.y)) / len;
            if (cross < 0.6 && along > -0.5) {
                return true;
            }
        }
        return false;
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
