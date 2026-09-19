package ru.lct.heatnet.cli;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.costing.CostCalculator;
import ru.lct.heatnet.costing.DiameterSelector;
import ru.lct.heatnet.costing.RankingCalculator;
import ru.lct.heatnet.costing.ReconstructionCalculator;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.SmartRoutingEngine;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.export.ResultGeoJsonExporter;
import ru.lct.heatnet.ingest.GeoJsonStreamingIngestor;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.persist.IngestedFeature;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SceneAssembler;
import ru.lct.heatnet.service.DatasetService;

/**
 * Полный расчёт без веб-слоя: входной GeoJSON → до трёх вариантов → combined FeatureCollection.
 */
public final class OfflineProcessor {

    private OfflineProcessor() {
    }

    public static void main(String[] args) throws Exception {
        Path input = null;
        Path output = Path.of("samples/contest-result.geojson");
        for (int i = 0; i < args.length; i++) {
            if ("--in".equals(args[i]) && i + 1 < args.length) {
                input = Path.of(args[++i]);
            } else if ("--out".equals(args[i]) && i + 1 < args.length) {
                output = Path.of(args[++i]);
            } else if (!args[i].startsWith("-") && input == null) {
                input = Path.of(args[i]);
            }
        }
        if (input == null) {
            input = DatasetService.findContestGeoJson();
        }
        if (input == null || !Files.isRegularFile(input)) {
            throw new IllegalStateException("Не найден конкурсный GeoJSON");
        }
        Result result = process(input, output);
        System.out.println("Вход: " + input.toAbsolutePath());
        System.out.println("Выход: " + output.toAbsolutePath() + " (" + result.bytes + " байт)");
        int rank = 1;
        for (Variant v : result.variants) {
            System.out.printf("  #%d %s  cost=%.0f  L=%.1f  score=%.3f  unconnected=%d%n",
                    rank++, v.title, v.totalCost, v.newLengthM + v.reconLengthM, v.score, v.unconnectedOks.size());
        }
    }

    public static Result process(Path input, Path output) throws Exception {
        HeatnetProperties props = new HeatnetProperties();
        AppendixModel appendix = new AppendixLoader(props).load();
        List<IngestedFeature> features = new ArrayList<>();
        new GeoJsonStreamingIngestor().parse(input, UUID.randomUUID(), appendix, features::addAll);
        Scene scene = new SceneAssembler().assemble(features, appendix);
        System.out.printf("Сцена: ОКС=%d сеть=%d камеры=%d ограничения=%d%n",
                scene.oks.size(), scene.segments.size(), scene.chambers.size(), scene.constraints.size());
        List<Variant> variants = new SmartRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (pct, msg) ->
                System.out.println(pct + "% " + msg));
        DiameterSelector diameters = new DiameterSelector();
        ReconstructionCalculator reconstruction = new ReconstructionCalculator();
        CostCalculator cost = new CostCalculator();
        for (Variant variant : variants) {
            diameters.apply(variant.segments, appendix);
            for (NewChamber ch : variant.chambers) {
                int max = ch.dn;
                for (NewSegment seg : variant.segments) {
                    if (ch.id.equals(seg.fromId) || ch.id.equals(seg.toId)) {
                        max = Math.max(max, seg.dn);
                    }
                }
                ch.dn = max;
            }
            for (TapPoint tap : variant.taps) {
                int req = 0;
                for (NewSegment seg : variant.segments) {
                    if (seg.toId != null && (seg.toId.equals(tap.nodeId) || seg.toId.equals(tap.id)
                            || seg.toId.equals(tap.existingObjectId))) {
                        req = Math.max(req, seg.dn);
                    }
                }
                if (req == 0) {
                    for (NewSegment seg : variant.segments) {
                        req = Math.max(req, seg.dn);
                    }
                }
                tap.requiredDiameter = req;
            }
            reconstruction.apply(variant, scene, appendix);
            cost.apply(variant, scene, appendix);
        }
        new RankingCalculator().rank(variants, appendix);
        String json = new ResultGeoJsonExporter().exportAll(variants, appendix, scene.projector);
        if (output != null) {
            Path parent = output.toAbsolutePath().getParent();
            if (parent != null) {
                Files.createDirectories(parent);
            }
            Files.writeString(output, json, StandardCharsets.UTF_8);
        }
        Result result = new Result();
        result.variants = variants;
        result.geoJson = json;
        result.bytes = json.getBytes(StandardCharsets.UTF_8).length;
        result.oks = scene.oks.size();
        return result;
    }

    public static final class Result {
        public List<Variant> variants;
        public String geoJson;
        public int bytes;
        public int oks;
    }
}
