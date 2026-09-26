package ru.lct.heatnet.service;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.Executor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.http.HttpStatus;
import org.springframework.stereotype.Service;
import org.springframework.transaction.support.TransactionTemplate;
import org.springframework.web.server.ResponseStatusException;
import ru.lct.heatnet.api.dto.CreateJobRequest;
import ru.lct.heatnet.api.dto.JobResponse;
import ru.lct.heatnet.api.dto.VariantSummaryResponse;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.costing.CostCalculator;
import ru.lct.heatnet.costing.DiameterSelector;
import ru.lct.heatnet.costing.RankingCalculator;
import ru.lct.heatnet.costing.ReconstructionCalculator;
import ru.lct.heatnet.engine.RoutingEngine;
import ru.lct.heatnet.engine.steiner.Strategy;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.depth.DepthPostProcessor;
import ru.lct.heatnet.export.ResultGeoJsonExporter;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.persist.CalculationJob;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.persist.Dataset;
import ru.lct.heatnet.persist.DatasetStatus;
import ru.lct.heatnet.persist.JobStatus;
import ru.lct.heatnet.persist.VariantRecord;
import ru.lct.heatnet.repo.CalculationJobRepository;
import ru.lct.heatnet.repo.IngestedFeatureRepository;
import ru.lct.heatnet.repo.VariantRecordRepository;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SceneAssembler;

@Service
public class JobService {

    private static final Logger log = LoggerFactory.getLogger(JobService.class);
    private final CalculationJobRepository jobs;
    private final VariantRecordRepository variants;
    private final IngestedFeatureRepository features;
    private final DatasetService datasets;
    private final AppendixLoader appendixLoader;
    private final SceneAssembler assembler;
    private final RoutingEngine routingEngine;
    private final DepthPostProcessor depthPostProcessor;
    private final ResultGeoJsonExporter exporter;
    private final FileStorageService storage;
    private final HeatnetProperties properties;
    private final TransactionTemplate tx;
    private final Executor executor;
    private final ObjectMapper mapper = GeoJsonGeometries.mapper();
    private final DiameterSelector diameterSelector = new DiameterSelector();
    private final ReconstructionCalculator reconstructionCalculator = new ReconstructionCalculator();
    private final CostCalculator costCalculator = new CostCalculator();
    private final RankingCalculator rankingCalculator = new RankingCalculator();

    public JobService(CalculationJobRepository jobs,
                      VariantRecordRepository variants,
                      IngestedFeatureRepository features,
                      DatasetService datasets,
                      AppendixLoader appendixLoader,
                      SceneAssembler assembler,
                      RoutingEngine routingEngine,
                      DepthPostProcessor depthPostProcessor,
                      ResultGeoJsonExporter exporter,
                      FileStorageService storage,
                      HeatnetProperties properties,
                      TransactionTemplate tx,
                      @Qualifier("heatnetExecutor") Executor executor) {
        this.jobs = jobs;
        this.variants = variants;
        this.features = features;
        this.datasets = datasets;
        this.appendixLoader = appendixLoader;
        this.assembler = assembler;
        this.routingEngine = routingEngine;
        this.depthPostProcessor = depthPostProcessor;
        this.exporter = exporter;
        this.storage = storage;
        this.properties = properties;
        this.tx = tx;
        this.executor = executor;
    }

    public JobResponse create(CreateJobRequest request) {
        if (request == null || request.datasetId == null) {
            throw new ResponseStatusException(HttpStatus.BAD_REQUEST, "datasetId обязателен");
        }
        Dataset dataset = datasets.load(request.datasetId);
        if (dataset.getStatus() != DatasetStatus.PARSED) {
            throw new ResponseStatusException(HttpStatus.CONFLICT, "Набор ещё не разобран: " + dataset.getStatus());
        }
        CalculationJob job = new CalculationJob();
        job.setId(UUID.randomUUID());
        job.setDatasetId(dataset.getId());
        job.setMode(request.mode == null ? CalculationMode.PLAN_2D : request.mode);
        job.setStrategyCodes(Strategy.join(request.strategies));
        job.setStatus(JobStatus.QUEUED);
        job.setMessage("В очереди");
        job.setCreatedAt(Instant.now());
        jobs.saveAndFlush(job);
        UUID id = job.getId();
        if (properties.isAsync()) {
            executor.execute(() -> runSafe(id));
        } else {
            runSafe(id);
        }
        return toDto(job);
    }

    public JobResponse get(UUID id) {
        return toDto(load(id));
    }

    public List<VariantSummaryResponse> variantSummaries(UUID jobId) {
        load(jobId);
        List<VariantSummaryResponse> out = new ArrayList<>();
        for (VariantRecord record : variants.findByJobIdOrderByRankAsc(jobId)) {
            if (record.getRank() == 0) {
                continue;
            }
            out.add(toSummary(record));
        }
        return out;
    }

    public String variantGeoJson(UUID jobId, int rank) {
        VariantRecord record = variants.findByJobIdAndRank(jobId, rank)
                .orElseThrow(() -> new ResponseStatusException(HttpStatus.NOT_FOUND, "Вариант не найден"));
        if (record.getGeoJsonInline() != null) {
            return record.getGeoJsonInline();
        }
        if (record.getGeoJsonPath() != null) {
            return storage.readResult(Path.of(record.getGeoJsonPath()));
        }
        throw new ResponseStatusException(HttpStatus.NOT_FOUND, "GeoJSON варианта ещё не готов");
    }

    public CalculationJob load(UUID id) {
        return jobs.findById(id).orElseThrow(() -> new ResponseStatusException(HttpStatus.NOT_FOUND, "Задача не найдена"));
    }

    private void runSafe(UUID id) {
        try {
            // Не оборачиваем весь расчёт в одну транзакцию: иначе UI видит 0%/QUEUED,
            // пока JTS не закончит, а GET упирается в старый снимок строки.
            run(id);
        } catch (Exception e) {
            log.error("Расчёт {} упал", id, e);
            tx.executeWithoutResult(status -> {
                jobs.findById(id).ifPresent(job -> {
                    job.setStatus(JobStatus.FAILED);
                    job.setError(e.getMessage());
                    job.setMessage("Ошибка расчёта");
                    job.setFinishedAt(Instant.now());
                    jobs.saveAndFlush(job);
                });
            });
        }
    }

    private void persist(CalculationJob job) {
        jobs.saveAndFlush(job);
    }

    private void run(UUID id) {
        CalculationJob job = jobs.findById(id).orElseThrow();
        job.setStatus(JobStatus.RUNNING);
        job.setStartedAt(Instant.now());
        job.setProgress(5);
        job.setMessage("Собираю сцену");
        persist(job);
        log.info("Старт расчёта {} dataset={}", id, job.getDatasetId());

        AppendixModel appendix = appendixLoader.load();
        Scene scene = assembler.assemble(features.findByDatasetId(job.getDatasetId()), appendix);
        if (scene.oks.isEmpty()) {
            job.setMessage("В наборе нет перспективных ОКС");
        }
        job.setProgress(12);
        persist(job);

        List<String> codes = job.getStrategyCodes() == null || job.getStrategyCodes().isBlank()
                ? List.of()
                : Arrays.asList(job.getStrategyCodes().split(","));
        List<Variant> result = routingEngine.route(scene, appendix, job.getMode(), (pct, msg) -> {
            job.setProgress(Math.min(90, pct));
            job.setMessage(msg);
            persist(job);
        }, codes);

        for (Variant variant : result) {
            diameterSelector.applyTree(variant, appendix);
            ru.lct.heatnet.engine.steiner.SubmissionHygiene.assignTapDiameters(variant);
            reconstructionCalculator.apply(variant, scene, appendix);
            if (job.getMode() == CalculationMode.DEPTH) {
                depthPostProcessor.apply(variant, scene, appendix);
            }
            costCalculator.apply(variant, scene, appendix);
        }
        rankingCalculator.rank(result, appendix);

        String combined = exporter.exportAll(result, appendix, scene.projector);
        Path combinedPath = storage.writeResult(job.getId(), 0, combined);
        VariantRecord all = new VariantRecord();
        all.setJobId(job.getId());
        all.setRank(0);
        all.setTitle("Все варианты");
        all.setGeoJsonPath(combinedPath.toString());
        if (combined.length() < 2_000_000) {
            all.setGeoJsonInline(combined);
        }
        variants.save(all);

        int rank = 1;
        for (Variant variant : result) {
            if (rank > 3) {
                break;
            }
            String geo = exporter.export(variant, String.valueOf(rank), appendix, scene.projector);
            Path path = storage.writeResult(job.getId(), rank, geo);
            VariantRecord record = new VariantRecord();
            record.setJobId(job.getId());
            record.setRank(rank);
            record.setTitle(variant.title);
            record.setCost(variant.totalCost);
            record.setLengthM(variant.newLengthM + variant.reconLengthM);
            record.setScore(variant.score);
            record.setUnconnectedCount(variant.unconnectedOks.size());
            record.setUnconnectedIds(String.join(",", variant.unconnectedOks));
            record.setGeoJsonPath(path.toString());
            if (geo.length() < 1_500_000) {
                record.setGeoJsonInline(geo);
            }
            Map<String, Object> summary = new LinkedHashMap<>();
            summary.put("code", variant.code);
            summary.put("description", variant.description);
            summary.put("breakdown", variant.costBreakdown);
            summary.put("notes", variant.notes);
            summary.put("newLengthM", variant.newLengthM);
            summary.put("reconLengthM", variant.reconLengthM);
            try {
                record.setSummaryJson(mapper.writeValueAsString(summary));
            } catch (Exception e) {
                record.setSummaryJson("{}");
            }
            variants.save(record);
            rank++;
        }
        job.setProgress(100);
        job.setStatus(JobStatus.COMPLETED);
        job.setMessage("Готово, вариантов: " + Math.min(3, result.size()));
        job.setFinishedAt(Instant.now());
        persist(job);
        log.info("Расчёт {} готов, вариантов {}", id, Math.min(3, result.size()));
    }

    private JobResponse toDto(CalculationJob job) {
        JobResponse r = new JobResponse();
        r.id = job.getId();
        r.datasetId = job.getDatasetId();
        r.mode = job.getMode();
        r.status = job.getStatus();
        r.progress = job.getProgress();
        r.message = job.getMessage();
        r.error = job.getError();
        r.createdAt = job.getCreatedAt();
        r.startedAt = job.getStartedAt();
        r.finishedAt = job.getFinishedAt();
        return r;
    }

    private VariantSummaryResponse toSummary(VariantRecord record) {
        VariantSummaryResponse r = new VariantSummaryResponse();
        r.id = record.getId();
        r.rank = record.getRank();
        r.title = record.getTitle();
        r.cost = record.getCost();
        r.lengthM = record.getLengthM();
        r.score = record.getScore();
        r.unconnectedCount = record.getUnconnectedCount();
        r.unconnectedIds = record.getUnconnectedIds() == null || record.getUnconnectedIds().isBlank()
                ? List.of()
                : Arrays.asList(record.getUnconnectedIds().split(","));
        if (record.getSummaryJson() != null) {
            try {
                Map<String, Object> summary = mapper.readValue(record.getSummaryJson(), new TypeReference<>() {
                });
                r.code = String.valueOf(summary.getOrDefault("code", ""));
                r.description = String.valueOf(summary.getOrDefault("description", ""));
                r.breakdown = castMap(summary.get("breakdown"));
                r.notes = castList(summary.get("notes"));
                Object recon = summary.get("reconLengthM");
                if (recon instanceof Number) {
                    r.reconLengthM = ((Number) recon).doubleValue();
                }
            } catch (Exception ignored) {
                r.breakdown = Map.of();
                r.notes = List.of();
            }
        }
        return r;
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> castMap(Object o) {
        if (o instanceof Map) {
            return (Map<String, Object>) o;
        }
        return Map.of();
    }

    @SuppressWarnings("unchecked")
    private static List<String> castList(Object o) {
        if (o instanceof List) {
            return (List<String>) o;
        }
        return List.of();
    }
}
