package ru.lct.heatnet.service;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.Executor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.data.domain.PageRequest;
import org.springframework.http.HttpStatus;
import org.springframework.stereotype.Service;
import org.springframework.transaction.support.TransactionTemplate;
import org.springframework.web.multipart.MultipartFile;
import org.springframework.web.server.ResponseStatusException;
import ru.lct.heatnet.api.UserFacing;
import ru.lct.heatnet.api.dto.DatasetResponse;
import ru.lct.heatnet.api.dto.FeaturePageResponse;
import ru.lct.heatnet.api.dto.FeatureView;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.ingest.FeatureKind;
import ru.lct.heatnet.ingest.GeoJsonStreamingIngestor;
import ru.lct.heatnet.persist.Dataset;
import ru.lct.heatnet.persist.DatasetStatus;
import ru.lct.heatnet.persist.IngestedFeature;
import ru.lct.heatnet.repo.DatasetRepository;
import ru.lct.heatnet.repo.IngestedFeatureRepository;

@Service
public class DatasetService {

    private static final Logger log = LoggerFactory.getLogger(DatasetService.class);
    private final DatasetRepository datasets;
    private final IngestedFeatureRepository features;
    private final FileStorageService storage;
    private final GeoJsonStreamingIngestor ingestor;
    private final AppendixLoader appendixLoader;
    private final HeatnetProperties properties;
    private final TransactionTemplate tx;
    private final Executor executor;
    private final ObjectMapper mapper = GeoJsonGeometries.mapper();

    public DatasetService(DatasetRepository datasets,
                          IngestedFeatureRepository features,
                          FileStorageService storage,
                          GeoJsonStreamingIngestor ingestor,
                          AppendixLoader appendixLoader,
                          HeatnetProperties properties,
                          TransactionTemplate tx,
                          @Qualifier("heatnetExecutor") Executor executor) {
        this.datasets = datasets;
        this.features = features;
        this.storage = storage;
        this.ingestor = ingestor;
        this.appendixLoader = appendixLoader;
        this.properties = properties;
        this.tx = tx;
        this.executor = executor;
    }

    public List<DatasetResponse> list() {
        List<DatasetResponse> out = new ArrayList<>();
        for (Dataset d : datasets.findAllByOrderByCreatedAtDesc()) {
            out.add(toDto(d));
        }
        return out;
    }

    public DatasetResponse get(UUID id) {
        return toDto(load(id));
    }

    public DatasetResponse upload(MultipartFile file) {
        if (file == null || file.isEmpty()) {
            throw new ResponseStatusException(HttpStatus.BAD_REQUEST, "Файл не передан");
        }
        if (file.getSize() > properties.getMaxUploadBytes()) {
            throw new ResponseStatusException(HttpStatus.PAYLOAD_TOO_LARGE, "Файл больше 3 ГБ");
        }
        Dataset dataset = new Dataset();
        dataset.setId(UUID.randomUUID());
        dataset.setOriginalFilename(file.getOriginalFilename());
        dataset.setSizeBytes(file.getSize());
        dataset.setStatus(DatasetStatus.UPLOADED);
        Path stored = storage.saveUpload(dataset.getId(), file);
        dataset.setStoredPath(stored.toString());
        datasets.save(dataset);
        runParse(dataset.getId());
        return toDto(dataset);
    }

    public DatasetResponse importClasspathSample() {
        try (InputStream in = getClass().getResourceAsStream("/samples/mini-input.geojson")) {
            if (in == null) {
                throw new ResponseStatusException(HttpStatus.INTERNAL_SERVER_ERROR, "Встроенный мини-набор не найден");
            }
            return importStream("mini-input.geojson", in);
        } catch (ResponseStatusException e) {
            throw e;
        } catch (Exception e) {
            throw new ResponseStatusException(HttpStatus.INTERNAL_SERVER_ERROR, "Не удалось загрузить демо-набор: " + e.getMessage());
        }
    }

    public DatasetResponse importContestSample() {
        Path path = findContestGeoJson();
        if (path == null) {
            throw new ResponseStatusException(HttpStatus.NOT_FOUND,
                    "Конкурсный GeoJSON не найден (ожидался !!!_Датасет.geojson в корне или samples/contest-input.geojson)");
        }
        try (InputStream in = Files.newInputStream(path)) {
            return importStream(path.getFileName().toString(), in);
        } catch (ResponseStatusException e) {
            throw e;
        } catch (Exception e) {
            throw new ResponseStatusException(HttpStatus.INTERNAL_SERVER_ERROR, "Не удалось загрузить конкурсный набор: " + e.getMessage());
        }
    }

    public static Path findContestGeoJson() {
        String[] names = {
                "!!!_Датасет.geojson",
                "samples/contest-input.geojson",
                "samples/!!!_Датасет.geojson",
                "/app/samples/contest-input.geojson",
                "/app/!!!_Датасет.geojson"
        };
        for (String name : names) {
            Path p = Path.of(name);
            if (Files.isRegularFile(p)) {
                return p.toAbsolutePath().normalize();
            }
        }
        Path cwd = Path.of(".").toAbsolutePath().normalize();
        try (java.nio.file.DirectoryStream<Path> stream = Files.newDirectoryStream(cwd, "*.geojson")) {
            for (Path p : stream) {
                String n = p.getFileName().toString();
                if (n.contains("result") || n.contains("mini")) {
                    continue;
                }
                if (n.contains("Датасет") || n.toLowerCase().contains("dataset") || n.toLowerCase().contains("contest")) {
                    return p.toAbsolutePath().normalize();
                }
            }
        } catch (Exception ignored) {
            // fall through
        }
        return null;
    }

    private DatasetResponse importStream(String filename, InputStream in) throws Exception {
        Dataset dataset = new Dataset();
        dataset.setId(UUID.randomUUID());
        dataset.setOriginalFilename(filename);
        Path stored = storage.saveUpload(dataset.getId(), in, ".geojson");
        dataset.setStoredPath(stored.toString());
        dataset.setSizeBytes(Files.size(stored));
        dataset.setStatus(DatasetStatus.UPLOADED);
        datasets.save(dataset);
        parseNow(dataset.getId());
        return toDto(load(dataset.getId()));
    }

    public FeaturePageResponse features(UUID datasetId, FeatureKind kind, int limit, int offset) {
        load(datasetId);
        int safeLimit = Math.max(limit, 1);
        int safeOffset = Math.max(offset, 0);
        List<IngestedFeature> rows;
        long total;
        if (kind != null) {
            List<IngestedFeature> matched = features.findByDatasetIdAndKind(datasetId, kind);
            total = matched.size();
            int from = Math.min(safeOffset, matched.size());
            int to = Math.min(from + safeLimit, matched.size());
            rows = matched.subList(from, to);
        } else {
            total = features.countByDatasetId(datasetId);
            if (safeOffset == 0 && safeLimit >= total) {
                rows = features.findByDatasetId(datasetId);
            } else {
                rows = features.findByDatasetId(datasetId, PageRequest.of(safeOffset / safeLimit, safeLimit));
            }
        }
        FeaturePageResponse page = new FeaturePageResponse();
        page.total = total;
        page.items = new ArrayList<>();
        for (IngestedFeature f : rows) {
            page.items.add(toView(f));
        }
        return page;
    }

    public String previewGeoJson(UUID datasetId) {
        load(datasetId);
        List<IngestedFeature> rows = features.findByDatasetId(datasetId);
        StringBuilder sb = new StringBuilder();
        sb.append("{\"type\":\"FeatureCollection\",\"features\":[");
        boolean first = true;
        for (IngestedFeature f : rows) {
            if (f.getGeometryJson() == null) {
                continue;
            }
            if (!first) {
                sb.append(',');
            }
            first = false;
            sb.append("{\"type\":\"Feature\",\"id\":")
                    .append(quote(f.getExternalId()))
                    .append(",\"properties\":")
                    .append(f.getPropertiesJson() == null ? "{}" : f.getPropertiesJson())
                    .append(",\"geometry\":")
                    .append(f.getGeometryJson())
                    .append('}');
        }
        sb.append("]}");
        return sb.toString();
    }

    public Dataset load(UUID id) {
        return datasets.findById(id)
                .orElseThrow(() -> new ResponseStatusException(HttpStatus.NOT_FOUND, "Набор не найден"));
    }

    private void runParse(UUID id) {
        if (properties.isAsync()) {
            executor.execute(() -> {
                try {
                    parseNow(id);
                } catch (Exception e) {
                    log.error("Ошибка разбора {}", id, e);
                }
            });
        } else {
            parseNow(id);
        }
    }

    public void parseNow(UUID id) {
        tx.executeWithoutResult(status -> doParse(id));
    }

    private void doParse(UUID id) {
        Dataset dataset = datasets.findById(id).orElse(null);
        if (dataset == null) {
            return;
        }
        dataset.setStatus(DatasetStatus.PARSING);
        dataset.setMessage("Потоковый разбор GeoJSON");
        datasets.save(dataset);
        try {
            GeoJsonStreamingIngestor.ParseStats stats = ingestor.parse(
                    Path.of(dataset.getStoredPath()),
                    id,
                    appendixLoader.load(),
                    batch -> features.saveAll(batch));
            dataset.setFeatureCount(stats.total);
            Map<String, Integer> counts = new LinkedHashMap<>();
            stats.byKind.forEach((k, v) -> counts.put(k.name(), v));
            dataset.setKindCountsJson(mapper.writeValueAsString(counts));
            dataset.setStatus(DatasetStatus.PARSED);
            dataset.setMessage("Разобрано объектов: " + stats.total
                    + (stats.unknown > 0 ? (". Неизвестных: " + stats.unknown) : ""));
            datasets.save(dataset);
        } catch (Exception e) {
            dataset.setStatus(DatasetStatus.FAILED);
            dataset.setMessage(UserFacing.cyrillicOr(e.getMessage(), "Файл не разобран"));
            datasets.save(dataset);
            throw new IllegalStateException(e);
        }
    }

    private DatasetResponse toDto(Dataset d) {
        DatasetResponse r = new DatasetResponse();
        r.id = d.getId();
        r.originalFilename = d.getOriginalFilename();
        r.status = d.getStatus();
        r.sizeBytes = d.getSizeBytes();
        r.featureCount = d.getFeatureCount();
        r.message = d.getMessage();
        r.createdAt = d.getCreatedAt();
        r.updatedAt = d.getUpdatedAt();
        if (d.getKindCountsJson() != null) {
            try {
                r.kindCounts = mapper.readValue(d.getKindCountsJson(), new TypeReference<>() {
                });
            } catch (Exception e) {
                r.kindCounts = Map.of();
            }
        }
        return r;
    }

    private FeatureView toView(IngestedFeature f) {
        FeatureView v = new FeatureView();
        v.id = f.getExternalId();
        v.kind = f.getKind();
        try {
            v.properties = mapper.readValue(f.getPropertiesJson() == null ? "{}" : f.getPropertiesJson(), new TypeReference<>() {
            });
            v.geometry = f.getGeometryJson() == null ? null : mapper.readValue(f.getGeometryJson(), Object.class);
        } catch (Exception e) {
            v.properties = Map.of();
        }
        return v;
    }

    private static String quote(String s) {
        if (s == null) {
            return "null";
        }
        return "\"" + s.replace("\\", "\\\\").replace("\"", "\\\"") + "\"";
    }
}
