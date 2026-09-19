package ru.lct.heatnet.api;

import java.util.List;
import java.util.UUID;
import org.springframework.http.MediaType;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;
import ru.lct.heatnet.api.dto.DatasetResponse;
import ru.lct.heatnet.api.dto.FeaturePageResponse;
import ru.lct.heatnet.ingest.FeatureKind;
import ru.lct.heatnet.service.DatasetService;

@RestController
@RequestMapping("/api/v1/datasets")
public class DatasetController {

    private final DatasetService datasets;

    public DatasetController(DatasetService datasets) {
        this.datasets = datasets;
    }

    @GetMapping
    public List<DatasetResponse> list() {
        return datasets.list();
    }

    @GetMapping("/{id}")
    public DatasetResponse get(@PathVariable UUID id) {
        return datasets.get(id);
    }

    @PostMapping(consumes = MediaType.MULTIPART_FORM_DATA_VALUE)
    public DatasetResponse upload(@RequestParam("file") MultipartFile file) {
        return datasets.upload(file);
    }

    @GetMapping("/{id}/features")
    public FeaturePageResponse features(@PathVariable UUID id,
                                        @RequestParam(required = false) FeatureKind kind,
                                        @RequestParam(defaultValue = "2000") int limit,
                                        @RequestParam(defaultValue = "0") int offset) {
        return datasets.features(id, kind, limit, offset);
    }

    @GetMapping(value = "/{id}/preview.geojson", produces = "application/geo+json")
    public String preview(@PathVariable UUID id) {
        return datasets.previewGeoJson(id);
    }
}
