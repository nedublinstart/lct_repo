package ru.lct.heatnet.api;

import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.UUID;
import org.springframework.http.ContentDisposition;
import org.springframework.http.HttpHeaders;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;
import ru.lct.heatnet.api.dto.CreateJobRequest;
import ru.lct.heatnet.api.dto.JobResponse;
import ru.lct.heatnet.api.dto.VariantSummaryResponse;
import ru.lct.heatnet.service.JobService;

@RestController
@RequestMapping("/api/v1/jobs")
public class JobController {

    private final JobService jobs;

    public JobController(JobService jobs) {
        this.jobs = jobs;
    }

    @PostMapping
    public JobResponse create(@RequestBody CreateJobRequest request) {
        return jobs.create(request);
    }

    @GetMapping("/{id}")
    public JobResponse get(@PathVariable UUID id) {
        return jobs.get(id);
    }

    @GetMapping("/{id}/variants")
    public List<VariantSummaryResponse> variants(@PathVariable UUID id) {
        return jobs.variantSummaries(id);
    }

    @GetMapping("/{id}/variants/{rank}")
    public VariantSummaryResponse variant(@PathVariable UUID id, @PathVariable int rank) {
        return jobs.variantSummaries(id).stream()
                .filter(v -> v.rank == rank)
                .findFirst()
                .orElseThrow();
    }

    @GetMapping("/{id}/variants/{rank}/geojson")
    public ResponseEntity<byte[]> geojson(@PathVariable UUID id, @PathVariable int rank) {
        String json = jobs.variantGeoJson(id, rank);
        byte[] body = json.getBytes(StandardCharsets.UTF_8);
        ContentDisposition cd = ContentDisposition.attachment()
                .filename("heatnet-" + id + "-v" + rank + ".geojson")
                .build();
        return ResponseEntity.ok()
                .header(HttpHeaders.CONTENT_DISPOSITION, cd.toString())
                .contentType(MediaType.parseMediaType("application/geo+json"))
                .contentLength(body.length)
                .body(body);
    }

    @GetMapping("/{id}/result.geojson")
    public ResponseEntity<byte[]> combined(@PathVariable UUID id) {
        String json = jobs.variantGeoJson(id, 0);
        byte[] body = json.getBytes(StandardCharsets.UTF_8);
        ContentDisposition cd = ContentDisposition.attachment()
                .filename("heatnet-" + id + "-result.geojson")
                .build();
        return ResponseEntity.ok()
                .header(HttpHeaders.CONTENT_DISPOSITION, cd.toString())
                .contentType(MediaType.parseMediaType("application/geo+json"))
                .contentLength(body.length)
                .body(body);
    }
}
