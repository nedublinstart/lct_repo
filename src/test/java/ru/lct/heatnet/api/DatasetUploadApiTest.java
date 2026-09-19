package ru.lct.heatnet.api;

import static org.assertj.core.api.Assertions.assertThat;

import java.nio.file.Path;
import java.time.Duration;
import java.util.UUID;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.web.client.TestRestTemplate;
import org.springframework.core.io.FileSystemResource;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.util.LinkedMultiValueMap;
import org.springframework.util.MultiValueMap;
import ru.lct.heatnet.api.dto.CreateJobRequest;
import ru.lct.heatnet.api.dto.DatasetResponse;
import ru.lct.heatnet.api.dto.JobResponse;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.persist.DatasetStatus;
import ru.lct.heatnet.persist.JobStatus;

@SpringBootTest(webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT)
@ActiveProfiles("test")
class DatasetUploadApiTest {

    @Autowired
    private TestRestTemplate rest;

    @Test
    void ownGeoJsonUploadsThenJobStarts() {
        DatasetResponse uploaded = uploadMini();
        assertThat(uploaded.id).isNotNull();
        DatasetResponse parsed = waitParsed(uploaded.id);
        assertThat(parsed.status).isEqualTo(DatasetStatus.PARSED);
        assertThat(parsed.featureCount).isEqualTo(13);
        assertThat(parsed.originalFilename).contains("mini-input");

        CreateJobRequest req = new CreateJobRequest();
        req.datasetId = parsed.id;
        req.mode = CalculationMode.PLAN_2D;
        ResponseEntity<JobResponse> created = rest.postForEntity("/api/v1/jobs", req, JobResponse.class);
        assertThat(created.getStatusCode().is2xxSuccessful()).isTrue();
        JobResponse job = created.getBody();
        assertThat(job).isNotNull();
        assertThat(job.datasetId).isEqualTo(parsed.id);

        JobResponse done = waitJob(job.id);
        assertThat(done.status).isEqualTo(JobStatus.COMPLETED);
    }

    @Test
    void jobWithoutDatasetIsRejected() {
        CreateJobRequest req = new CreateJobRequest();
        ResponseEntity<String> res = rest.postForEntity("/api/v1/jobs", req, String.class);
        assertThat(res.getStatusCode()).isEqualTo(HttpStatus.BAD_REQUEST);
    }

    private DatasetResponse uploadMini() {
        MultiValueMap<String, Object> body = new LinkedMultiValueMap<>();
        body.add("file", new FileSystemResource(Path.of("samples/mini-input.geojson")));
        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.MULTIPART_FORM_DATA);
        ResponseEntity<DatasetResponse> res = rest.postForEntity(
                "/api/v1/datasets", new HttpEntity<>(body, headers), DatasetResponse.class);
        assertThat(res.getStatusCode().is2xxSuccessful()).isTrue();
        return res.getBody();
    }

    private DatasetResponse waitParsed(UUID id) {
        DatasetResponse d = null;
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < Duration.ofSeconds(30).toMillis()) {
            d = rest.getForObject("/api/v1/datasets/" + id, DatasetResponse.class);
            if (d != null && (d.status == DatasetStatus.PARSED || d.status == DatasetStatus.FAILED)) {
                return d;
            }
        }
        return d;
    }

    private JobResponse waitJob(UUID id) {
        JobResponse done = null;
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < Duration.ofSeconds(60).toMillis()) {
            done = rest.getForObject("/api/v1/jobs/" + id, JobResponse.class);
            if (done != null && (done.status == JobStatus.COMPLETED || done.status == JobStatus.FAILED)) {
                return done;
            }
        }
        return done;
    }
}
