package ru.lct.heatnet.api;

import static org.assertj.core.api.Assertions.assertThat;

import java.time.Duration;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.web.client.TestRestTemplate;
import org.springframework.http.ResponseEntity;
import org.springframework.test.context.ActiveProfiles;
import ru.lct.heatnet.api.dto.JobResponse;
import ru.lct.heatnet.api.dto.VariantSummaryResponse;
import ru.lct.heatnet.persist.JobStatus;

@SpringBootTest(webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT)
@ActiveProfiles("test")
class DemoFlowTest {

    @Autowired
    private TestRestTemplate rest;

    @Test
    void demoRunProducesRankedGeoJsonVariants() {
        ResponseEntity<JobResponse> created = rest.postForEntity("/api/v1/demo/run", null, JobResponse.class);
        assertThat(created.getStatusCode().is2xxSuccessful()).isTrue();
        JobResponse job = created.getBody();
        assertThat(job).isNotNull();

        JobResponse done = null;
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < Duration.ofSeconds(60).toMillis()) {
            done = rest.getForObject("/api/v1/jobs/" + job.id, JobResponse.class);
            if (done != null && (done.status == JobStatus.COMPLETED || done.status == JobStatus.FAILED)) {
                break;
            }
        }
        assertThat(done).isNotNull();
        assertThat(done.status).isEqualTo(JobStatus.COMPLETED);

        VariantSummaryResponse[] variants = rest.getForObject("/api/v1/jobs/" + job.id + "/variants", VariantSummaryResponse[].class);
        assertThat(variants).isNotEmpty();
        assertThat(variants[0].rank).isEqualTo(1);
        String geo = rest.getForObject("/api/v1/jobs/" + job.id + "/variants/1/geojson", String.class);
        assertThat(geo).contains("FeatureCollection");
        assertThat(geo).contains("new_segment");
        assertThat(geo).contains("tap_point");
    }
}
