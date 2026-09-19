package ru.lct.heatnet.api;

import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import ru.lct.heatnet.api.dto.CreateJobRequest;
import ru.lct.heatnet.api.dto.DatasetResponse;
import ru.lct.heatnet.api.dto.JobResponse;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.service.DatasetService;
import ru.lct.heatnet.service.JobService;

@RestController
@RequestMapping("/api/v1/demo")
public class DemoController {

    private final DatasetService datasets;
    private final JobService jobs;

    public DemoController(DatasetService datasets, JobService jobs) {
        this.datasets = datasets;
        this.jobs = jobs;
    }

    @PostMapping("/run")
    public JobResponse run(@RequestParam(defaultValue = "PLAN_2D") CalculationMode mode) {
        DatasetResponse dataset = datasets.importClasspathSample();
        CreateJobRequest req = new CreateJobRequest();
        req.datasetId = dataset.id;
        req.mode = mode;
        return jobs.create(req);
    }

    @PostMapping("/contest")
    public JobResponse contest(@RequestParam(defaultValue = "PLAN_2D") CalculationMode mode) {
        DatasetResponse dataset = datasets.importContestSample();
        CreateJobRequest req = new CreateJobRequest();
        req.datasetId = dataset.id;
        req.mode = mode;
        return jobs.create(req);
    }
}
