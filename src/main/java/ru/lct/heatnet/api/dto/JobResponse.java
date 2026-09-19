package ru.lct.heatnet.api.dto;

import java.time.Instant;
import java.util.UUID;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.persist.JobStatus;

public class JobResponse {
    public UUID id;
    public UUID datasetId;
    public CalculationMode mode;
    public JobStatus status;
    public int progress;
    public String message;
    public String error;
    public Instant createdAt;
    public Instant startedAt;
    public Instant finishedAt;
}
