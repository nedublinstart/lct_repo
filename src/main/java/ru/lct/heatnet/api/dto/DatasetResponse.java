package ru.lct.heatnet.api.dto;

import java.time.Instant;
import java.util.Map;
import java.util.UUID;
import ru.lct.heatnet.persist.DatasetStatus;

public class DatasetResponse {
    public UUID id;
    public String originalFilename;
    public DatasetStatus status;
    public long sizeBytes;
    public int featureCount;
    public Map<String, Integer> kindCounts;
    public String message;
    public Instant createdAt;
    public Instant updatedAt;
}
