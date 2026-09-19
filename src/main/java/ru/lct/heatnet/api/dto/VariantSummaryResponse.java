package ru.lct.heatnet.api.dto;

import java.util.List;
import java.util.Map;
import java.util.UUID;

public class VariantSummaryResponse {
    public UUID id;
    public int rank;
    public String title;
    public String code;
    public String description;
    public double cost;
    public double lengthM;
    public double reconLengthM;
    public double score;
    public int unconnectedCount;
    public List<String> unconnectedIds;
    public Map<String, Object> breakdown;
    public List<String> notes;
}
