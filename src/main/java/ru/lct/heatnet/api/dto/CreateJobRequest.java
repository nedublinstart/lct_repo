package ru.lct.heatnet.api.dto;

import ru.lct.heatnet.persist.CalculationMode;

public class CreateJobRequest {
    public java.util.UUID datasetId;
    public CalculationMode mode = CalculationMode.PLAN_2D;
}
