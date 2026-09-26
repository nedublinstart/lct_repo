package ru.lct.heatnet.api.dto;

import java.util.List;
import ru.lct.heatnet.persist.CalculationMode;

public class CreateJobRequest {
    public java.util.UUID datasetId;
    public CalculationMode mode = CalculationMode.PLAN_2D;
    /** Коды режимов: mincost, mintaps, minrecon. Пусто — все три. */
    public List<String> strategies;
}
