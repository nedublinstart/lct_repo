package ru.lct.heatnet.api.dto;

import java.util.Map;
import ru.lct.heatnet.ingest.FeatureKind;

public class FeatureView {
    public String id;
    public FeatureKind kind;
    public Map<String, Object> properties;
    public Object geometry;
}
