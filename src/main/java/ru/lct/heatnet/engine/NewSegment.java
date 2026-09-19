package ru.lct.heatnet.engine;

import org.locationtech.jts.geom.LineString;

public class NewSegment {
    public String id;
    public LineString geometryMeters;
    public double lengthM;
    public double flowTph;
    public int dn;
    public String layingMethod = "trenchless";
    public String fromId;
    public String toId;
    public String parentId;
    public double cost;
    public Double depthM;
    public String specialReason;
}
