package ru.lct.heatnet.engine;

import org.locationtech.jts.geom.LineString;

public class NewSegment {
    public String id;
    public LineString geometryMeters;
    public double lengthM;
    public double flowTph;
    public int dn;
    public String layingMethod = "base";
    public String fromId;
    public String toId;
    public String parentId;
    public double cost;
    /** Устаревшая одна отметка. В режиме глубины используются {@link #depthFrom} и {@link #depthTo}. */
    public Double depthM;
    /** Глубина до верха габарита у fromId, первая вершина геометрии, м. В плане null. */
    public Double depthFrom;
    /** Глубина до верха габарита у toId, последняя вершина геометрии, м. В плане null. */
    public Double depthTo;
    public String specialReason;
    public double kSpec = 1.0;
}
