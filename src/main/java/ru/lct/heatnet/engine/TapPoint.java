package ru.lct.heatnet.engine;

import org.locationtech.jts.geom.Point;

public class TapPoint {
    public String id;
    public String nodeId;
    public Point geometryMeters;
    public String existingObjectId;
    public String existingObjectKind;
    public double extraFlowTph;
    public double cost;
    public int existingDiameter;
    public int requiredDiameter;
}
