package ru.lct.heatnet.engine;

import org.locationtech.jts.geom.Point;

public class TapPoint {
    public String id;
    public Point geometryMeters;
    public String existingObjectId;
    public String existingObjectKind;
    public double extraFlowTph;
    public double cost;
}
