package ru.lct.heatnet.engine;

import org.locationtech.jts.geom.LineString;

public class ReconstructionSegment {
    public String id;
    public String existingObjectId;
    public LineString geometryMeters;
    public int existingDn;
    public int requiredDn;
    public double extraFlowTph;
    public double lengthM;
    public double cost;
    public double existingFlowTph;
    public double calculatedFlowTph;
}
