package ru.lct.heatnet.engine;

import org.locationtech.jts.geom.Point;

public class ReconstructionChamber {
    public String id;
    public Point geometryMeters;
    public int existingDn;
    public int requiredDn;
    public double cost;
}
