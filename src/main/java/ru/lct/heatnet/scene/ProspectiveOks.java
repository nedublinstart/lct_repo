package ru.lct.heatnet.scene;

import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Point;

public class ProspectiveOks {
    public String id;
    public Geometry footprint;
    public Point connection;
    public double flowTph;
    public Double heatLoad;
    public String name;
}
