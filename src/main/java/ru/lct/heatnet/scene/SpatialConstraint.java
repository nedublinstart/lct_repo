package ru.lct.heatnet.scene;

import org.locationtech.jts.geom.Geometry;
import ru.lct.heatnet.appendix.AppendixModel;

public class SpatialConstraint {
    public String id;
    public String type;
    public Geometry geometry;
    public AppendixModel.ConstraintSpec rule;
}
