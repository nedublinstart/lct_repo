package ru.lct.heatnet.engine.steiner;

import java.util.List;
import org.locationtech.jts.geom.Coordinate;

/**
 * Кратчайший путь на каркасе улиц: рельсы параллельно и перпендикулярно осям дорог.
 */
public interface PathMetric {

    List<Coordinate> find(Coordinate a, Coordinate b);

    double cost(Coordinate a, Coordinate b);

    double length(Coordinate a, Coordinate b);
}
