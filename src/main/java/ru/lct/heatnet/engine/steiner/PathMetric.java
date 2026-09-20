package ru.lct.heatnet.engine.steiner;

import java.util.List;
import org.locationtech.jts.geom.Coordinate;

/**
 * Кратчайший путь и его вес на графе достижимости (видимый граф + прямоугольные ходы).
 */
public interface PathMetric {

    List<Coordinate> find(Coordinate a, Coordinate b);

    double cost(Coordinate a, Coordinate b);

    double length(Coordinate a, Coordinate b);
}
