package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.PathSmoother;
import ru.lct.heatnet.engine.greedy.StreetFrame;

/**
 * Кэш кратчайших путей по уличному каркасу (Дейкстра на рельсах ∥/⊥ осям дорог).
 */
public final class MetricPathCache implements PathMetric {

    private final StreetFrame frame;
    private final ObstacleIndex obstacles;
    private final Map<String, Cached> cache = new HashMap<>();

    public MetricPathCache(StreetFrame frame, ObstacleIndex obstacles) {
        this.frame = frame;
        this.obstacles = obstacles;
    }

    @Override
    public List<Coordinate> find(Coordinate a, Coordinate b) {
        return lookup(a, b).path;
    }

    @Override
    public double cost(Coordinate a, Coordinate b) {
        return lookup(a, b).cost;
    }

    @Override
    public double length(Coordinate a, Coordinate b) {
        return lookup(a, b).length;
    }

    public double pathCost(List<Coordinate> path) {
        return costOf(path, obstacles);
    }

    public double pathLength(List<Coordinate> path) {
        return lengthOf(path);
    }

    private Cached lookup(Coordinate a, Coordinate b) {
        if (a == null || b == null) {
            return Cached.NONE;
        }
        if (a.distance(b) < 0.05) {
            List<Coordinate> trivial = new ArrayList<>(2);
            trivial.add(new Coordinate(a));
            trivial.add(new Coordinate(b));
            return new Cached(trivial, 0, 0);
        }
        String k = key(a) + ">" + key(b);
        Cached hit = cache.get(k);
        if (hit != null) {
            return hit;
        }
        String kr = key(b) + ">" + key(a);
        Cached rev = cache.get(kr);
        if (rev != null) {
            Cached mirrored = rev.reverse();
            cache.put(k, mirrored);
            return mirrored;
        }
        Cached computed = compute(a, b);
        cache.put(k, computed);
        return computed;
    }

    private Cached compute(Coordinate a, Coordinate b) {
        List<Coordinate> path = frame.find(a, b);
        if (path == null || path.size() < 2) {
            return Cached.NONE;
        }
        path = PathSmoother.collapseColinear(path, obstacles);
        if (path == null || path.size() < 2 || obstacles.pathHitsAvoid(path, 1)) {
            return Cached.NONE;
        }
        double cost = costOf(path, obstacles);
        if (!Double.isFinite(cost)) {
            cost = lengthOf(path);
        }
        if (!Double.isFinite(cost) || cost <= 0) {
            return Cached.NONE;
        }
        return new Cached(path, cost, lengthOf(path));
    }

    static double costOf(List<Coordinate> path, ObstacleIndex obstacles) {
        if (path == null || path.size() < 2) {
            return path == null ? Double.POSITIVE_INFINITY : 0;
        }
        double s = 0;
        for (int i = 1; i < path.size(); i++) {
            double w = obstacles.travelCost(path.get(i - 1), path.get(i));
            if (!Double.isFinite(w)) {
                return Double.POSITIVE_INFINITY;
            }
            s += w;
        }
        return s;
    }

    static double lengthOf(List<Coordinate> path) {
        if (path == null || path.size() < 2) {
            return 0;
        }
        double s = 0;
        for (int i = 1; i < path.size(); i++) {
            s += path.get(i - 1).distance(path.get(i));
        }
        return s;
    }

    static String key(Coordinate c) {
        return Math.round(c.x * 2) + ":" + Math.round(c.y * 2);
    }

    private static final class Cached {
        static final Cached NONE = new Cached(null, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        final List<Coordinate> path;
        final double cost;
        final double length;

        Cached(List<Coordinate> path, double cost, double length) {
            this.path = path;
            this.cost = cost;
            this.length = length;
        }

        Cached reverse() {
            if (path == null) {
                return NONE;
            }
            List<Coordinate> rev = new ArrayList<>(path);
            Collections.reverse(rev);
            return new Cached(rev, cost, length);
        }
    }
}
