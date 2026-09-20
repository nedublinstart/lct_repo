package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import ru.lct.heatnet.engine.greedy.GridPathfinder;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.OrthoPaths;
import ru.lct.heatnet.engine.greedy.PathSmoother;
import ru.lct.heatnet.engine.greedy.SpecialLayer;
import ru.lct.heatnet.engine.greedy.VisibilityPathfinder;

/**
 * Кэш путей: видимый граф (фасады + прямые углы), запасная 4-связная сетка, обход контура.
 */
public final class MetricPathCache implements PathMetric {

    private final GridPathfinder grid;
    private final VisibilityPathfinder visibility;
    private final ObstacleIndex obstacles;
    private final double keepDeg;
    private final Envelope env;
    private final Map<String, Cached> cache = new HashMap<>();

    public MetricPathCache(GridPathfinder grid, VisibilityPathfinder visibility, ObstacleIndex obstacles,
                           double keepDeg, Envelope env) {
        this.grid = grid;
        this.visibility = visibility;
        this.obstacles = obstacles;
        this.keepDeg = keepDeg;
        this.env = env;
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
        List<Coordinate> path = pick(visibility.find(a, b), true);
        if (path == null) {
            path = pick(grid.find(a, b), true);
        }
        if (path == null) {
            path = via(a, b);
        }
        if (path == null) {
            return Cached.NONE;
        }
        if (hasLongOpenDiagonal(path)) {
            List<Coordinate> elbow = OrthoPaths.usefulElbow(obstacles, a, b);
            if (ok(elbow) && !hasLongOpenDiagonal(elbow)) {
                path = elbow;
            }
        }
        if (obstacles.pathHitsAvoid(path, 0)) {
            return Cached.NONE;
        }
        double cost = costOf(path, obstacles);
        if (!Double.isFinite(cost)) {
            return Cached.NONE;
        }
        return new Cached(path, cost, lengthOf(path));
    }

    private List<Coordinate> pick(List<Coordinate> raw, boolean polish) {
        if (raw == null || raw.size() < 2) {
            return null;
        }
        if (polish) {
            List<Coordinate> slim = PathSmoother.smooth(raw, obstacles, keepDeg);
            if (ok(slim)) {
                return slim;
            }
        }
        return ok(raw) ? raw : null;
    }

    private boolean ok(List<Coordinate> path) {
        if (path == null || path.size() < 2) {
            return false;
        }
        if (obstacles.pathHitsAvoid(path, 0)) {
            return false;
        }
        return Double.isFinite(costOf(path, obstacles));
    }

    private boolean hasLongOpenDiagonal(List<Coordinate> path) {
        for (int i = 1; i < path.size(); i++) {
            Coordinate a = path.get(i - 1);
            Coordinate b = path.get(i);
            if (!OrthoPaths.longOpenDiagonal(a, b) && a.distance(b) < 40) {
                continue;
            }
            if (obstacles.alongAvoid(a, b, OrthoPaths.FACADE_M)) {
                continue;
            }
            SpecialLayer.Travel t = obstacles.special().inspect(a, b);
            if (t.allowed && t.special && t.crossingAngleDeg + 1e-6 >= OrthoPaths.PERP_MIN_DEG) {
                continue;
            }
            return true;
        }
        return false;
    }

    private List<Coordinate> via(Coordinate a, Coordinate b) {
        if (env == null || env.isNull()) {
            return null;
        }
        double minX = env.getMinX() + 8;
        double maxX = env.getMaxX() - 8;
        double minY = env.getMinY() + 8;
        double maxY = env.getMaxY() - 8;
        double cx = (minX + maxX) / 2;
        double cy = (minY + maxY) / 2;
        Coordinate[] wps = {
                new Coordinate(cx, minY),
                new Coordinate(cx, maxY),
                new Coordinate(minX, cy),
                new Coordinate(maxX, cy)
        };
        List<Coordinate> best = null;
        double bestCost = Double.POSITIVE_INFINITY;
        for (Coordinate wp : wps) {
            List<Coordinate> p1 = grid.find(a, wp);
            List<Coordinate> p2 = grid.find(wp, b);
            if (p1 == null || p2 == null) {
                continue;
            }
            List<Coordinate> joined = new ArrayList<>(p1);
            joined.addAll(p2.subList(1, p2.size()));
            joined = PathSmoother.smooth(joined, obstacles, keepDeg);
            double c = costOf(joined, obstacles);
            if (c < bestCost) {
                bestCost = c;
                best = joined;
            }
        }
        return best;
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
