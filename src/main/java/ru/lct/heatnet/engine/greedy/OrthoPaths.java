package ru.lct.heatnet.engine.greedy;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;

/**
 * Прямоугольная геометрия трассы: вдоль фасада / вдоль улицы, поворот 90°,
 * пересечение проезжей почти перпендикулярно. Без диагоналей через парк и без
 * сеточной лесенки.
 */
public final class OrthoPaths {

    public static final double AXIS_TOL_M = 2.8;
    public static final double AXIS_DEG = 14;
    public static final double PERP_MIN_DEG = 70;
    public static final double SHORT_M = 22;
    public static final double HIT_WIDTH_M = 0.22;
    public static final double FACADE_M = 8.0;

    private OrthoPaths() {
    }

    public static boolean axisAligned(Coordinate a, Coordinate b) {
        if (a == null || b == null) {
            return false;
        }
        return Math.abs(a.x - b.x) <= AXIS_TOL_M || Math.abs(a.y - b.y) <= AXIS_TOL_M;
    }

    public static boolean nearlyAxis(Coordinate a, Coordinate b) {
        if (axisAligned(a, b)) {
            return true;
        }
        if (a == null || b == null) {
            return false;
        }
        double dx = Math.abs(b.x - a.x);
        double dy = Math.abs(b.y - a.y);
        if (dx < 1e-6 && dy < 1e-6) {
            return true;
        }
        double ang = Math.toDegrees(Math.atan2(dy, dx));
        return ang <= AXIS_DEG || ang >= (90.0 - AXIS_DEG);
    }

    public static boolean legal(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        if (a == null || b == null) {
            return false;
        }
        if (a.distance(b) < 0.25) {
            return true;
        }
        if (obstacles.segmentHitsAvoid(a, b, HIT_WIDTH_M, true)) {
            return false;
        }
        return obstacles.allowsTravel(a, b);
    }

    /**
     * Ребро видимого графа: короткая связь, вдоль фасада, ось, или перпендикуляр через улицу.
     * Длинная диагональ через двор/парк/проспект — нет.
     */
    public static boolean usefulChord(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        if (!legal(obstacles, a, b)) {
            return false;
        }
        double d = a.distance(b);
        if (d <= 12) {
            return true;
        }
        SpecialLayer.Travel t = obstacles.special().inspect(a, b);
        if (!t.allowed) {
            return false;
        }
        if (t.special && t.crossingAngleDeg + 1e-6 >= PERP_MIN_DEG && d <= obstacles.maxStreetEdgeM() + 8) {
            return true;
        }
        return obstacles.alongAvoid(a, b, FACADE_M);
    }

    public static boolean usefulLeg(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        if (!legal(obstacles, a, b)) {
            return false;
        }
        double d = a.distance(b);
        if (d <= SHORT_M + 8) {
            return true;
        }
        if (obstacles.alongAvoid(a, b, FACADE_M)) {
            return true;
        }
        SpecialLayer.Travel t = obstacles.special().inspect(a, b);
        if (t.allowed && t.special && t.crossingAngleDeg + 1e-6 >= PERP_MIN_DEG
                && d <= obstacles.maxStreetEdgeM() + 8) {
            return true;
        }
        return nearlyAxis(a, b) && obstacles.alongAvoid(a, b, FACADE_M + 3);
    }

    public static List<Coordinate> bestElbow(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        List<Coordinate> hv = elbow(obstacles, a, b, true);
        List<Coordinate> vh = elbow(obstacles, a, b, false);
        if (hv == null) {
            return vh;
        }
        if (vh == null) {
            return hv;
        }
        return length(hv) <= length(vh) ? hv : vh;
    }

    public static List<Coordinate> usefulElbow(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        List<Coordinate> hv = elbow(obstacles, a, b, true);
        List<Coordinate> vh = elbow(obstacles, a, b, false);
        boolean hOk = keepElbow(obstacles, hv);
        boolean vOk = keepElbow(obstacles, vh);
        if (hOk && vOk) {
            return length(hv) <= length(vh) ? hv : vh;
        }
        if (hOk) {
            return hv;
        }
        if (vOk) {
            return vh;
        }
        return null;
    }

    public static boolean keepElbow(ObstacleIndex obstacles, List<Coordinate> elbow) {
        if (elbow == null || elbow.size() < 2) {
            return false;
        }
        if (elbow.size() == 2) {
            return usefulChord(obstacles, elbow.get(0), elbow.get(1));
        }
        if (elbow.size() != 3) {
            return false;
        }
        Coordinate a = elbow.get(0);
        Coordinate c = elbow.get(1);
        Coordinate b = elbow.get(2);
        boolean l1s = specialPerp(obstacles, a, c);
        boolean l2s = specialPerp(obstacles, c, b);
        boolean l1a = obstacles.alongAvoid(a, c, FACADE_M);
        boolean l2a = obstacles.alongAvoid(c, b, FACADE_M);
        boolean l1short = a.distance(c) <= SHORT_M + 16;
        boolean l2short = c.distance(b) <= SHORT_M + 16;
        if (l1s && (l2a || l2s || l2short)) {
            return true;
        }
        if (l2s && (l1a || l1s || l1short)) {
            return true;
        }
        if (l1a && l2a) {
            return true;
        }
        return l1short && l2short;
    }

    /**
     * Для сглаживания уже найденного пути: хорда, если она «полезная», иначе Г-образный обход,
     * даже через открытое пространство (лесенку сетки схлопываем в прямой угол, не в диагональ).
     */
    public static List<Coordinate> shortcut(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        if (a == null || b == null) {
            return null;
        }
        if (a.distance(b) < 0.4) {
            return two(a, b);
        }
        if (usefulChord(obstacles, a, b)) {
            return two(a, b);
        }
        List<Coordinate> elbow = usefulElbow(obstacles, a, b);
        if (elbow != null) {
            return elbow;
        }
        if (obstacles.avoidPolygons().isEmpty()) {
            return bestElbow(obstacles, a, b);
        }
        return null;
    }

    public static List<Coordinate> collapse(List<Coordinate> pts, ObstacleIndex obstacles) {
        if (pts == null || pts.size() <= 2) {
            return copy(pts);
        }
        List<Coordinate> out = new ArrayList<>();
        int i = 0;
        out.add(new Coordinate(pts.get(0)));
        int guard = 0;
        while (i < pts.size() - 1 && guard++ < pts.size() + 4) {
            int best = i + 1;
            List<Coordinate> bestSpan = null;
            for (int j = pts.size() - 1; j > i; j--) {
                List<Coordinate> span = shortcut(obstacles, pts.get(i), pts.get(j));
                if (span != null && span.size() >= 2) {
                    best = j;
                    bestSpan = span;
                    break;
                }
            }
            if (bestSpan == null) {
                out.add(new Coordinate(pts.get(i + 1)));
                i++;
                continue;
            }
            for (int k = 1; k < bestSpan.size(); k++) {
                Coordinate c = bestSpan.get(k);
                if (out.get(out.size() - 1).distance(c) >= 0.35) {
                    out.add(new Coordinate(c));
                }
            }
            if (best <= i) {
                break;
            }
            i = best;
        }
        return out;
    }

    public static double length(List<Coordinate> path) {
        if (path == null || path.size() < 2) {
            return 0;
        }
        double s = 0;
        for (int i = 1; i < path.size(); i++) {
            s += path.get(i - 1).distance(path.get(i));
        }
        return s;
    }

    public static double turnDeg(Coordinate a, Coordinate b, Coordinate c) {
        double ux = b.x - a.x;
        double uy = b.y - a.y;
        double vx = c.x - b.x;
        double vy = c.y - b.y;
        double nu = Math.hypot(ux, uy);
        double nv = Math.hypot(vx, vy);
        if (nu < 1e-6 || nv < 1e-6) {
            return 0;
        }
        double cos = (ux * vx + uy * vy) / (nu * nv);
        cos = Math.max(-1, Math.min(1, cos));
        return Math.toDegrees(Math.acos(cos));
    }

    public static boolean rightAngle(Coordinate a, Coordinate b, Coordinate c) {
        double t = turnDeg(a, b, c);
        return t >= 75 && t <= 105;
    }

    public static boolean longOpenDiagonal(Coordinate a, Coordinate b) {
        if (a == null || b == null) {
            return false;
        }
        double d = a.distance(b);
        if (d < 36) {
            return false;
        }
        if (nearlyAxis(a, b)) {
            return false;
        }
        double dx = Math.abs(b.x - a.x);
        double dy = Math.abs(b.y - a.y);
        return dx > 12 && dy > 12;
    }

    static List<Coordinate> elbow(ObstacleIndex obstacles, Coordinate a, Coordinate b, boolean hv) {
        if (a == null || b == null) {
            return null;
        }
        if (axisAligned(a, b)) {
            return legal(obstacles, a, b) ? two(a, b) : null;
        }
        Coordinate corner = hv ? new Coordinate(b.x, a.y) : new Coordinate(a.x, b.y);
        if (a.distance(corner) < 0.35 || b.distance(corner) < 0.35) {
            return legal(obstacles, a, b) ? two(a, b) : null;
        }
        if (!legal(obstacles, a, corner) || !legal(obstacles, corner, b)) {
            return null;
        }
        List<Coordinate> out = new ArrayList<>(3);
        out.add(new Coordinate(a));
        out.add(corner);
        out.add(new Coordinate(b));
        return out;
    }

    private static boolean specialPerp(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        SpecialLayer.Travel t = obstacles.special().inspect(a, b);
        return t.allowed && t.special && t.crossingAngleDeg + 1e-6 >= PERP_MIN_DEG
                && a.distance(b) <= obstacles.maxStreetEdgeM() + 12;
    }

    private static List<Coordinate> two(Coordinate a, Coordinate b) {
        List<Coordinate> out = new ArrayList<>(2);
        out.add(new Coordinate(a));
        out.add(new Coordinate(b));
        return out;
    }

    private static List<Coordinate> copy(List<Coordinate> raw) {
        if (raw == null) {
            return null;
        }
        List<Coordinate> out = new ArrayList<>(raw.size());
        for (Coordinate c : raw) {
            if (c != null) {
                out.add(new Coordinate(c));
            }
        }
        return out;
    }
}
