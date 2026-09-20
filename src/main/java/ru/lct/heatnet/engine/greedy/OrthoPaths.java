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
        List<Coordinate> street = streetElbow(obstacles, a, b);
        if (street != null) {
            return street;
        }
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

    /**
     * Г-обход по осям ближайшей улицы, не по северу карты: кварталы Зиларта повёрнуты.
     */
    public static List<Coordinate> streetElbow(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        return streetElbow(obstacles, a, b, false);
    }

    public static List<Coordinate> streetElbow(ObstacleIndex obstacles, Coordinate a, Coordinate b,
                                               boolean dominant) {
        if (a == null || b == null || obstacles == null || obstacles.special() == null) {
            return null;
        }
        if (a.distance(b) < 0.4) {
            return two(a, b);
        }
        Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        List<Coordinate> axes = new ArrayList<>();
        if (dominant && obstacles.special().dominantAxes() != null) {
            axes.addAll(obstacles.special().dominantAxes());
        }
        SpecialLayer.Corridor c = obstacles.special().nearestCorridor(mid, 90);
        if (c == null || c.axis == null) {
            c = obstacles.special().nearestCorridor(a, 90);
        }
        if (c != null && c.axis != null) {
            axes.add(c.axis);
        }
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        for (Coordinate axis : axes) {
            if (axis == null) {
                continue;
            }
            List<Coordinate> elbow = streetElbowOn(obstacles, a, b, axis);
            if (elbow == null) {
                continue;
            }
            double len = length(elbow);
            if (len + 0.4 < bestLen) {
                bestLen = len;
                best = elbow;
            }
        }
        return best;
    }

    private static List<Coordinate> streetElbowOn(ObstacleIndex obstacles, Coordinate a, Coordinate b,
                                                   Coordinate axis) {
        double n = Math.hypot(axis.x, axis.y);
        if (n < 1e-9) {
            return null;
        }
        Coordinate u = new Coordinate(axis.x / n, axis.y / n);
        Coordinate v = new Coordinate(-u.y, u.x);
        if (alignedTo(a, b, u) || alignedTo(a, b, v)) {
            return legal(obstacles, a, b) ? two(a, b) : null;
        }
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double du = dx * u.x + dy * u.y;
        double dv = dx * v.x + dy * v.y;
        Coordinate alongU = new Coordinate(a.x + du * u.x, a.y + du * u.y);
        Coordinate alongV = new Coordinate(a.x + dv * v.x, a.y + dv * v.y);
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        for (Coordinate corner : new Coordinate[]{alongU, alongV}) {
            if (a.distance(corner) < 0.35 || b.distance(corner) < 0.35) {
                continue;
            }
            if (!legal(obstacles, a, corner) || !legal(obstacles, corner, b)) {
                continue;
            }
            List<Coordinate> elbow = new ArrayList<>(3);
            elbow.add(new Coordinate(a));
            elbow.add(new Coordinate(corner));
            elbow.add(new Coordinate(b));
            if (!keepElbow(obstacles, elbow) && a.distance(b) > SHORT_M) {
                SpecialLayer.Travel t1 = obstacles.special().inspect(a, corner);
                SpecialLayer.Travel t2 = obstacles.special().inspect(corner, b);
                boolean streetish = (t1.allowed && !t1.special) || (t2.allowed && !t2.special)
                        || obstacles.alongAvoid(a, corner, FACADE_M)
                        || obstacles.alongAvoid(corner, b, FACADE_M);
                if (!streetish && a.distance(corner) > SHORT_M && corner.distance(b) > SHORT_M) {
                    continue;
                }
            }
            double len = a.distance(corner) + corner.distance(b);
            if (len < bestLen) {
                bestLen = len;
                best = elbow;
            }
        }
        return best;
    }

    private static boolean alignedTo(Coordinate a, Coordinate b, Coordinate axis) {
        if (axis == null) {
            return false;
        }
        return SpecialLayer.crossingAngleDeg(a, b, axis) <= AXIS_DEG
                || SpecialLayer.crossingAngleDeg(a, b, axis) >= (90.0 - AXIS_DEG);
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

    /**
     * П-обход по осям улицы: смещение на тротуар и ход вдоль фасада, когда
     * Г режет корпус. Ребро OASG / OARSMT (Kahng–Robins 1-Steiner).
     */
    public static List<Coordinate> sidewalkU(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        return sidewalkU(obstacles, a, b, false);
    }

    /**
     * @param rejectTouch true — на выдаче не берём П, которое касается корпуса;
     *                    следующий оффсет (10, 14, … м) остаётся кандидатом.
     */
    public static List<Coordinate> sidewalkU(ObstacleIndex obstacles, Coordinate a, Coordinate b,
                                            boolean rejectTouch) {
        if (a == null || b == null || obstacles == null || obstacles.special() == null) {
            return null;
        }
        double eu = a.distance(b);
        if (eu < 8) {
            return null;
        }
        Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        SpecialLayer.Corridor c = obstacles.special().nearestCorridor(mid, 110);
        if (c == null || c.axis == null) {
            c = obstacles.special().nearestCorridor(a, 110);
        }
        if (c == null || c.axis == null) {
            c = obstacles.special().nearestCorridor(b, 110);
        }
        if (c == null || c.axis == null) {
            return null;
        }
        double n = Math.hypot(c.axis.x, c.axis.y);
        if (n < 1e-9) {
            return null;
        }
        Coordinate u = new Coordinate(c.axis.x / n, c.axis.y / n);
        Coordinate v = new Coordinate(-u.y, u.x);
        List<Coordinate> dirs = new ArrayList<>();
        dirs.add(v);
        dirs.add(u);
        if (obstacles.special() != null) {
            for (Coordinate axis : obstacles.special().dominantAxes()) {
                if (axis == null) {
                    continue;
                }
                double an = Math.hypot(axis.x, axis.y);
                if (an < 1e-9) {
                    continue;
                }
                Coordinate du = new Coordinate(axis.x / an, axis.y / an);
                Coordinate dv = new Coordinate(-du.y, du.x);
                dirs.add(dv);
                dirs.add(du);
            }
        }
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        double cap = eu * 2.15 + 36;
        for (Coordinate axis : dirs) {
            for (int s : new int[]{1, -1}) {
                for (double off : new double[]{6, 10, 14, 20, 28, 38, 52, 70, 90}) {
                    Coordinate au = new Coordinate(a.x + s * off * axis.x, a.y + s * off * axis.y);
                    Coordinate bu = new Coordinate(b.x + s * off * axis.x, b.y + s * off * axis.y);
                    List<Coordinate> path = uPath(obstacles, a, au, bu, b);
                    if (path == null) {
                        continue;
                    }
                    if (rejectTouch && pathTouches(obstacles, path)) {
                        continue;
                    }
                    double len = length(path);
                    if (len + 0.4 < bestLen && len <= cap) {
                        bestLen = len;
                        best = path;
                    }
                }
            }
        }
        return best;
    }

    private static List<Coordinate> uPath(ObstacleIndex obstacles, Coordinate a, Coordinate au,
                                          Coordinate bu, Coordinate b) {
        if (au == null || bu == null) {
            return null;
        }
        if (a.distance(au) < 0.35 && b.distance(bu) < 0.35) {
            return null;
        }
        if (au.distance(bu) < 0.4) {
            return null;
        }
        if (!legal(obstacles, a, au) || !legal(obstacles, bu, b) || !legal(obstacles, au, bu)) {
            return null;
        }
        if (au.distance(bu) > SHORT_M + 8 && !nearlyAxis(au, bu)
                && !obstacles.alongAvoid(au, bu, FACADE_M + 4)
                && !usefulLeg(obstacles, au, bu)) {
            return null;
        }
        List<Coordinate> out = new ArrayList<>();
        out.add(new Coordinate(a));
        if (a.distance(au) >= 0.4) {
            out.add(new Coordinate(au));
        }
        if (out.get(out.size() - 1).distance(bu) >= 0.4) {
            out.add(new Coordinate(bu));
        }
        if (out.get(out.size() - 1).distance(b) >= 0.4) {
            out.add(new Coordinate(b));
        }
        return out.size() >= 3 ? out : null;
    }

    private static boolean pathTouches(ObstacleIndex obstacles, List<Coordinate> path) {
        if (path == null || path.size() < 2) {
            return true;
        }
        for (int i = 1; i < path.size(); i++) {
            if (obstacles.segmentHitsAvoid(path.get(i - 1), path.get(i), 0, false)) {
                return true;
            }
        }
        return false;
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
