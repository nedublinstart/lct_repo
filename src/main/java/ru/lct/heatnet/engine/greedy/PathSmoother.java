package ru.lct.heatnet.engine.greedy;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;

/**
 * Трасса из прямых отрезков: без сеточной «лесенки» и без мелких зигзагов.
 * Видимый граф даёт углы, этот шаг вытягивает хорды, пока линия не пересекает запрет.
 */
public final class PathSmoother {

    private PathSmoother() {
    }

    public static List<Coordinate> smooth(List<Coordinate> raw, ObstacleIndex obstacles, double keepDeg) {
        return straighten(raw, obstacles, keepDeg);
    }

    public static List<Coordinate> straighten(List<Coordinate> raw, ObstacleIndex obstacles) {
        return straighten(raw, obstacles, 18);
    }

    public static List<Coordinate> straighten(List<Coordinate> raw, ObstacleIndex obstacles, double keepDeg) {
        if (raw == null || raw.size() <= 2) {
            return raw;
        }
        List<Coordinate> pts = dedupe(raw, 0.55);
        pts = collapseHeading(pts, Math.max(8, keepDeg * 0.5));
        pts = collapseStairs(pts, obstacles);
        pts = stringPull(pts, obstacles);
        pts = rdpLegal(pts, obstacles, 2.8);
        pts = stringPull(pts, obstacles);
        pts = collapseHeading(pts, Math.max(12, keepDeg));
        if (pts.size() < 2) {
            return raw;
        }
        return pts;
    }

    /**
     * Первый сегмент — ввод от ИТП: его нельзя вытягивать сквозь здание.
     */
    public static List<Coordinate> straightenKeepStub(List<Coordinate> raw, ObstacleIndex obstacles) {
        return straightenKeepStub(raw, obstacles, 18);
    }

    public static List<Coordinate> straightenKeepStub(List<Coordinate> raw, ObstacleIndex obstacles, double keepDeg) {
        if (raw == null || raw.size() <= 2) {
            return raw;
        }
        Coordinate stub = new Coordinate(raw.get(0));
        List<Coordinate> rest = straighten(raw.subList(1, raw.size()), obstacles, keepDeg);
        List<Coordinate> out = new ArrayList<>();
        out.add(stub);
        if (rest == null || rest.isEmpty()) {
            out.add(new Coordinate(raw.get(raw.size() - 1)));
            return out;
        }
        int start = rest.get(0).distance(stub) < 0.45 ? 1 : 0;
        for (int i = start; i < rest.size(); i++) {
            out.add(new Coordinate(rest.get(i)));
        }
        if (out.size() < 2) {
            out.add(new Coordinate(raw.get(raw.size() - 1)));
        }
        return out;
    }

    public static List<Coordinate> collapseColinear(List<Coordinate> raw) {
        if (raw == null || raw.size() <= 2) {
            return raw;
        }
        return collapseHeading(dedupe(raw, 0.4), 8);
    }

    static boolean visible(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        if (a == null || b == null) {
            return false;
        }
        if (a.distance(b) < 0.35) {
            return true;
        }
        if (obstacles.segmentHitsAvoid(a, b, 0, true)) {
            return false;
        }
        return obstacles.allowsTravel(a, b);
    }

    private static List<Coordinate> dedupe(List<Coordinate> raw, double minM) {
        List<Coordinate> out = new ArrayList<>();
        Coordinate prev = null;
        for (Coordinate c : raw) {
            if (c == null) {
                continue;
            }
            if (prev != null && prev.distance(c) < minM) {
                continue;
            }
            out.add(new Coordinate(c));
            prev = c;
        }
        if (out.size() == 1 && raw.size() >= 2) {
            out.add(new Coordinate(raw.get(raw.size() - 1)));
        }
        return out;
    }

    private static List<Coordinate> collapseHeading(List<Coordinate> pts, double keepDeg) {
        if (pts.size() <= 2) {
            return pts;
        }
        List<Coordinate> out = new ArrayList<>();
        out.add(pts.get(0));
        for (int i = 1; i < pts.size() - 1; i++) {
            Coordinate a = out.get(out.size() - 1);
            Coordinate b = pts.get(i);
            Coordinate c = pts.get(i + 1);
            if (b.distance(a) < 0.5) {
                continue;
            }
            if (turnDeg(a, b, c) >= keepDeg && deviation(a, c, b) >= 0.9) {
                out.add(b);
            }
        }
        out.add(pts.get(pts.size() - 1));
        return out;
    }

    /**
     * Сетка 8-связности даёт чередование двух соседних октантов — заменяем на хорду.
     */
    private static List<Coordinate> collapseStairs(List<Coordinate> pts, ObstacleIndex obstacles) {
        if (pts.size() <= 3) {
            return pts;
        }
        List<Coordinate> out = new ArrayList<>();
        int i = 0;
        out.add(pts.get(0));
        while (i < pts.size() - 1) {
            int d1 = octant(pts.get(i), pts.get(i + 1));
            int d2 = -1;
            int k = i + 1;
            while (k + 1 < pts.size()) {
                int d = octant(pts.get(k), pts.get(k + 1));
                if (d == d1 || d == d2) {
                    k++;
                    continue;
                }
                if (d2 < 0 && stairOctant(d1, d)) {
                    d2 = d;
                    k++;
                    continue;
                }
                break;
            }
            while (k > i + 1 && !visible(obstacles, pts.get(i), pts.get(k))) {
                k--;
            }
            out.add(pts.get(k));
            i = k;
        }
        return out;
    }

    private static List<Coordinate> stringPull(List<Coordinate> pts, ObstacleIndex obstacles) {
        if (pts.size() <= 2) {
            return pts;
        }
        List<Coordinate> out = new ArrayList<>();
        int i = 0;
        out.add(pts.get(0));
        while (i < pts.size() - 1) {
            int best = i + 1;
            for (int j = pts.size() - 1; j > i + 1; j--) {
                if (visible(obstacles, pts.get(i), pts.get(j))) {
                    best = j;
                    break;
                }
            }
            out.add(pts.get(best));
            i = best;
        }
        return out;
    }

    private static List<Coordinate> rdpLegal(List<Coordinate> pts, ObstacleIndex obstacles, double eps) {
        if (pts.size() <= 2) {
            return pts;
        }
        boolean[] keep = new boolean[pts.size()];
        keep[0] = true;
        keep[pts.size() - 1] = true;
        rdp(pts, 0, pts.size() - 1, obstacles, eps, keep);
        List<Coordinate> out = new ArrayList<>();
        for (int i = 0; i < pts.size(); i++) {
            if (keep[i]) {
                out.add(pts.get(i));
            }
        }
        return out;
    }

    private static void rdp(List<Coordinate> pts, int i, int j, ObstacleIndex obstacles, double eps, boolean[] keep) {
        if (j <= i + 1) {
            return;
        }
        int mid = i + 1;
        double best = -1;
        for (int k = i + 1; k < j; k++) {
            double d = deviation(pts.get(i), pts.get(j), pts.get(k));
            if (d > best) {
                best = d;
                mid = k;
            }
        }
        if (best <= eps && visible(obstacles, pts.get(i), pts.get(j))) {
            return;
        }
        keep[mid] = true;
        rdp(pts, i, mid, obstacles, eps, keep);
        rdp(pts, mid, j, obstacles, eps, keep);
    }

    private static double deviation(Coordinate a, Coordinate c, Coordinate b) {
        double vx = c.x - a.x;
        double vy = c.y - a.y;
        double len = Math.hypot(vx, vy);
        if (len < 1e-6) {
            return a.distance(b);
        }
        double t = ((b.x - a.x) * vx + (b.y - a.y) * vy) / (len * len);
        t = Math.max(0, Math.min(1, t));
        double px = a.x + t * vx;
        double py = a.y + t * vy;
        return Math.hypot(b.x - px, b.y - py);
    }

    private static double turnDeg(Coordinate a, Coordinate b, Coordinate c) {
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

    private static int octant(Coordinate a, Coordinate b) {
        double ang = Math.atan2(b.y - a.y, b.x - a.x);
        int o = (int) Math.round(ang / (Math.PI / 4.0));
        if (o < 0) {
            o += 8;
        }
        return o & 7;
    }

    private static boolean stairOctant(int a, int b) {
        int d = Math.abs(a - b);
        d = Math.min(d, 8 - d);
        return d == 1 || d == 2;
    }
}
