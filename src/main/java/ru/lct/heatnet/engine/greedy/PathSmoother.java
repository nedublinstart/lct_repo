package ru.lct.heatnet.engine.greedy;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;

public final class PathSmoother {

    private PathSmoother() {
    }

    public static List<Coordinate> smooth(List<Coordinate> raw, ObstacleIndex obstacles, double keepDeg) {
        if (raw == null || raw.size() <= 2) {
            return raw;
        }
        List<Coordinate> collapsed = collapse(raw, keepDeg);
        List<Coordinate> pulled = new ArrayList<>();
        pulled.add(collapsed.get(0));
        int i = 0;
        while (i < collapsed.size() - 1) {
            int best = i + 1;
            for (int j = collapsed.size() - 1; j > i + 1; j--) {
                if (!shortcutOk(obstacles, collapsed, i, j)) {
                    continue;
                }
                best = j;
                break;
            }
            pulled.add(collapsed.get(best));
            i = best;
        }
        return pulled;
    }

    private static boolean shortcutOk(ObstacleIndex obstacles, List<Coordinate> pts, int i, int j) {
        Coordinate a = pts.get(i);
        Coordinate b = pts.get(j);
        if (obstacles.segmentHitsAvoid(a, b, 0.35)) {
            return false;
        }
        SpecialLayer.Travel t = obstacles.special().inspect(a, b);
        if (!t.allowed) {
            return false;
        }
        double via = 0;
        for (int k = i; k < j; k++) {
            via += obstacles.travelCost(pts.get(k), pts.get(k + 1));
        }
        return t.cost <= via * 1.08;
    }

    private static List<Coordinate> collapse(List<Coordinate> raw, double keepDeg) {
        List<Coordinate> out = new ArrayList<>();
        out.add(raw.get(0));
        for (int i = 1; i < raw.size() - 1; i++) {
            Coordinate a = out.get(out.size() - 1);
            Coordinate b = raw.get(i);
            Coordinate c = raw.get(i + 1);
            if (b.distance(a) < 0.4) {
                continue;
            }
            double ang = turnDeg(a, b, c);
            if (ang >= keepDeg) {
                out.add(b);
            }
        }
        out.add(raw.get(raw.size() - 1));
        return out;
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
}
