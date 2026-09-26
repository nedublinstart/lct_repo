package ru.lct.heatnet.engine.flow;

/** Примитивы плоской геометрии на double без аллокаций. */
final class Geo {

    private Geo() {
    }

    static double cross(double ax, double ay, double bx, double by, double cx, double cy) {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    }

    static double dist(double ax, double ay, double bx, double by) {
        double dx = bx - ax;
        double dy = by - ay;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Отрезки ab и cd пересекаются во внутренней точке обоих (касание концом не считается). */
    static boolean properCross(double ax, double ay, double bx, double by,
                               double cx, double cy, double dx, double dy) {
        double d1 = cross(ax, ay, bx, by, cx, cy);
        double d2 = cross(ax, ay, bx, by, dx, dy);
        if ((d1 > 1e-12 && d2 > 1e-12) || (d1 < -1e-12 && d2 < -1e-12)) {
            return false;
        }
        double d3 = cross(cx, cy, dx, dy, ax, ay);
        double d4 = cross(cx, cy, dx, dy, bx, by);
        if ((d3 > 1e-12 && d4 > 1e-12) || (d3 < -1e-12 && d4 < -1e-12)) {
            return false;
        }
        double s1 = Math.abs(d1) <= 1e-12 ? 0 : Math.signum(d1);
        double s2 = Math.abs(d2) <= 1e-12 ? 0 : Math.signum(d2);
        double s3 = Math.abs(d3) <= 1e-12 ? 0 : Math.signum(d3);
        double s4 = Math.abs(d4) <= 1e-12 ? 0 : Math.signum(d4);
        return s1 * s2 < 0 && s3 * s4 < 0;
    }

    /** Отрезки ab и cd имеют общую точку (включая касание и наложение). */
    static boolean touches(double ax, double ay, double bx, double by,
                           double cx, double cy, double dx, double dy) {
        double d1 = cross(ax, ay, bx, by, cx, cy);
        double d2 = cross(ax, ay, bx, by, dx, dy);
        double d3 = cross(cx, cy, dx, dy, ax, ay);
        double d4 = cross(cx, cy, dx, dy, bx, by);
        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            return true;
        }
        return (Math.abs(d1) < 1e-9 && onSeg(ax, ay, bx, by, cx, cy))
                || (Math.abs(d2) < 1e-9 && onSeg(ax, ay, bx, by, dx, dy))
                || (Math.abs(d3) < 1e-9 && onSeg(cx, cy, dx, dy, ax, ay))
                || (Math.abs(d4) < 1e-9 && onSeg(cx, cy, dx, dy, bx, by));
    }

    private static boolean onSeg(double ax, double ay, double bx, double by, double px, double py) {
        return px >= Math.min(ax, bx) - 1e-9 && px <= Math.max(ax, bx) + 1e-9
                && py >= Math.min(ay, by) - 1e-9 && py <= Math.max(ay, by) + 1e-9;
    }

    /** Параметр t ∈ [0,1] пересечения ab с прямой cd, или NaN. */
    static double crossParam(double ax, double ay, double bx, double by,
                             double cx, double cy, double dx, double dy) {
        double rx = bx - ax;
        double ry = by - ay;
        double sx = dx - cx;
        double sy = dy - cy;
        double den = rx * sy - ry * sx;
        if (Math.abs(den) < 1e-15) {
            return Double.NaN;
        }
        double t = ((cx - ax) * sy - (cy - ay) * sx) / den;
        double u = ((cx - ax) * ry - (cy - ay) * rx) / den;
        if (t < 0 || t > 1 || u < 0 || u > 1) {
            return Double.NaN;
        }
        return t;
    }

    /** Параметр проекции p на отрезок ab, зажатый в [0,1]. */
    static double footParam(double ax, double ay, double bx, double by, double px, double py) {
        double dx = bx - ax;
        double dy = by - ay;
        double l2 = dx * dx + dy * dy;
        if (l2 < 1e-18) {
            return 0;
        }
        double t = ((px - ax) * dx + (py - ay) * dy) / l2;
        return t < 0 ? 0 : (t > 1 ? 1 : t);
    }

    static double segDist(double ax, double ay, double bx, double by, double px, double py) {
        double t = footParam(ax, ay, bx, by, px, py);
        return dist(ax + t * (bx - ax), ay + t * (by - ay), px, py);
    }

    /** Расстояние между отрезками ab и cd; 0, если они имеют общую точку. */
    static double segSegDist(double ax, double ay, double bx, double by,
                             double cx, double cy, double dx, double dy) {
        double d1 = cross(ax, ay, bx, by, cx, cy);
        double d2 = cross(ax, ay, bx, by, dx, dy);
        double d3 = cross(cx, cy, dx, dy, ax, ay);
        double d4 = cross(cx, cy, dx, dy, bx, by);
        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            return 0;
        }
        return Math.min(Math.min(segDist(ax, ay, bx, by, cx, cy), segDist(ax, ay, bx, by, dx, dy)),
                Math.min(segDist(cx, cy, dx, dy, ax, ay), segDist(cx, cy, dx, dy, bx, by)));
    }

    /** Синус угла между направлениями ab и cd (0 — параллельно, 1 — перпендикулярно). */
    static double sinAngle(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
        double ux = bx - ax;
        double uy = by - ay;
        double vx = dx - cx;
        double vy = dy - cy;
        double lu = Math.hypot(ux, uy);
        double lv = Math.hypot(vx, vy);
        if (lu < 1e-12 || lv < 1e-12) {
            return 1;
        }
        return Math.abs(ux * vy - uy * vx) / (lu * lv);
    }

    static boolean segmentTouchesBox(double ax, double ay, double bx, double by,
                                     double x0, double y0, double x1, double y1) {
        if (Math.max(ax, bx) < x0 || Math.min(ax, bx) > x1 || Math.max(ay, by) < y0 || Math.min(ay, by) > y1) {
            return false;
        }
        if ((ax >= x0 && ax <= x1 && ay >= y0 && ay <= y1) || (bx >= x0 && bx <= x1 && by >= y0 && by <= y1)) {
            return true;
        }
        double c1 = cross(ax, ay, bx, by, x0, y0);
        double c2 = cross(ax, ay, bx, by, x1, y0);
        double c3 = cross(ax, ay, bx, by, x1, y1);
        double c4 = cross(ax, ay, bx, by, x0, y1);
        boolean pos = c1 > 0 || c2 > 0 || c3 > 0 || c4 > 0;
        boolean neg = c1 < 0 || c2 < 0 || c3 < 0 || c4 < 0;
        return pos && neg || c1 == 0 || c2 == 0 || c3 == 0 || c4 == 0;
    }

    /** Точка на ломаной (x,y пары) на расстоянии s от начала. */
    static double[] at(double[] line, double s) {
        double acc = 0;
        for (int i = 0; i + 3 < line.length; i += 2) {
            double l = dist(line[i], line[i + 1], line[i + 2], line[i + 3]);
            if (acc + l >= s || i + 4 >= line.length) {
                double t = l < 1e-12 ? 0 : Math.max(0, Math.min(1, (s - acc) / l));
                return new double[]{line[i] + t * (line[i + 2] - line[i]), line[i + 1] + t * (line[i + 3] - line[i + 1])};
            }
            acc += l;
        }
        return new double[]{line[0], line[1]};
    }

    /** Отклонение от прямой в вершине b, градусы. 0 — прямо, 90 — прямой угол. */
    static double deflection(double ax, double ay, double bx, double by, double cx, double cy) {
        double ux = ax - bx;
        double uy = ay - by;
        double vx = cx - bx;
        double vy = cy - by;
        double nu = Math.hypot(ux, uy);
        double nv = Math.hypot(vx, vy);
        if (nu < 1e-6 || nv < 1e-6) {
            return 0;
        }
        double cos = (ux * vx + uy * vy) / (nu * nv);
        if (cos > 1) {
            cos = 1;
        } else if (cos < -1) {
            cos = -1;
        }
        return 180.0 - Math.toDegrees(Math.acos(cos));
    }

    static double length(double[] line) {
        double acc = 0;
        for (int i = 0; i + 3 < line.length; i += 2) {
            acc += dist(line[i], line[i + 1], line[i + 2], line[i + 3]);
        }
        return acc;
    }

    /** Кусок ломаной между расстояниями s0 ≤ s1 от начала. */
    static double[] slice(double[] line, double s0, double s1) {
        double[] out = new double[line.length + 4];
        int n = 0;
        double acc = 0;
        double[] a = at(line, s0);
        out[n++] = a[0];
        out[n++] = a[1];
        for (int i = 0; i + 3 < line.length; i += 2) {
            double l = dist(line[i], line[i + 1], line[i + 2], line[i + 3]);
            double end = acc + l;
            if (end > s0 + 1e-9 && end < s1 - 1e-9) {
                out[n++] = line[i + 2];
                out[n++] = line[i + 3];
            }
            acc = end;
        }
        double[] b = at(line, s1);
        out[n++] = b[0];
        out[n++] = b[1];
        return java.util.Arrays.copyOf(out, n);
    }

    static double[] reverse(double[] line) {
        double[] out = new double[line.length];
        for (int i = 0, j = line.length - 2; i < line.length; i += 2, j -= 2) {
            out[i] = line[j];
            out[i + 1] = line[j + 1];
        }
        return out;
    }

    static double[] concat(double[] a, double[] b) {
        if (a == null || a.length == 0) {
            return b;
        }
        if (b == null || b.length == 0) {
            return a;
        }
        boolean joint = Math.abs(a[a.length - 2] - b[0]) < 1e-9 && Math.abs(a[a.length - 1] - b[1]) < 1e-9;
        int skip = joint ? 2 : 0;
        double[] out = new double[a.length + b.length - skip];
        System.arraycopy(a, 0, out, 0, a.length);
        System.arraycopy(b, skip, out, a.length, b.length - skip);
        return out;
    }
}
