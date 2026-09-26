package ru.lct.heatnet.engine.flow;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.simplify.TopologyPreservingSimplifier;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;

/**
 * Выходы ИТП. Точка подключения лежит внутри своего корпуса; ввод идёт перпендикулярно одной из
 * ближайших стен (не дальше самой близкой плюс {@link #NEAR_M}) и продолжается по той же нормали,
 * пока ось не выйдет из зоны минимального расстояния. ИТП, основание перпендикуляра и выход лежат на
 * одной прямой, поэтому ввод — один прямой отрезок. Выход проверяется точно по исходным контурам
 * соседних зданий; корпус своего ОКС для ввода не препятствие.
 */
final class Ports {

    private static final double NEAR_M = 1.2;
    private static final double FALLBACK_NEAR_M = 6.0;
    private static final double WALL_MIN_M = 2.5;
    private static final double CORNER_KEEP_M = 0.5;
    private static final double STEP_M = 0.25;
    private static final double RAY_MAX_M = 60;
    private static final double HOST_SEARCH_M = 15;

    static final class Terminal {
        int index;
        String id;
        double flow;
        double x;
        double y;
        int host = -1;
        int first;
        int count;
    }

    final List<Terminal> terms = new ArrayList<>();
    double[] px = new double[16];
    double[] py = new double[16];
    double[] stub = new double[16];
    double[] violation = new double[16];
    int[] owner = new int[16];
    int count;

    Ports(Scene scene, FreeSpace space) {
        for (ProspectiveOks o : scene.oks) {
            if (o.connection == null) {
                continue;
            }
            Terminal t = new Terminal();
            t.index = terms.size();
            t.id = o.id;
            t.flow = o.flowTph;
            t.x = o.connection.getX();
            t.y = o.connection.getY();
            t.host = hostOf(space, o, t.x, t.y);
            t.first = count;
            build(space, t);
            t.count = count - t.first;
            terms.add(t);
        }
        px = java.util.Arrays.copyOf(px, count);
        py = java.util.Arrays.copyOf(py, count);
        stub = java.util.Arrays.copyOf(stub, count);
        violation = java.util.Arrays.copyOf(violation, count);
        owner = java.util.Arrays.copyOf(owner, count);
    }

    private static int hostOf(FreeSpace space, ProspectiveOks o, double x, double y) {
        String own = o.id + "-footprint";
        for (FreeSpace.Avoid a : space.avoids) {
            if (own.equals(a.id)) {
                return a.index;
            }
        }
        return space.avoidAt(x, y, HOST_SEARCH_M);
    }

    private void build(FreeSpace space, Terminal t) {
        List<double[]> cands = new ArrayList<>();
        if (t.host >= 0) {
            Polygon part = partNear(space.avoids.get(t.host).raw, t.x, t.y);
            if (part != null) {
                walls(space, t, part, cands);
            }
        }
        if (cands.isEmpty()) {
            radial(space, t, cands);
        }
        double nearest = Double.POSITIVE_INFINITY;
        for (double[] c : cands) {
            nearest = Math.min(nearest, c[4]);
        }
        boolean added = pick(t, cands, nearest + NEAR_M, true);
        if (!added) {
            added = pick(t, cands, nearest + FALLBACK_NEAR_M, true);
        }
        if (!added) {
            double least = Double.POSITIVE_INFINITY;
            for (double[] c : cands) {
                if (c[4] <= nearest + NEAR_M) {
                    least = Math.min(least, c[5]);
                }
            }
            for (double[] c : cands) {
                if (c[4] <= nearest + NEAR_M && c[5] <= least + 1e-6) {
                    add(t, c);
                }
            }
        }
    }

    /** Кандидаты {portX, portY, footX, footY, indoor, violation}. */
    private boolean pick(Terminal t, List<double[]> cands, double maxIndoor, boolean validOnly) {
        boolean any = false;
        for (double[] c : cands) {
            if (c[4] <= maxIndoor + 1e-9 && (!validOnly || c[5] <= 1e-6)) {
                add(t, c);
                any = true;
            }
        }
        return any;
    }

    private void add(Terminal t, double[] c) {
        for (int i = t.first; i < count; i++) {
            if (Geo.dist(px[i], py[i], c[0], c[1]) < 1.5) {
                return;
            }
        }
        if (count == px.length) {
            int cap = count * 2;
            px = java.util.Arrays.copyOf(px, cap);
            py = java.util.Arrays.copyOf(py, cap);
            stub = java.util.Arrays.copyOf(stub, cap);
            violation = java.util.Arrays.copyOf(violation, cap);
            owner = java.util.Arrays.copyOf(owner, cap);
        }
        px[count] = c[0];
        py[count] = c[1];
        stub[count] = Geo.dist(t.x, t.y, c[0], c[1]);
        violation[count] = c[5];
        owner[count] = t.index;
        count++;
    }

    private static Polygon partNear(Geometry g, double x, double y) {
        Polygon best = null;
        double bestD = Double.POSITIVE_INFINITY;
        Geometry p = GeoJsonGeometries.GF.createPoint(new Coordinate(x, y));
        for (int i = 0; i < g.getNumGeometries(); i++) {
            Geometry part = g.getGeometryN(i);
            if (!(part instanceof Polygon)) {
                continue;
            }
            double d = part.distance(p);
            if (d < bestD) {
                bestD = d;
                best = (Polygon) part;
            }
        }
        return best;
    }

    /** Перпендикуляры на стены упрощённого внешнего контура своей части корпуса. */
    private static void walls(FreeSpace space, Terminal t, Polygon part, List<double[]> out) {
        Geometry simple = TopologyPreservingSimplifier.simplify(part, 0.5);
        Polygon shell = simple instanceof Polygon && !simple.isEmpty() ? (Polygon) simple : part;
        double[] r = FreeSpace.coords(shell.getExteriorRing().getCoordinates());
        Polygon test = part;
        for (int i = 0; i + 3 < r.length; i += 2) {
            double ax = r[i];
            double ay = r[i + 1];
            double bx = r[i + 2];
            double by = r[i + 3];
            double len = Geo.dist(ax, ay, bx, by);
            if (len < WALL_MIN_M) {
                continue;
            }
            double s = ((t.x - ax) * (bx - ax) + (t.y - ay) * (by - ay)) / len;
            if (s < CORNER_KEEP_M || s > len - CORNER_KEEP_M) {
                continue;
            }
            double fx = ax + (bx - ax) * s / len;
            double fy = ay + (by - ay) * s / len;
            double nx = -(by - ay) / len;
            double ny = (bx - ax) / len;
            if (contains(test, fx + nx * 0.4, fy + ny * 0.4)) {
                nx = -nx;
                ny = -ny;
            }
            double[] c = ray(space, t, fx, fy, nx, ny);
            if (c != null) {
                out.add(c);
            }
        }
    }

    /** Корпус не найден: выходы по 16 направлениям, первая свободная точка. */
    private static void radial(FreeSpace space, Terminal t, List<double[]> out) {
        if (space.nodeFree(t.x, t.y)) {
            out.add(new double[]{t.x, t.y, t.x, t.y, 0, space.violation(t.x, t.y, t.x, t.y, t.host)});
            return;
        }
        for (int k = 0; k < 16; k++) {
            double a = Math.PI * 2 * k / 16;
            double[] c = ray(space, t, t.x, t.y, Math.cos(a), Math.sin(a));
            if (c != null) {
                out.add(c);
            }
        }
    }

    private static double[] ray(FreeSpace space, Terminal t, double fx, double fy, double nx, double ny) {
        for (double d = STEP_M; d <= RAY_MAX_M; d += STEP_M) {
            double qx = fx + nx * d;
            double qy = fy + ny * d;
            if (space.nodeFree(qx, qy)) {
                qx += nx * STEP_M;
                qy += ny * STEP_M;
                if (!space.nodeFree(qx, qy)) {
                    continue;
                }
                double indoor = Geo.dist(t.x, t.y, fx, fy);
                double v = space.violation(t.x, t.y, qx, qy, t.host);
                return new double[]{qx, qy, fx, fy, indoor, v};
            }
        }
        return null;
    }

    private static boolean contains(Polygon p, double x, double y) {
        return p.covers(GeoJsonGeometries.GF.createPoint(new Coordinate(x, y)));
    }

    int portsOf(int term, int[] out) {
        Terminal t = terms.get(term);
        for (int i = 0; i < t.count; i++) {
            out[i] = t.first + i;
        }
        return t.count;
    }
}
