package ru.lct.heatnet.engine.flow;

import java.util.List;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.geo.GeoJsonGeometries;

/**
 * Лес → вариант по разделу 7 актуального приложения.
 * <p>
 * Ребро дерева режется на участки по границам специальных проходов, в точке смены способа прокладки —
 * технический узел. Разветвление — новая камера. Присоединение к участку существующей сети — новая
 * камера в точке врезки, её стоимость уже включает присоединение. Присоединение к существующей камере
 * заканчивает трубу в ней: отдельный объект врезки не создаётся, в смете это 5 млн ₽ за каждую
 * приходящую трубу. Внутри варианта {@code fromId} — конец со стороны точки подключения, {@code toId} —
 * со стороны сети. DN и расходы берутся из {@link Model}.
 */
final class Emitter {

    private final Model model;
    private final FreeSpace space;
    private int seq = 1;

    Emitter(Model model, FreeSpace space) {
        this.model = model;
        this.space = space;
    }

    Variant emit(Forest f) {
        model.evaluate(f);
        Variant v = new Variant();
        int maxId = 0;
        for (Forest.Node n : f.nodes) {
            maxId = Math.max(maxId, n.id);
        }
        String[] own = new String[maxId + 1];
        String[] up = new String[maxId + 1];
        boolean[] connected = new boolean[model.ports.terms.size()];
        for (Forest.Node n : f.nodes) {
            if (n.type == Forest.LEAF) {
                own[n.id] = model.ports.terms.get(n.term).id;
                connected[n.term] = attached(n);
            } else if (n.type == Forest.JUNC) {
                own[n.id] = n.kids.size() >= 2 ? chamber(v, n.x, n.y, junctionDn(n), false) : technical(v, n.x, n.y);
            }
        }
        for (Forest.Node t : f.nodes) {
            if (t.type != Forest.TAP || t.kids.isEmpty()) {
                continue;
            }
            if (t.chamber < 0) {
                String ch = chamber(v, t.x, t.y, t.siteDn, true);
                for (Forest.Node k : t.kids) {
                    up[k.id] = ch;
                }
            } else {
                ExistingNet.Cham c = model.net.chambers.get(t.chamber);
                for (Forest.Node k : t.kids) {
                    up[k.id] = c.id;
                    existingTie(v, t, k, c);
                }
            }
        }
        for (Forest.Node n : f.nodes) {
            if (n.type == Forest.TAP || n.parent == null || n.path == null || !attached(n)) {
                continue;
            }
            String to = up[n.id] != null ? up[n.id] : own[n.parent.id];
            edge(v, n, own[n.id], to, n.parent.type == Forest.TAP);
        }
        for (int i = 0; i < connected.length; i++) {
            if (!connected[i]) {
                Ports.Terminal term = model.ports.terms.get(i);
                v.unconnectedOks.add(term.id);
                v.unconnectedFlows.put(term.id, term.flow);
                v.notes.add("Маршрут не найден для ОКС " + term.id);
            }
        }
        return v;
    }

    private static boolean attached(Forest.Node n) {
        Forest.Node u = n;
        while (u.parent != null) {
            u = u.parent;
        }
        return u.type == Forest.TAP;
    }

    private static int junctionDn(Forest.Node n) {
        int max = n.dn;
        for (Forest.Node k : n.kids) {
            max = Math.max(max, k.dn);
        }
        return max;
    }

    /** Участки ребра n → родитель: по одному на каждый интервал одного способа прокладки. */
    private void edge(Variant v, Forest.Node n, String from, String to, boolean tapEnd) {
        double[] path = openTurns(n.path);
        double total = Geo.length(path);
        List<double[]> spans = space.pathSpans(path, false, tapEnd);
        double pos = 0;
        String cur = from;
        int si = 0;
        while (pos < total - 1e-9) {
            double end;
            double k;
            if (spans != null && si < spans.size() && spans.get(si)[0] <= pos + 1e-9) {
                end = spans.get(si)[1];
                k = spans.get(si)[2];
                si++;
            } else {
                end = spans != null && si < spans.size() ? spans.get(si)[0] : total;
                k = 1.0;
            }
            end = Math.min(end, total);
            String next = end >= total - 1e-9 ? to : technical(v, Geo.at(path, end));
            piece(v, n, Geo.slice(path, pos, end), cur, next, k);
            cur = next;
            pos = end;
        }
    }

    /**
     * Специальный проход — один прямой участок. Если окно коэффициента захватило излом ломаной,
     * каждый прямой кусок пишется отдельным участком с тем же Kспец, на изломе — технический узел.
     */
    private void piece(Variant v, Forest.Node n, double[] line, String from, String to, double k) {
        if (k > 1 + 1e-12 && bends(line)) {
            String cur = from;
            int legs = line.length / 2 - 1;
            for (int i = 0; i < legs; i++) {
                double x = line[(i + 1) * 2];
                double y = line[(i + 1) * 2 + 1];
                String next = i == legs - 1 ? to : technical(v, x, y);
                write(v, n, new double[]{line[i * 2], line[i * 2 + 1], x, y}, cur, next, k);
                cur = next;
            }
            return;
        }
        write(v, n, line, from, to, k);
    }

    private static boolean bends(double[] line) {
        int n = line.length / 2;
        for (int i = 1; i < n - 1; i++) {
            if (Geo.deflection(line[(i - 1) * 2], line[(i - 1) * 2 + 1], line[i * 2], line[i * 2 + 1],
                    line[(i + 1) * 2], line[(i + 1) * 2 + 1]) > 1.0) {
                return true;
            }
        }
        return false;
    }

    private void write(Variant v, Forest.Node n, double[] line, String from, String to, double k) {
        NewSegment s = new NewSegment();
        s.id = "NS-" + seq++;
        s.geometryMeters = line(line);
        s.lengthM = Geo.length(line);
        s.flowTph = n.flow;
        s.dn = n.dn;
        s.kSpec = k;
        s.layingMethod = k > 1 + 1e-12 ? "special" : "base";
        s.fromId = from;
        s.toId = to;
        v.segments.add(s);
    }

    /** Врезка в существующую камеру: в выгрузке отдельной точки нет, в смете — 5 млн ₽ за трубу. */
    private void existingTie(Variant v, Forest.Node t, Forest.Node k, ExistingNet.Cham c) {
        TapPoint p = new TapPoint();
        p.id = "TI-" + seq++;
        p.nodeId = c.id;
        p.geometryMeters = point(t.x, t.y);
        p.existingObjectId = c.id;
        p.existingObjectKind = "heat_chamber";
        p.existingDiameter = c.dn;
        p.requiredDiameter = k.dn;
        p.extraFlowTph = k.flow;
        p.cost = model.prices.tap;
        v.taps.add(p);
    }

    private String chamber(Variant v, double x, double y, int dn, boolean atTap) {
        NewChamber c = new NewChamber();
        c.id = "CH-" + seq++;
        c.geometryMeters = point(x, y);
        c.dn = dn;
        c.atTap = atTap;
        v.chambers.add(c);
        return c.id;
    }

    private String technical(Variant v, double x, double y) {
        TechnicalNode t = new TechnicalNode();
        t.id = "TN-" + seq++;
        t.geometryMeters = point(x, y);
        v.technicalNodes.add(t);
        return t.id;
    }

    /**
     * Поворот больше 90° режется хордой, пока оба новых угла не станут не круче 90°.
     * Хорда обязана лежать в свободном пространстве.
     */
    private double[] openTurns(double[] path) {
        double[] cur = path;
        for (int guard = 0; guard < 4; guard++) {
            double[] next = openOnce(cur);
            if (next == cur) {
                return cur;
            }
            cur = next;
        }
        return cur;
    }

    private double[] openOnce(double[] path) {
        int n = path.length / 2;
        if (n < 3) {
            return path;
        }
        double[] out = new double[path.length * 2];
        int m = 0;
        m = put(out, m, path[0], path[1]);
        boolean changed = false;
        for (int i = 1; i < n - 1; i++) {
            double ax = path[(i - 1) * 2];
            double ay = path[(i - 1) * 2 + 1];
            double bx = path[i * 2];
            double by = path[i * 2 + 1];
            double cx = path[(i + 1) * 2];
            double cy = path[(i + 1) * 2 + 1];
            if (Geo.deflection(ax, ay, bx, by, cx, cy) <= 90.05) {
                m = put(out, m, bx, by);
                continue;
            }
            double lab = Geo.dist(ax, ay, bx, by);
            double lbc = Geo.dist(bx, by, cx, cy);
            double limit = 0.45 * Math.min(lab, lbc);
            boolean opened = false;
            if (lab > 1e-6 && lbc > 1e-6) {
                double uax = (ax - bx) / lab;
                double uay = (ay - by) / lab;
                double ucx = (cx - bx) / lbc;
                double ucy = (cy - by) / lbc;
                for (double t = 0.3; t <= limit + 1e-6; t = t < 4 ? t + 0.3 : t * 1.4) {
                    double px = bx + uax * t;
                    double py = by + uay * t;
                    double qx = bx + ucx * t;
                    double qy = by + ucy * t;
                    if (Geo.deflection(ax, ay, px, py, qx, qy) > 90.0
                            || Geo.deflection(px, py, qx, qy, cx, cy) > 90.0
                            || space.violation(px, py, qx, qy, -1) > 0.02) {
                        continue;
                    }
                    m = put(out, m, px, py);
                    m = put(out, m, qx, qy);
                    opened = true;
                    changed = true;
                    break;
                }
                if (!opened) {
                    double bn = Math.hypot(uax + ucx, uay + ucy);
                    if (bn > 1e-6) {
                        double ix = (uax + ucx) / bn;
                        double iy = (uay + ucy) / bn;
                        for (double d = 0.05; d <= 1.2; d += 0.05) {
                            double nx = bx + ix * d;
                            double ny = by + iy * d;
                            if (Geo.deflection(ax, ay, nx, ny, cx, cy) > 90.0) {
                                continue;
                            }
                            if (space.violation(ax, ay, nx, ny, -1) > 0.02
                                    || space.violation(nx, ny, cx, cy, -1) > 0.02) {
                                continue;
                            }
                            m = put(out, m, nx, ny);
                            opened = true;
                            changed = true;
                            break;
                        }
                    }
                }
            }
            if (!opened) {
                m = put(out, m, bx, by);
            }
        }
        m = put(out, m, path[path.length - 2], path[path.length - 1]);
        if (!changed) {
            return path;
        }
        double[] packed = new double[m];
        System.arraycopy(out, 0, packed, 0, m);
        return packed;
    }

    private static int put(double[] out, int n, double x, double y) {
        if (n >= 2 && Math.abs(out[n - 2] - x) < 1e-6 && Math.abs(out[n - 1] - y) < 1e-6) {
            return n;
        }
        if (n + 2 > out.length) {
            return n;
        }
        out[n++] = x;
        out[n++] = y;
        return n;
    }

    private String technical(Variant v, double[] at) {
        return technical(v, at[0], at[1]);
    }

    private static Point point(double x, double y) {
        return GeoJsonGeometries.GF.createPoint(new Coordinate(x, y));
    }

    private static LineString line(double[] xy) {
        Coordinate[] cs = new Coordinate[xy.length / 2];
        for (int i = 0; i < cs.length; i++) {
            cs[i] = new Coordinate(xy[i * 2], xy[i * 2 + 1]);
        }
        return GeoJsonGeometries.GF.createLineString(cs);
    }
}
