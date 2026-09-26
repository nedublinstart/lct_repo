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
 * Лес → вариант с объектами раздела 10.
 * <p>
 * Ребро дерева режется на участки по границам специальных проходов, в точке смены способа прокладки —
 * технический узел. Камера разветвления — новая камера. У корня на каждую приходящую трубу своя точка
 * врезки; врезка в участок сети получает новую камеру в той же точке. Внутри варианта {@code fromId} —
 * конец со стороны ОКС, {@code toId} — со стороны сети, геометрия идёт от {@code fromId}.
 * DN, расходы и надбавки берутся из {@link Model}, поэтому смета варианта совпадает с целью поиска.
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
                chamber(v, t.x, t.y, t.siteDn, true);
            }
            for (Forest.Node k : t.kids) {
                up[k.id] = tieIn(v, t, k);
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
        double[] path = n.path;
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

    private void piece(Variant v, Forest.Node n, double[] line, String from, String to, double k) {
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

    private String tieIn(Variant v, Forest.Node t, Forest.Node k) {
        ExistingNet net = model.net;
        TapPoint p = new TapPoint();
        p.id = "TI-" + seq++;
        p.nodeId = p.id;
        p.geometryMeters = point(t.x, t.y);
        if (t.chamber >= 0) {
            ExistingNet.Cham c = net.chambers.get(t.chamber);
            p.existingObjectId = c.id;
            p.existingObjectKind = "heat_chamber";
            p.existingDiameter = c.dn;
        } else {
            ExistingNet.Seg s = net.segs.get(t.seg);
            p.existingObjectId = s.id;
            p.existingObjectKind = "heat_network";
            p.existingDiameter = s.dn;
        }
        p.requiredDiameter = k.dn;
        p.extraFlowTph = k.flow;
        p.cost = model.prices.tap;
        v.taps.add(p);
        return p.id;
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
