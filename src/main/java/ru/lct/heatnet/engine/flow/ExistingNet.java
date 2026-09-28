package ru.lct.heatnet.engine.flow;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.HeatSource;
import ru.lct.heatnet.scene.Scene;

/**
 * Существующая сеть как дерево к источнику. Конец участка к источнику берётся из upstream_object_id
 * или стыкуется в окне 4 м. Ломаная хранится от верхнего конца к нижнему, поэтому позиция врезки —
 * длина от верхнего конца. Живой расчёт использует участки и камеры для правила 10 м и четырёх
 * примыканий. Реконструкция существующей сети в смету не входит.
 */
final class ExistingNet {

    static final double JOIN_M = 4.0;

    final List<Seg> segs = new ArrayList<>();
    final List<Cham> chambers = new ArrayList<>();
    final Map<String, Integer> segIndex = new HashMap<>();
    final Map<String, Integer> chamberIndex = new HashMap<>();
    final java.util.Set<String> sources = new java.util.HashSet<>();
    /** Прямые куски всех участков: {x1,y1,x2,y2} + (участок, позиция начала куска от верхнего конца). */
    double[] pieces = new double[0];
    int[] pieceSeg = new int[0];
    double[] pieceAt = new double[0];

    static final class Seg {
        int index;
        String id;
        int dn;
        double flow;
        /** Ломаная от конца к источнику до нижнего конца. */
        double[] line;
        double length;
        /** Следующий объект к источнику: индекс участка (>=0), камеры (-2 - idx) или -1 — источник/нет. */
        int next;
        String nextId;
        final List<Integer> chambersAtEnds = new ArrayList<>();
    }

    static final class Cham {
        int index;
        String id;
        double x;
        double y;
        int dn;
        int incident;
        int next;
        String nextId;
        final List<Integer> adjacent = new ArrayList<>();
    }

    static final int NONE = -1;

    static int chamberRef(int idx) {
        return -2 - idx;
    }

    static int chamberOf(int ref) {
        return -2 - ref;
    }

    ExistingNet(Scene scene) {
        for (HeatSource s : scene.sources) {
            sources.add(s.id);
        }
        for (ExistingSegment s : scene.segments) {
            if (s.line == null || s.line.getNumPoints() < 2) {
                continue;
            }
            Seg g = new Seg();
            g.index = segs.size();
            g.id = s.id;
            g.dn = s.dn;
            g.flow = s.existingFlowTph;
            g.nextId = s.nextId;
            g.line = FreeSpace.coords(s.line.getCoordinates());
            g.length = Geo.length(g.line);
            segs.add(g);
            segIndex.put(g.id, g.index);
        }
        for (Chamber c : scene.chambers) {
            if (c.point == null) {
                continue;
            }
            Cham h = new Cham();
            h.index = chambers.size();
            h.id = c.id;
            h.x = c.point.getX();
            h.y = c.point.getY();
            h.dn = c.dn;
            h.incident = c.incidentCount;
            h.nextId = c.nextId;
            chambers.add(h);
            chamberIndex.put(h.id, h.index);
        }
        for (Seg g : segs) {
            g.next = resolve(g.nextId);
        }
        for (Cham h : chambers) {
            h.next = resolve(h.nextId);
        }
        Map<String, Coordinate> sourceAt = new HashMap<>();
        for (HeatSource s : scene.sources) {
            sourceAt.put(s.id, s.point.getCoordinate());
        }
        for (Seg g : segs) {
            orient(g, sourceAt);
        }
        for (Cham h : chambers) {
            int incident = 0;
            int maxDn = 0;
            for (Seg g : segs) {
                int n = g.line.length;
                boolean atStart = Geo.dist(g.line[0], g.line[1], h.x, h.y) <= JOIN_M;
                boolean atEnd = Geo.dist(g.line[n - 2], g.line[n - 1], h.x, h.y) <= JOIN_M;
                if (atStart || atEnd) {
                    incident++;
                    h.adjacent.add(g.index);
                    g.chambersAtEnds.add(h.index);
                    maxDn = Math.max(maxDn, g.dn);
                }
            }
            if (h.incident <= 0) {
                h.incident = incident;
            }
            if (h.dn <= 0) {
                h.dn = maxDn;
            }
        }
        buildPieces();
    }

    private int resolve(String id) {
        if (id == null || id.isBlank() || sources.contains(id)) {
            return NONE;
        }
        Integer s = segIndex.get(id);
        if (s != null) {
            return s;
        }
        Integer c = chamberIndex.get(id);
        if (c != null) {
            return chamberRef(c);
        }
        return NONE;
    }

    /** Разворачиваем ломаную так, чтобы она начиналась у объекта по направлению к источнику. */
    private void orient(Seg g, Map<String, Coordinate> sourceAt) {
        double[] up = null;
        if (g.next >= 0) {
            Seg n = segs.get(g.next);
            up = nearestEnd(n.line, g.line);
        } else if (g.next < -1) {
            Cham c = chambers.get(chamberOf(g.next));
            up = new double[]{c.x, c.y};
        } else if (g.nextId != null && sourceAt.containsKey(g.nextId)) {
            Coordinate c = sourceAt.get(g.nextId);
            up = new double[]{c.x, c.y};
        }
        if (up == null) {
            return;
        }
        int n = g.line.length;
        double dStart = Geo.dist(g.line[0], g.line[1], up[0], up[1]);
        double dEnd = Geo.dist(g.line[n - 2], g.line[n - 1], up[0], up[1]);
        if (dEnd < dStart) {
            g.line = Geo.reverse(g.line);
        }
    }

    /** Конец ломаной other, ближайший к какому-либо концу line. */
    private static double[] nearestEnd(double[] other, double[] line) {
        int n = other.length;
        int m = line.length;
        double[][] ends = {{other[0], other[1]}, {other[n - 2], other[n - 1]}};
        double best = Double.POSITIVE_INFINITY;
        double[] pick = ends[0];
        for (double[] e : ends) {
            double d = Math.min(Geo.dist(e[0], e[1], line[0], line[1]), Geo.dist(e[0], e[1], line[m - 2], line[m - 1]));
            if (d < best) {
                best = d;
                pick = e;
            }
        }
        return pick;
    }

    private void buildPieces() {
        int count = 0;
        for (Seg g : segs) {
            count += g.line.length / 2 - 1;
        }
        pieces = new double[count * 4];
        pieceSeg = new int[count];
        pieceAt = new double[count];
        int k = 0;
        for (Seg g : segs) {
            double acc = 0;
            for (int i = 0; i + 3 < g.line.length; i += 2) {
                pieces[k * 4] = g.line[i];
                pieces[k * 4 + 1] = g.line[i + 1];
                pieces[k * 4 + 2] = g.line[i + 2];
                pieces[k * 4 + 3] = g.line[i + 3];
                pieceSeg[k] = g.index;
                pieceAt[k] = acc;
                acc += Geo.dist(g.line[i], g.line[i + 1], g.line[i + 2], g.line[i + 3]);
                k++;
            }
        }
    }

    int pieceCount() {
        return pieceSeg.length;
    }

    /** Врезка для расчёта реконструкции: участок и позиция от верхнего конца, либо камера. */
    static final class Tap {
        int seg = -1;
        double at;
        int chamber = -1;
        double flow;
        /** DN новой трубы в точке врезки (для камеры). */
        int newDn;
        /** Выход: DN существующего участка сразу выше врезки на участке после реконструкции. */
        int requiredAbove;

        Tap() {
        }

        Tap(int seg, double at, int chamber, double flow, int newDn) {
            this.seg = seg;
            this.at = at;
            this.chamber = chamber;
            this.flow = flow;
            this.newDn = newDn;
        }
    }

    /** Итог реконструкции для набора врезок. */
    static final class Recon {
        double cost;
        double length;
        double chamberCost;
        /** {seg, from, to, added, required} — от верхнего конца участка. */
        final List<double[]> parts = new ArrayList<>();
        /** {chamber, requiredDn}. */
        final List<int[]> chamberParts = new ArrayList<>();
    }

    private double[] full = new double[0];
    private int[] reqAtLower = new int[0];
    private int[] reqAtUpper = new int[0];
    private final List<List<Tap>> partial = new ArrayList<>();
    private final List<Integer> touched = new ArrayList<>();

    /**
     * Прежний расчёт реконструкции. Сервис его не вызывает: актуальное приложение реконструкцию
     * не выполняет. Метод оставлен для старых тестов.
     */
    Recon recon(List<Tap> taps, Prices prices, boolean collect) {
        Recon r = new Recon();
        if (taps.isEmpty()) {
            return r;
        }
        int ns = segs.size();
        if (full.length != ns) {
            full = new double[ns];
            reqAtLower = new int[ns];
            reqAtUpper = new int[ns];
            partial.clear();
            for (int i = 0; i < ns; i++) {
                partial.add(null);
            }
        }
        java.util.Arrays.fill(full, 0);
        java.util.Arrays.fill(reqAtLower, 0);
        java.util.Arrays.fill(reqAtUpper, 0);
        touched.clear();
        for (Tap t : taps) {
            int cur;
            t.requiredAbove = 0;
            if (t.chamber >= 0) {
                cur = chambers.get(t.chamber).next;
            } else {
                List<Tap> list = partial.get(t.seg);
                if (list == null) {
                    list = new ArrayList<>(2);
                    partial.set(t.seg, list);
                }
                if (list.isEmpty()) {
                    touched.add(t.seg);
                }
                list.add(t);
                cur = segs.get(t.seg).next;
            }
            int guard = ns + chambers.size() + 2;
            while (cur != NONE && guard-- > 0) {
                if (cur >= 0) {
                    full[cur] += t.flow;
                    cur = segs.get(cur).next;
                } else {
                    cur = chambers.get(chamberOf(cur)).next;
                }
            }
        }
        for (Seg g : segs) {
            List<Tap> list = partial.get(g.index);
            boolean hasPartial = list != null && !list.isEmpty();
            if (full[g.index] <= 1e-12 && !hasPartial) {
                continue;
            }
            if (!hasPartial) {
                int req = prices.dnFor(g.flow + full[g.index]);
                reqAtLower[g.index] = req;
                reqAtUpper[g.index] = req;
                if (req > g.dn) {
                    r.cost += g.length * prices.reconPerM(req);
                    r.length += g.length;
                    if (collect) {
                        r.parts.add(new double[]{g.index, 0, g.length, full[g.index], req});
                    }
                }
                continue;
            }
            list.sort((a, b) -> Double.compare(a.at, b.at));
            double from = 0;
            double pending = full[g.index];
            for (Tap p : list) {
                pending += p.flow;
            }
            int idx = 0;
            while (from < g.length - 1e-9) {
                double to = g.length;
                while (idx < list.size() && list.get(idx).at <= from + 1e-9) {
                    pending -= list.get(idx).flow;
                    idx++;
                }
                if (idx < list.size()) {
                    to = Math.min(to, list.get(idx).at);
                }
                int req = prices.dnFor(g.flow + pending);
                int after = Math.max(req, g.dn);
                for (int k = idx; k < list.size() && list.get(k).at <= to + 1e-9; k++) {
                    list.get(k).requiredAbove = after;
                }
                if (from <= 1e-9) {
                    reqAtUpper[g.index] = req;
                }
                if (to >= g.length - 1e-9) {
                    reqAtLower[g.index] = req;
                }
                if (req > g.dn && to > from) {
                    r.cost += (to - from) * prices.reconPerM(req);
                    r.length += to - from;
                    if (collect) {
                        r.parts.add(new double[]{g.index, from, to, pending, req});
                    }
                }
                from = to;
            }
        }
        for (int s : touched) {
            partial.get(s).clear();
        }
        for (Tap t : taps) {
            if (t.chamber < 0) {
                continue;
            }
            boolean counted = false;
            for (int[] cp : r.chamberParts) {
                if (cp[0] == t.chamber) {
                    counted = true;
                    break;
                }
            }
            if (counted) {
                continue;
            }
            Cham h = chambers.get(t.chamber);
            int req = 0;
            for (Tap u : taps) {
                if (u.chamber == t.chamber) {
                    req = Math.max(req, u.newDn);
                }
            }
            for (int si : h.adjacent) {
                Seg g = segs.get(si);
                boolean upperEnd = Geo.dist(g.line[0], g.line[1], h.x, h.y) <= JOIN_M;
                int after = upperEnd ? reqAtUpper[si] : reqAtLower[si];
                req = Math.max(req, Math.max(g.dn, after));
            }
            r.chamberParts.add(new int[]{t.chamber, req > h.dn ? req : -1});
            if (req > h.dn) {
                r.chamberCost += prices.chamber(req);
            }
        }
        r.chamberParts.removeIf(cp -> cp[1] < 0);
        return r;
    }

    /** DN существующего участка в точке новой камеры. */
    int segDn(int seg) {
        return segs.get(seg).dn;
    }
}
