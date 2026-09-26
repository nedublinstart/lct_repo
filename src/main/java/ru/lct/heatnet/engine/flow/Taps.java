package ru.lct.heatnet.engine.flow;

import java.util.ArrayList;
import java.util.List;

/**
 * Места врезки, видимые из точки плоскости: существующие камеры со свободным местом и основания
 * перпендикуляров на куски существующих участков. По п. 8.2 врезка ближе 10 м к камере со свободным
 * местом выполняется в этой камере, поэтому такие основания не предлагаются. Последний отрезок до
 * существующей сети проверяется точно по исходным контурам: камера может стоять ближе к зданию, чем
 * упрощённая граница запретов.
 */
final class Taps {

    static final double CHAMBER_RULE_M = 10.0;
    static final int MAX_DEGREE = 4;
    /** Новая камера на участке: два куска существующего участка плюс до двух новых. */
    static final int PIPE_SLOTS = 2;
    private static final double RANGE_M = 250.0;
    private static final double END_KEEP_M = 0.5;
    private static final Option[] NONE = new Option[0];

    static final class Option {
        final int chamber;
        final int seg;
        final double at;
        final double x;
        final double y;
        /** Длина последнего отрезка до места врезки плюс надбавка спецпроходов, м. */
        final double leg;

        Option(int chamber, int seg, double at, double x, double y, double leg) {
            this.chamber = chamber;
            this.seg = seg;
            this.at = at;
            this.x = x;
            this.y = y;
            this.leg = leg;
        }
    }

    final ExistingNet net;
    final FreeSpace space;
    final VisGraph g;
    /** Свободные места существующих камер: 4 минус уже примыкающие участки. */
    final int[] slots;
    private final Option[][] cache;

    Taps(ExistingNet net, FreeSpace space, VisGraph g) {
        this.net = net;
        this.space = space;
        this.g = g;
        slots = new int[net.chambers.size()];
        for (ExistingNet.Cham c : net.chambers) {
            slots[c.index] = Math.max(0, MAX_DEGREE - Math.max(c.incident, c.adjacent.size()));
        }
        cache = new Option[g.n][];
    }

    Option[] from(int v) {
        Option[] o = cache[v];
        if (o == null) {
            o = compute(g.x[v], g.y[v]);
            cache[v] = o;
        }
        return o;
    }

    Option[] fromPoint(double x, double y) {
        return compute(x, y);
    }

    private Option[] compute(double x, double y) {
        List<Option> out = new ArrayList<>();
        for (ExistingNet.Cham c : net.chambers) {
            if (slots[c.index] <= 0 || Geo.dist(x, y, c.x, c.y) > RANGE_M) {
                continue;
            }
            double leg = legTo(x, y, c.x, c.y);
            if (!Double.isNaN(leg)) {
                out.add(new Option(c.index, -1, 0, c.x, c.y, leg));
            }
        }
        double[] p = net.pieces;
        Option[] bestPerSeg = new Option[net.segs.size()];
        for (int k = 0; k < net.pieceCount(); k++) {
            int o = k * 4;
            double ax = p[o];
            double ay = p[o + 1];
            double bx = p[o + 2];
            double by = p[o + 3];
            double plen = Geo.dist(ax, ay, bx, by);
            if (plen < 1e-6) {
                continue;
            }
            double t = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (plen * plen);
            if (t <= 1e-3 || t >= 1 - 1e-3) {
                continue;
            }
            double fx = ax + t * (bx - ax);
            double fy = ay + t * (by - ay);
            double d = Geo.dist(x, y, fx, fy);
            int s = net.pieceSeg[k];
            if (d > RANGE_M || (bestPerSeg[s] != null && bestPerSeg[s].leg <= d)) {
                continue;
            }
            double at = net.pieceAt[k] + t * plen;
            ExistingNet.Seg seg = net.segs.get(s);
            if (at < END_KEEP_M || at > seg.length - END_KEEP_M || nearFreeChamber(fx, fy)) {
                continue;
            }
            double leg = legTo(x, y, fx, fy);
            if (Double.isNaN(leg)) {
                continue;
            }
            if (bestPerSeg[s] == null || leg < bestPerSeg[s].leg) {
                bestPerSeg[s] = new Option(-1, s, at, fx, fy, leg);
            }
        }
        for (Option o : bestPerSeg) {
            if (o != null) {
                out.add(o);
            }
        }
        return out.isEmpty() ? NONE : out.toArray(NONE);
    }

    private boolean nearFreeChamber(double x, double y) {
        for (ExistingNet.Cham c : net.chambers) {
            if (slots[c.index] > 0 && Geo.dist(x, y, c.x, c.y) <= CHAMBER_RULE_M) {
                return true;
            }
        }
        return false;
    }

    /** Отрезок до точки на существующей сети: точная проверка расстояний и угол выхода. */
    double legTo(double ax, double ay, double tx, double ty) {
        if (!space.segmentFree(ax, ay, tx, ty) && space.violation(ax, ay, tx, ty, -1) > 1e-6) {
            return Double.NaN;
        }
        double extra = space.specialExtra(ax, ay, tx, ty, false, true);
        if (Double.isNaN(extra)) {
            return Double.NaN;
        }
        return Geo.dist(ax, ay, tx, ty) + extra;
    }

    /** Мест под новые трубы у места врезки. */
    int capacity(int chamber) {
        return chamber >= 0 ? slots[chamber] : PIPE_SLOTS;
    }
}
