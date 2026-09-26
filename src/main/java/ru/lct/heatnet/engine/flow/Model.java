package ru.lct.heatnet.engine.flow;

import java.util.ArrayList;
import java.util.List;

/**
 * Точная стоимость леса по разделам 3, 7 и 8 приложения за O(узлы + врезки · глубина цепочки).
 * <p>
 * Расход участка — сумма расходов ОКС ниже по дереву; DN — минимальный по пропускной способности,
 * затем подъём, если непрерывная часть одного DN (через камеры без смены DN) длиннее предельной.
 * Камера разветвления стоит по наибольшему DN примыкающих участков; новая камера врезки на участке —
 * с учётом DN существующего участка после реконструкции. Каждая труба от существующей сети — врезка.
 */
final class Model {

    static final class Eval {
        double pipes;
        double chambers;
        double taps;
        double recon;
        double reconChambers;
        double penalty;
        double total;
        double objective;
        double length;
        int tieIns;
        int unconnected;
    }

    final Prices prices;
    final ExistingNet net;
    final Ports ports;
    /** Добавка к цели за каждую врезку сверх первой (режим «минимум врезок»). */
    double extraTapWeight;
    /** Множитель к стоимости реконструкции в цели (режим «минимум реконструкции»). */
    double reconWeight;

    private final List<ExistingNet.Tap> tapList = new ArrayList<>();
    private final List<Forest.Node> tapNodes = new ArrayList<>();
    private final List<Forest.Node> order = new ArrayList<>();
    private boolean[] connected = new boolean[0];

    Model(Prices prices, ExistingNet net, Ports ports) {
        this.prices = prices;
        this.net = net;
        this.ports = ports;
    }

    Eval evaluate(Forest f) {
        Eval e = new Eval();
        int nt = ports.terms.size();
        if (connected.length != nt) {
            connected = new boolean[nt];
        }
        java.util.Arrays.fill(connected, false);
        tapList.clear();
        tapNodes.clear();
        for (Forest.Node t : f.nodes) {
            if (t.type != Forest.TAP || t.kids.isEmpty()) {
                continue;
            }
            order.clear();
            Forest.subtree(t, order);
            for (int i = order.size() - 1; i >= 1; i--) {
                Forest.Node v = order.get(i);
                double flow = 0;
                if (v.type == Forest.LEAF) {
                    flow = ports.terms.get(v.term).flow;
                    connected[v.term] = true;
                }
                for (Forest.Node k : v.kids) {
                    flow += k.flow;
                }
                v.flow = flow;
                sizeAndRun(v);
                e.pipes += (v.len + v.extra) * prices.perM(v.dn);
                e.length += v.len;
                if (v.type == Forest.JUNC && v.kids.size() >= 2) {
                    int max = v.dn;
                    for (Forest.Node k : v.kids) {
                        max = Math.max(max, k.dn);
                    }
                    e.chambers += prices.chamber(max);
                }
            }
            double flow = 0;
            int max = 0;
            for (Forest.Node k : t.kids) {
                flow += k.flow;
                max = Math.max(max, k.dn);
            }
            t.flow = flow;
            t.dn = max;
            e.tieIns += t.kids.size();
            ExistingNet.Tap tap = new ExistingNet.Tap(t.chamber >= 0 ? -1 : t.seg, t.at, t.chamber, flow, max);
            tapList.add(tap);
            tapNodes.add(t);
        }
        e.taps = e.tieIns * prices.tap;
        ExistingNet.Recon r = net.recon(tapList, prices, false);
        e.recon = r.cost;
        e.reconChambers = r.chamberCost;
        for (int i = 0; i < tapList.size(); i++) {
            ExistingNet.Tap tap = tapList.get(i);
            if (tap.chamber < 0) {
                int d = Math.max(tap.newDn, Math.max(net.segs.get(tap.seg).dn, tap.requiredAbove));
                tapNodes.get(i).siteDn = d;
                e.chambers += prices.chamber(d);
            }
        }
        for (int i = 0; i < nt; i++) {
            if (!connected[i]) {
                e.penalty += prices.penalty(ports.terms.get(i).flow);
                e.unconnected++;
            }
        }
        e.total = e.pipes + e.chambers + e.taps + e.recon + e.reconChambers + e.penalty;
        e.objective = e.total + extraTapWeight * Math.max(0, e.tieIns - 1)
                + reconWeight * (e.recon + e.reconChambers);
        return e;
    }

    /**
     * DN по расходу и подъём по предельной длине: отсчёт идёт от листьев вверх и продолжается
     * через камеру, если DN не меняется (п. 3).
     */
    private void sizeAndRun(Forest.Node v) {
        int dn = prices.dnFor(v.flow);
        int guard = prices.dn.length;
        while (true) {
            double childRun = 0;
            for (Forest.Node k : v.kids) {
                if (k.dn == dn) {
                    childRun = Math.max(childRun, k.run);
                }
            }
            double run = childRun + v.len;
            if (run <= prices.maxRun(dn) + 1e-6 || guard-- <= 0 || prices.bump(dn) == dn) {
                v.dn = dn;
                v.run = run;
                return;
            }
            dn = prices.bump(dn);
        }
    }

    /** Штраф за ОКС поддерева, если его не подключить (для базы при переподвешивании). */
    double penaltyOf(Forest.Node x) {
        List<Forest.Node> sub = new ArrayList<>();
        Forest.subtree(x, sub);
        double p = 0;
        for (Forest.Node n : sub) {
            if (n.type == Forest.LEAF) {
                p += prices.penalty(ports.terms.get(n.term).flow);
            }
        }
        return p;
    }

    double flowOf(Forest.Node x) {
        List<Forest.Node> sub = new ArrayList<>();
        Forest.subtree(x, sub);
        double f = 0;
        for (Forest.Node n : sub) {
            if (n.type == Forest.LEAF) {
                f += ports.terms.get(n.term).flow;
            }
        }
        return f;
    }

    /** Текущий список врезок для расчёта приращения реконструкции. */
    List<ExistingNet.Tap> currentTaps() {
        List<ExistingNet.Tap> out = new ArrayList<>(tapList.size());
        for (ExistingNet.Tap t : tapList) {
            out.add(new ExistingNet.Tap(t.seg, t.at, t.chamber, t.flow, t.newDn));
        }
        return out;
    }

    List<Forest.Node> currentTapNodes() {
        return new ArrayList<>(tapNodes);
    }
}
