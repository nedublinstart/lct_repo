package ru.lct.heatnet.engine.flow;

import java.util.ArrayList;
import java.util.List;

/**
 * Точная стоимость леса по актуальному техническому приложению.
 * <p>
 * Расход участка — сумма расходов точек ниже по дереву. DN — минимальный, который одновременно
 * проходит по расходу и по предельной длине непрерывного пути одного DN. По пути от точки
 * подключения к месту присоединения DN не уменьшается. Камера без смены DN отсчёт не сбрасывает,
 * и DN не меняется только ради нового отсчёта: поднимается диаметр всего ребра.
 * Новая камера на участке уже включает присоединение. Врезка 5 млн ₽ берётся только с трубы,
 * которая заканчивается в существующей камере. Реконструкция существующей сети не считается.
 * Цель поиска совпадает с показателем S: к полной стоимости добавлена длина в рублях за метр.
 * Неподключённый ОКС в цели дороже любой связной сметы. В файл пишется штраф раздела 6.
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

    /**
     * Сколько рублей цели приходится на метр длины, чтобы минимум цели совпадал с минимумом
     * S = 0,7·C/25e6 + 0,3·L/100.
     */
    static final double SCORE_LENGTH_RUB_PER_M = (0.3 / 100.0) / (0.7 / 25_000_000.0);

    final Prices prices;
    final ExistingNet net;
    final Ports ports;
    /** Добавка к цели за каждую врезку сверх первой (режим «минимум врезок»). */
    double extraTapWeight;
    /** Рубли за метр в цели. По умолчанию ровно вес длины в S. */
    double lengthPrice = SCORE_LENGTH_RUB_PER_M;
    /** Дополнительный вес метра. В режиме минимальной длины он больше цены камеры. */
    double lengthBoost;
    /**
     * Добавка цели за каждый неподключённый ОКС. Больше любой связной сметы, поэтому найденный
     * маршрут не отбрасывается ради показателя. В файл и в S пишется штраф раздела 6, не эта добавка.
     */
    static final double MUST_CONNECT = 1.0e15;

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
            }
            for (int i = order.size() - 1; i >= 1; i--) {
                sizeAndRun(order.get(i));
            }
            for (int i = order.size() - 1; i >= 1; i--) {
                Forest.Node v = order.get(i);
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
            if (t.chamber >= 0) {
                e.taps += t.kids.size() * prices.tap;
            } else {
                int d = Math.max(max, net.segs.get(t.seg).dn);
                t.siteDn = d;
                e.chambers += prices.chamber(d);
            }
        }
        for (int i = 0; i < nt; i++) {
            if (!connected[i]) {
                e.penalty += prices.penalty(ports.terms.get(i).flow);
                e.unconnected++;
            }
        }
        e.total = e.pipes + e.chambers + e.taps + e.penalty;
        e.objective = e.total + (lengthPrice + lengthBoost) * e.length
                + extraTapWeight * Math.max(0, e.tieIns - 1)
                + MUST_CONNECT * e.unconnected;
        return e;
    }

    /**
     * Минимальный DN, который проходит и по расходу, и по предельной длине непрерывного пути.
     * DN не меньше DN ниже по дереву: к месту присоединения диаметр не уменьшается.
     */
    private void sizeAndRun(Forest.Node v) {
        int dn = prices.dnFor(v.flow);
        for (Forest.Node k : v.kids) {
            if (k.dn > dn) {
                dn = k.dn;
            }
        }
        int guard = prices.dn.length;
        while (true) {
            applyDn(v, dn);
            if (v.run <= prices.maxRun(dn) + 1e-6 || guard-- <= 0 || prices.bump(dn) == dn) {
                return;
            }
            dn = prices.bump(dn);
        }
    }

    /** Один DN на непрерывном участке того же расхода. Отсчёт сбрасывается только при смене DN. */
    private void applyDn(Forest.Node v, int dn) {
        v.dn = dn;
        double childRun = 0;
        for (Forest.Node k : v.kids) {
            if (Math.abs(k.flow - v.flow) < 1e-6) {
                if (k.dn != dn) {
                    applyDn(k, dn);
                }
                childRun = Math.max(childRun, k.run);
            } else if (k.dn == dn) {
                childRun = Math.max(childRun, k.run);
            }
        }
        v.run = childRun + v.len;
    }

    /** Сколько точек подключения в поддереве. */
    int leavesOf(Forest.Node x) {
        List<Forest.Node> sub = new ArrayList<>();
        Forest.subtree(x, sub);
        int n = 0;
        for (Forest.Node node : sub) {
            if (node.type == Forest.LEAF) {
                n++;
            }
        }
        return n;
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
