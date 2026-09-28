package ru.lct.heatnet.engine.flow;

import java.util.ArrayList;
import java.util.List;

/**
 * Решение: лес деревьев, корень каждого — место врезки на существующей сети. Узлы: ИТП (лист),
 * камера разветвления, место врезки. У каждого не корневого узла ломаная до родителя
 * ({@code path}: от узла к родителю). Массивы ломаных не меняются на месте, поэтому копия леса
 * делит их с оригиналом: O(узлы) памяти на копию.
 */
final class Forest {

    static final int LEAF = 0;
    static final int JUNC = 1;
    static final int TAP = 2;

    static final class Node {
        int id;
        int type;
        double x;
        double y;
        Node parent;
        final ArrayList<Node> kids = new ArrayList<>(3);
        double[] path;
        double len;
        /** Надбавка спецпроходов в метрах: (Kспец − 1) · длина спецучастка. */
        double extra;
        int term = -1;
        int chamber = -1;
        int seg = -1;
        double at;
        double flow;
        int dn;
        double run;
        /** Корень на участке сети: DN новой камеры, максимум из новой ветки и существующего участка. */
        int siteDn;

        boolean tapSite(int chamberIdx, int segIdx, double pos) {
            if (type != TAP) {
                return false;
            }
            if (chamberIdx >= 0) {
                return chamber == chamberIdx;
            }
            return chamber < 0 && seg == segIdx && Math.abs(at - pos) < 0.5;
        }
    }

    /** Надбавка спецпроходов для ломаной; tapEnd — последняя точка лежит на существующей сети. */
    interface Legs {
        double extra(double[] path, boolean tapEnd);
    }

    final ArrayList<Node> nodes = new ArrayList<>();
    private Node[] byId = new Node[64];
    private int nextId;
    /** Надбавки пересчитываются по целой ломаной: сумма надбавок частей не равна надбавке целого. */
    final Legs legs;

    Forest(Legs legs) {
        this.legs = legs;
    }

    Node add(int type, double x, double y) {
        Node n = new Node();
        n.id = nextId++;
        n.type = type;
        n.x = x;
        n.y = y;
        put(n);
        nodes.add(n);
        return n;
    }

    private void put(Node n) {
        if (n.id >= byId.length) {
            byId = java.util.Arrays.copyOf(byId, Math.max(byId.length * 2, n.id + 1));
        }
        byId[n.id] = n;
    }

    Node byId(int id) {
        return id >= 0 && id < byId.length ? byId[id] : null;
    }

    private void drop(Node n) {
        nodes.remove(n);
        byId[n.id] = null;
    }

    Forest copy() {
        Forest f = new Forest(legs);
        f.nextId = nextId;
        f.byId = new Node[byId.length];
        for (Node n : nodes) {
            Node c = new Node();
            c.id = n.id;
            c.type = n.type;
            c.x = n.x;
            c.y = n.y;
            c.path = n.path;
            c.len = n.len;
            c.extra = n.extra;
            c.term = n.term;
            c.chamber = n.chamber;
            c.seg = n.seg;
            c.at = n.at;
            c.flow = n.flow;
            c.dn = n.dn;
            c.run = n.run;
            c.siteDn = n.siteDn;
            f.byId[c.id] = c;
            f.nodes.add(c);
        }
        for (Node n : nodes) {
            Node c = f.byId[n.id];
            c.parent = n.parent == null ? null : f.byId[n.parent.id];
            for (Node k : n.kids) {
                c.kids.add(f.byId[k.id]);
            }
        }
        return f;
    }

    List<Node> roots() {
        List<Node> out = new ArrayList<>();
        for (Node n : nodes) {
            if (n.type == TAP) {
                out.add(n);
            }
        }
        return out;
    }

    Node root(Node n) {
        Node u = n;
        while (u.parent != null) {
            u = u.parent;
        }
        return u;
    }

    /** Отсоединить поддерево x; камера с одним оставшимся потомком растворяется в ломаную. */
    void detach(Node x) {
        Node p = x.parent;
        if (p == null) {
            return;
        }
        p.kids.remove(x);
        x.parent = null;
        x.path = null;
        x.len = 0;
        x.extra = 0;
        if (p.type == JUNC && p.kids.size() == 1) {
            dissolve(p);
        } else if (p.type == JUNC && p.kids.isEmpty()) {
            detach(p);
            drop(p);
        } else if (p.type == TAP && p.kids.isEmpty()) {
            drop(p);
        }
    }

    /** Камера с одним потомком: потомок получает ломаную камеры в продолжение своей. */
    void dissolve(Node p) {
        Node k = p.kids.get(0);
        Node gp = p.parent;
        k.path = Geo.concat(k.path, p.path);
        k.len += p.len;
        k.extra = legs.extra(k.path, gp != null && gp.type == TAP);
        k.parent = gp;
        if (gp != null) {
            int i = gp.kids.indexOf(p);
            gp.kids.set(i, k);
        }
        p.kids.clear();
        drop(p);
    }

    /** Подвесить x к узлу p ломаной path (от x к p). */
    void attachToNode(Node x, Node p, double[] path, double len, double extra) {
        x.parent = p;
        x.path = path;
        x.len = len;
        x.extra = extra;
        p.kids.add(x);
    }

    /**
     * Врезать x в ребро c → c.parent в точке q на отрезке seg ломаной c.path: новая камера в q.
     * Надбавки двух частей пересчитываются.
     */
    Node attachToEdge(Node x, Node c, int seg, double qx, double qy, double[] path, double len, double extra) {
        double[] cp = c.path;
        int nPts = cp.length / 2;
        double[] lower = new double[(seg + 2) * 2];
        System.arraycopy(cp, 0, lower, 0, (seg + 1) * 2);
        lower[(seg + 1) * 2] = qx;
        lower[(seg + 1) * 2 + 1] = qy;
        double[] upper = new double[(nPts - seg) * 2];
        upper[0] = qx;
        upper[1] = qy;
        System.arraycopy(cp, (seg + 1) * 2, upper, 2, (nPts - seg - 1) * 2);
        lower = dedupe(lower);
        upper = dedupe(upper);
        Node p = c.parent;
        Node j = add(JUNC, qx, qy);
        j.parent = p;
        j.path = upper;
        j.len = Geo.length(upper);
        j.extra = legs.extra(upper, p.type == TAP);
        p.kids.set(p.kids.indexOf(c), j);
        c.parent = j;
        c.path = lower;
        c.len = Geo.length(lower);
        c.extra = legs.extra(lower, false);
        j.kids.add(c);
        attachToNode(x, j, path, len, extra);
        return j;
    }

    /** Ломаная без повторяющихся подряд точек. */
    static double[] dedupe(double[] path) {
        double[] out = new double[path.length];
        int n = 0;
        for (int i = 0; i < path.length; i += 2) {
            if (n >= 2 && Math.abs(out[n - 2] - path[i]) < 1e-9 && Math.abs(out[n - 1] - path[i + 1]) < 1e-9) {
                continue;
            }
            out[n++] = path[i];
            out[n++] = path[i + 1];
        }
        return n == path.length ? path : java.util.Arrays.copyOf(out, n);
    }

    /** Корень-врезка для места; существующий узел, если место уже используется. */
    Node tapNode(int chamber, int seg, double at, double x, double y) {
        for (Node n : nodes) {
            if (n.tapSite(chamber, seg, at)) {
                return n;
            }
        }
        Node t = add(TAP, x, y);
        t.chamber = chamber;
        t.seg = chamber >= 0 ? -1 : seg;
        t.at = at;
        return t;
    }

    /** Все узлы поддерева x (включая x). */
    static void subtree(Node x, List<Node> out) {
        out.add(x);
        for (int i = out.size() - 1; i < out.size(); i++) {
            for (Node k : out.get(i).kids) {
                out.add(k);
            }
        }
    }

    int tieIns() {
        int n = 0;
        for (Node t : nodes) {
            if (t.type == TAP) {
                n += t.kids.size();
            }
        }
        return n;
    }
}
