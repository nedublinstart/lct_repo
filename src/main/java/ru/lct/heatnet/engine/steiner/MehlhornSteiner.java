package ru.lct.heatnet.engine.steiner;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.locationtech.jts.geom.Coordinate;

/**
 * Приближение дерева Штейнера по Mehlhorn: MST терминалов по кратчайшим путям,
 * разворот рёбер в исходный граф, MST объединения, обрезка листьев-нетерминалов.
 */
public final class MehlhornSteiner {

    private MehlhornSteiner() {
    }

    public static SteinerTree connect(List<OksPort> ports, TapCandidate tap, PathMetric metric, int maxChamberDegree) {
        if (tap == null || ports == null || ports.isEmpty()) {
            return SteinerTree.unconnected(ports);
        }
        List<OksPort> reachable = new ArrayList<>();
        List<OksPort> lost = new ArrayList<>();
        for (OksPort p : ports) {
            if (Double.isFinite(metric.cost(p.at, tap.coordinate))) {
                reachable.add(p);
            } else {
                lost.add(p);
            }
        }
        if (reachable.isEmpty()) {
            SteinerTree empty = SteinerTree.unconnected(ports);
            empty.tap = tap;
            return empty;
        }
        List<Term> terms = new ArrayList<>(reachable.size() + 1);
        for (OksPort p : reachable) {
            terms.add(Term.port(p));
        }
        terms.add(Term.tap(tap));
        int n = terms.size();
        double[][] w = new double[n][n];
        @SuppressWarnings("unchecked")
        List<Coordinate>[][] paths = new List[n][n];
        for (int i = 0; i < n; i++) {
            Arrays.fill(w[i], Double.POSITIVE_INFINITY);
            w[i][i] = 0;
        }
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                List<Coordinate> path = metric.find(terms.get(i).at, terms.get(j).at);
                double c = path == null ? Double.POSITIVE_INFINITY : metric.cost(terms.get(i).at, terms.get(j).at);
                if (path != null && Double.isFinite(c)) {
                    w[i][j] = c;
                    w[j][i] = c;
                    paths[i][j] = path;
                    List<Coordinate> rev = new ArrayList<>(path);
                    Collections.reverse(rev);
                    paths[j][i] = rev;
                }
            }
        }
        int[] parent = prim(w);
        if (parent == null) {
            return SteinerTree.unconnected(ports);
        }
        Overlay overlay = new Overlay();
        for (Term t : terms) {
            overlay.addTerminal(t);
        }
        for (int i = 0; i < n; i++) {
            int p = parent[i];
            if (p < 0) {
                continue;
            }
            List<Coordinate> path = paths[i][p];
            if (path == null) {
                return SteinerTree.unconnected(ports);
            }
            overlay.addPath(path);
        }
        SteinerTree tree = overlay.extract(tap, maxChamberDegree, metric);
        if (tree == null) {
            tree = star(reachable, tap, metric, maxChamberDegree);
        }
        if (tree == null) {
            return SteinerTree.unconnected(ports);
        }
        tree.unconnected.addAll(lost);
        return tree;
    }

    public static double terminalMstCost(List<OksPort> ports, TapCandidate tap, PathMetric metric) {
        return terminalMst(ports, tap, metric, false);
    }

    public static double terminalMstLength(List<OksPort> ports, TapCandidate tap, PathMetric metric) {
        return terminalMst(ports, tap, metric, true);
    }

    private static double terminalMst(List<OksPort> ports, TapCandidate tap, PathMetric metric, boolean useLength) {
        if (tap == null || ports == null || ports.isEmpty()) {
            return Double.POSITIVE_INFINITY;
        }
        int n = ports.size() + 1;
        double[][] w = new double[n][n];
        for (int i = 0; i < n; i++) {
            Arrays.fill(w[i], Double.POSITIVE_INFINITY);
            w[i][i] = 0;
        }
        for (int i = 0; i < ports.size(); i++) {
            double toTapCost = metric.cost(ports.get(i).at, tap.coordinate);
            if (!Double.isFinite(toTapCost)) {
                return Double.POSITIVE_INFINITY;
            }
            double toTap = useLength ? metric.length(ports.get(i).at, tap.coordinate) : toTapCost;
            w[i][n - 1] = toTap;
            w[n - 1][i] = toTap;
            for (int j = i + 1; j < ports.size(); j++) {
                double c = metric.cost(ports.get(i).at, ports.get(j).at);
                if (!Double.isFinite(c)) {
                    continue;
                }
                double edge = useLength ? metric.length(ports.get(i).at, ports.get(j).at) : c;
                w[i][j] = edge;
                w[j][i] = edge;
            }
        }
        int[] parent = prim(w);
        if (parent == null) {
            return Double.POSITIVE_INFINITY;
        }
        double sum = 0;
        for (int i = 0; i < n; i++) {
            if (parent[i] >= 0) {
                sum += w[i][parent[i]];
            }
        }
        return sum;
    }

    static int[] prim(double[][] w) {
        int n = w.length;
        boolean[] used = new boolean[n];
        double[] key = new double[n];
        int[] parent = new int[n];
        Arrays.fill(key, Double.POSITIVE_INFINITY);
        Arrays.fill(parent, -1);
        key[n - 1] = 0;
        for (int it = 0; it < n; it++) {
            int u = -1;
            double best = Double.POSITIVE_INFINITY;
            for (int i = 0; i < n; i++) {
                if (!used[i] && key[i] < best) {
                    best = key[i];
                    u = i;
                }
            }
            if (u < 0 || !Double.isFinite(best)) {
                return null;
            }
            used[u] = true;
            for (int v = 0; v < n; v++) {
                if (!used[v] && w[u][v] + 1e-9 < key[v]) {
                    key[v] = w[u][v];
                    parent[v] = u;
                }
            }
        }
        return parent;
    }

    static SteinerTree star(List<OksPort> ports, TapCandidate tap, PathMetric metric, int maxChamberDegree) {
        if (ports.size() > maxChamberDegree) {
            return null;
        }
        SteinerTree tree = new SteinerTree();
        tree.tap = tap;
        SteinerTree.Node tapNode = new SteinerTree.Node();
        tapNode.id = 0;
        tapNode.at = new Coordinate(tap.coordinate);
        tapNode.tap = true;
        tree.nodes.add(tapNode);
        double cost = 0;
        double length = 0;
        for (OksPort p : ports) {
            List<Coordinate> path = metric.find(p.at, tap.coordinate);
            if (path == null) {
                tree.unconnected.add(p);
                continue;
            }
            SteinerTree.Node n = new SteinerTree.Node();
            n.id = tree.nodes.size();
            n.at = new Coordinate(p.at);
            n.port = p;
            tree.nodes.add(n);
            SteinerTree.Branch b = new SteinerTree.Branch();
            b.from = n.id;
            b.to = tapNode.id;
            b.path = path;
            b.flow = p.flow();
            b.cost = metric.cost(p.at, tap.coordinate);
            tree.branches.add(b);
            tree.connected.add(p);
            if (Double.isFinite(b.cost)) {
                cost += b.cost;
            }
            length += MetricPathCache.lengthOf(path);
        }
        tree.cost = cost;
        tree.length = length;
        tree.tapChildren = tree.branches.size();
        if (tree.connected.isEmpty()) {
            return null;
        }
        return tree;
    }

    private static final class Term {
        final Coordinate at;
        final OksPort port;
        final boolean tap;

        private Term(Coordinate at, OksPort port, boolean tap) {
            this.at = at;
            this.port = port;
            this.tap = tap;
        }

        static Term port(OksPort p) {
            return new Term(p.at, p, false);
        }

        static Term tap(TapCandidate t) {
            return new Term(t.coordinate, null, true);
        }
    }

    private static final class Overlay {
        private final List<Coordinate> coords = new ArrayList<>();
        private final List<OksPort> portAt = new ArrayList<>();
        private final List<Boolean> tapAt = new ArrayList<>();
        private final Map<String, Integer> index = new HashMap<>();
        private final List<OvEdge> edges = new ArrayList<>();

        void addTerminal(Term t) {
            int id = addNode(t.at, 2.2);
            if (t.port != null) {
                portAt.set(id, t.port);
            }
            if (t.tap) {
                tapAt.set(id, true);
            }
        }

        void addPath(List<Coordinate> path) {
            if (path == null || path.size() < 2) {
                return;
            }
            int prev = addNode(path.get(0), 2.2);
            for (int i = 1; i < path.size(); i++) {
                int cur = addNode(path.get(i), 2.6);
                if (cur == prev) {
                    continue;
                }
                double len = coords.get(prev).distance(coords.get(cur));
                if (len < 1e-4) {
                    continue;
                }
                edges.add(new OvEdge(prev, cur, len));
                prev = cur;
            }
        }

        SteinerTree extract(TapCandidate tap, int maxChamberDegree, PathMetric metric) {
            int tapNode = -1;
            for (int i = 0; i < coords.size(); i++) {
                if (Boolean.TRUE.equals(tapAt.get(i))) {
                    tapNode = i;
                    break;
                }
            }
            if (tapNode < 0) {
                tapNode = nearest(tap.coordinate);
            }
            if (tapNode < 0) {
                return null;
            }
            tapAt.set(tapNode, true);
            List<Integer> portNodes = new ArrayList<>();
            for (int i = 0; i < coords.size(); i++) {
                if (portAt.get(i) != null) {
                    portNodes.add(i);
                }
            }
            if (portNodes.isEmpty()) {
                return null;
            }
            List<OvEdge> mst = kruskal(coords.size(), edges);
            List<List<OvEdge>> adj = adjacency(coords.size(), mst);
            prune(adj, portNodes, tapNode);
            if (adj.get(tapNode).size() > maxChamberDegree) {
                return null;
            }
            int[] parent = bfsParent(adj, tapNode);
            for (int p : portNodes) {
                if (parent[p] < 0 && p != tapNode) {
                    return null;
                }
            }
            boolean[] keep = new boolean[coords.size()];
            keep[tapNode] = true;
            for (int p : portNodes) {
                keep[p] = true;
            }
            for (int i = 0; i < coords.size(); i++) {
                if (adj.get(i).size() >= 3) {
                    keep[i] = true;
                }
            }
            Map<Integer, Integer> map = new HashMap<>();
            SteinerTree tree = new SteinerTree();
            tree.tap = tap;
            for (int i = 0; i < coords.size(); i++) {
                if (!keep[i]) {
                    continue;
                }
                SteinerTree.Node node = new SteinerTree.Node();
                node.id = tree.nodes.size();
                node.at = new Coordinate(coords.get(i));
                node.port = portAt.get(i);
                node.tap = i == tapNode || Boolean.TRUE.equals(tapAt.get(i));
                map.put(i, node.id);
                tree.nodes.add(node);
                if (node.port != null) {
                    tree.connected.add(node.port);
                }
            }
            for (int i = 0; i < coords.size(); i++) {
                if (!keep[i] || i == tapNode) {
                    continue;
                }
                List<Coordinate> path = new ArrayList<>();
                path.add(new Coordinate(coords.get(i)));
                int cur = i;
                int nextKeep = -1;
                int guard = 0;
                while (cur != tapNode && guard++ < coords.size() + 2) {
                    int par = parent[cur];
                    if (par < 0) {
                        return null;
                    }
                    path.add(new Coordinate(coords.get(par)));
                    cur = par;
                    if (keep[cur]) {
                        nextKeep = cur;
                        break;
                    }
                }
                if (nextKeep < 0) {
                    return null;
                }
                SteinerTree.Branch b = new SteinerTree.Branch();
                b.from = map.get(i);
                b.to = map.get(nextKeep);
                b.path = path;
                b.cost = MetricPathCache.lengthOf(path);
                tree.branches.add(b);
            }
            assignFlows(tree);
            for (SteinerTree.Node node : tree.nodes) {
                int deg = 0;
                for (SteinerTree.Branch b : tree.branches) {
                    if (b.from == node.id || b.to == node.id) {
                        deg++;
                    }
                }
                node.junction = !node.tap && node.port == null && deg >= 3;
            }
            tree.tapChildren = 0;
            for (SteinerTree.Branch b : tree.branches) {
                if (tree.nodes.get(b.to).tap) {
                    tree.tapChildren++;
                }
            }
            if (tree.tapChildren > maxChamberDegree) {
                return null;
            }
            tree.length = 0;
            tree.cost = 0;
            for (SteinerTree.Branch b : tree.branches) {
                double len = MetricPathCache.lengthOf(b.path);
                tree.length += len;
                double edgeCost = metric instanceof MetricPathCache
                        ? ((MetricPathCache) metric).pathCost(b.path)
                        : len;
                if (!Double.isFinite(edgeCost)) {
                    edgeCost = len;
                }
                tree.cost += edgeCost;
            }
            if (!Double.isFinite(tree.cost)) {
                tree.cost = tree.length;
            }
            return tree;
        }

        private int addNode(Coordinate c, double snapM) {
            int near = nearestWithin(c, snapM);
            if (near >= 0) {
                return near;
            }
            String key = Math.round(c.x * 2) + ":" + Math.round(c.y * 2);
            Integer existing = index.get(key);
            if (existing != null) {
                return existing;
            }
            int id = coords.size();
            coords.add(new Coordinate(c));
            portAt.add(null);
            tapAt.add(false);
            index.put(key, id);
            return id;
        }

        private int nearest(Coordinate c) {
            return nearestWithin(c, 2.4);
        }

        private int nearestWithin(Coordinate c, double snapM) {
            int best = -1;
            double bestD = snapM;
            for (int i = 0; i < coords.size(); i++) {
                double d = coords.get(i).distance(c);
                if (d <= bestD) {
                    bestD = d;
                    best = i;
                }
            }
            return best;
        }
    }

    private static List<OvEdge> kruskal(int n, List<OvEdge> edges) {
        List<OvEdge> sorted = new ArrayList<>(edges);
        sorted.sort(Comparator.comparingDouble(e -> e.w));
        int[] p = new int[n];
        int[] r = new int[n];
        for (int i = 0; i < n; i++) {
            p[i] = i;
        }
        List<OvEdge> mst = new ArrayList<>();
        for (OvEdge e : sorted) {
            int a = find(p, e.a);
            int b = find(p, e.b);
            if (a == b) {
                continue;
            }
            if (r[a] < r[b]) {
                p[a] = b;
            } else if (r[a] > r[b]) {
                p[b] = a;
            } else {
                p[b] = a;
                r[a]++;
            }
            mst.add(e);
        }
        return mst;
    }

    private static int find(int[] p, int x) {
        while (p[x] != x) {
            p[x] = p[p[x]];
            x = p[x];
        }
        return x;
    }

    private static List<List<OvEdge>> adjacency(int n, List<OvEdge> edges) {
        List<List<OvEdge>> adj = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            adj.add(new ArrayList<>());
        }
        for (OvEdge e : edges) {
            adj.get(e.a).add(e);
            adj.get(e.b).add(e);
        }
        return adj;
    }

    private static void prune(List<List<OvEdge>> adj, List<Integer> portNodes, int tapNode) {
        boolean[] terminal = new boolean[adj.size()];
        terminal[tapNode] = true;
        for (int p : portNodes) {
            terminal[p] = true;
        }
        boolean changed = true;
        while (changed) {
            changed = false;
            for (int i = 0; i < adj.size(); i++) {
                if (terminal[i] || adj.get(i).size() != 1) {
                    continue;
                }
                OvEdge e = adj.get(i).get(0);
                int other = e.a == i ? e.b : e.a;
                adj.get(i).clear();
                adj.get(other).remove(e);
                changed = true;
            }
        }
    }

    private static int[] bfsParent(List<List<OvEdge>> adj, int tapNode) {
        int[] parent = new int[adj.size()];
        Arrays.fill(parent, -1);
        ArrayDeque<Integer> q = new ArrayDeque<>();
        q.add(tapNode);
        parent[tapNode] = tapNode;
        while (!q.isEmpty()) {
            int u = q.removeFirst();
            for (OvEdge e : adj.get(u)) {
                int v = e.a == u ? e.b : e.a;
                if (parent[v] < 0) {
                    parent[v] = u;
                    q.add(v);
                }
            }
        }
        return parent;
    }

    private static void assignFlows(SteinerTree tree) {
        double[] flow = new double[tree.nodes.size()];
        for (SteinerTree.Node n : tree.nodes) {
            if (n.port != null) {
                flow[n.id] = n.port.flow();
            }
        }
        int[] waiting = new int[tree.nodes.size()];
        for (SteinerTree.Branch b : tree.branches) {
            waiting[b.to]++;
        }
        ArrayDeque<Integer> q = new ArrayDeque<>();
        for (int i = 0; i < waiting.length; i++) {
            if (waiting[i] == 0) {
                q.add(i);
            }
        }
        boolean[] done = new boolean[tree.branches.size()];
        int guard = 0;
        while (!q.isEmpty() && guard++ < 20_000) {
            int u = q.removeFirst();
            for (int i = 0; i < tree.branches.size(); i++) {
                if (done[i]) {
                    continue;
                }
                SteinerTree.Branch b = tree.branches.get(i);
                if (b.from != u) {
                    continue;
                }
                b.flow = flow[u];
                flow[b.to] += flow[u];
                done[i] = true;
                waiting[b.to]--;
                if (waiting[b.to] == 0) {
                    q.add(b.to);
                }
            }
        }
        for (int i = 0; i < tree.branches.size(); i++) {
            if (!done[i]) {
                SteinerTree.Branch b = tree.branches.get(i);
                if (b.flow <= 0) {
                    b.flow = flow[b.from];
                }
            }
        }
    }

    private static final class OvEdge {
        final int a;
        final int b;
        final double w;

        OvEdge(int a, int b, double w) {
            this.a = a;
            this.b = b;
            this.w = w;
        }
    }
}
