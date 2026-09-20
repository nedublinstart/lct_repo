package ru.lct.heatnet.engine.greedy;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.index.strtree.STRtree;
import org.locationtech.jts.simplify.DouglasPeuckerSimplifier;

/**
 * Видимый граф по ТЗ: вершины — углы буферов кварталов (тротуар у фасада),
 * рёбра — вдоль кольца, короткие осевые связи и пересечения улиц ≥ ~70°.
 * Длинные диагонали через проспект/парк не создаются.
 */
public final class VisibilityPathfinder {

    private static final double SIMPLIFY_M = 6.0;
    private static final double DENSIFY_M = 36.0;
    private static final int MAX_NODES = 2800;
    private static final double CORNER_TURN_DEG = 14;
    private static final double RING_OFFSET_M = 1.25;

    private final ObstacleIndex obstacles;
    private final List<Coordinate> nodes = new ArrayList<>();
    private final List<List<Integer>> adj = new ArrayList<>();
    private final List<List<Double>> adjW = new ArrayList<>();
    private final List<List<Coordinate>> adjVia = new ArrayList<>();
    private final Map<String, Integer> index = new HashMap<>();
    private final List<int[]> ringEdges = new ArrayList<>();
    private final STRtree nodeTree = new STRtree();

    private VisibilityPathfinder(ObstacleIndex obstacles) {
        this.obstacles = obstacles;
    }

    public static VisibilityPathfinder build(ObstacleIndex obstacles) {
        VisibilityPathfinder vis = new VisibilityPathfinder(obstacles);
        vis.buildGraph();
        return vis;
    }

    public List<Coordinate> find(Coordinate start, Coordinate goal) {
        if (start == null || goal == null) {
            return null;
        }
        List<Coordinate> direct = OrthoPaths.usefulChord(obstacles, start, goal)
                ? two(start, goal)
                : OrthoPaths.usefulElbow(obstacles, start, goal);
        if (direct != null) {
            return PathSmoother.straighten(direct, obstacles);
        }
        if (nodes.isEmpty()) {
            return null;
        }
        int s = nodes.size();
        int g = nodes.size() + 1;
        int n = nodes.size() + 2;
        Coordinate[] goalVia = new Coordinate[nodes.size()];
        double[] goalW = new double[nodes.size()];
        boolean[] goalLink = new boolean[nodes.size()];
        List<Link> startLinks = endpointLinks(start, 48);
        List<Link> goalLinks = endpointLinks(goal, 48);
        for (Link link : goalLinks) {
            goalLink[link.to] = true;
            goalW[link.to] = link.w;
            goalVia[link.to] = link.via;
        }
        if (startLinks.isEmpty() || goalLinks.isEmpty()) {
            return null;
        }
        double[] dist = new double[n];
        int[] parent = new int[n];
        Coordinate[] viaAt = new Coordinate[n];
        byte[] seen = new byte[n];
        java.util.Arrays.fill(dist, Double.POSITIVE_INFINITY);
        java.util.Arrays.fill(parent, -1);
        dist[s] = 0;
        PriorityQueue<Node> pq = new PriorityQueue<>();
        pq.add(new Node(s, start.distance(goal)));
        int iter = 0;
        int limit = Math.max(10_000, n * 16);
        while (!pq.isEmpty() && iter++ < limit) {
            Node cur = pq.poll();
            if (seen[cur.i] == 1) {
                continue;
            }
            seen[cur.i] = 1;
            if (cur.i == g) {
                break;
            }
            if (cur.i == s) {
                for (Link link : startLinks) {
                    relax(s, link.to, link.w, nodes.get(link.to), link.via, dist, parent, viaAt, pq, goal);
                }
                continue;
            }
            Coordinate at = nodes.get(cur.i);
            List<Integer> nbs = adj.get(cur.i);
            List<Double> ws = adjW.get(cur.i);
            List<Coordinate> vias = adjVia.get(cur.i);
            for (int k = 0; k < nbs.size(); k++) {
                int ni = nbs.get(k);
                relax(cur.i, ni, ws.get(k), nodes.get(ni), vias.get(k), dist, parent, viaAt, pq, goal);
            }
            if (goalLink[cur.i]) {
                relax(cur.i, g, goalW[cur.i], goal, goalVia[cur.i], dist, parent, viaAt, pq, goal);
            } else {
                Link snap = tryLink(at, goal, cur.i);
                if (snap != null) {
                    relax(cur.i, g, snap.w, goal, snap.via, dist, parent, viaAt, pq, goal);
                }
            }
        }
        if (parent[g] < 0) {
            return null;
        }
        List<Coordinate> path = new ArrayList<>();
        path.add(new Coordinate(goal));
        int i = g;
        int guard = 0;
        while (i != s && i >= 0 && guard++ < n) {
            Coordinate via = viaAt[i];
            if (via != null) {
                path.add(new Coordinate(via));
            }
            i = parent[i];
            if (i == s) {
                path.add(new Coordinate(start));
            } else if (i >= 0 && i < nodes.size()) {
                path.add(new Coordinate(nodes.get(i)));
            }
        }
        if (path.get(path.size() - 1).distance(start) > 1e-6) {
            path.add(new Coordinate(start));
        }
        Collections.reverse(path);
        return PathSmoother.straighten(path, obstacles);
    }

    private void relax(int from, int to, double step, Coordinate b, Coordinate via, double[] dist, int[] parent,
                       Coordinate[] viaAt, PriorityQueue<Node> pq, Coordinate goal) {
        if (!Double.isFinite(step)) {
            return;
        }
        double nd = dist[from] + step;
        if (nd + 1e-6 < dist[to]) {
            dist[to] = nd;
            parent[to] = from;
            viaAt[to] = via;
            pq.add(new Node(to, nd + b.distance(goal)));
        }
    }

    private void buildGraph() {
        double simplify = SIMPLIFY_M;
        List<Polygon> polys = obstacles.avoidPolygons();
        int estimate = 0;
        for (Polygon p : polys) {
            estimate += Math.max(4, p.getNumPoints());
        }
        if (estimate > MAX_NODES * 2) {
            simplify = 10.0;
        }
        double offset = RING_OFFSET_M;
        try {
            offset = Math.max(0.9, Math.min(1.6, obstacles.sidewalkM() * 0.35));
        } catch (RuntimeException ignored) {
        }
        for (Polygon p : polys) {
            addPolygon(p, simplify, offset);
        }
        for (int i = 0; i < nodes.size(); i++) {
            adj.add(new ArrayList<>());
            adjW.add(new ArrayList<>());
            adjVia.add(new ArrayList<>());
            nodeTree.insert(new Envelope(nodes.get(i)), i);
        }
        nodeTree.build();
        for (int[] e : ringEdges) {
            if (e[0] < adj.size() && e[1] < adj.size() && e[0] != e[1]) {
                Coordinate a = nodes.get(e[0]);
                Coordinate b = nodes.get(e[1]);
                double w = obstacles.travelCost(a, b);
                if (!Double.isFinite(w)) {
                    if (!OrthoPaths.usefulChord(obstacles, a, b) && a.distance(b) > 14) {
                        continue;
                    }
                    w = a.distance(b);
                } else if (a.distance(b) > 40 && OrthoPaths.longOpenDiagonal(a, b)
                        && !obstacles.alongAvoid(a, b, 5.5)) {
                    continue;
                }
                addUndirected(e[0], e[1], w, null);
            }
        }
        int n = nodes.size();
        double reach = Math.max(obstacles.maxStreetEdgeM() + 40, 80);
        for (int i = 0; i < n; i++) {
            Coordinate a = nodes.get(i);
            Envelope env = new Envelope(a);
            env.expandBy(reach);
            @SuppressWarnings("unchecked")
            List<Integer> near = nodeTree.query(env);
            if (near == null) {
                continue;
            }
            for (int j : near) {
                if (j <= i) {
                    continue;
                }
                Coordinate b = nodes.get(j);
                double d = a.distance(b);
                if (d < 1.2 || d > reach) {
                    continue;
                }
                if (OrthoPaths.usefulChord(obstacles, a, b)) {
                    double w = obstacles.travelCost(a, b);
                    if (Double.isFinite(w)) {
                        addUndirected(i, j, w, null);
                    }
                    continue;
                }
                double dx = Math.abs(a.x - b.x);
                double dy = Math.abs(a.y - b.y);
                if (Math.min(dx, dy) > obstacles.maxStreetEdgeM() + 16) {
                    continue;
                }
                List<Coordinate> elbow = OrthoPaths.usefulElbow(obstacles, a, b);
                if (elbow != null && elbow.size() == 3) {
                    double w = obstacles.travelCost(elbow.get(0), elbow.get(1))
                            + obstacles.travelCost(elbow.get(1), elbow.get(2));
                    if (Double.isFinite(w)) {
                        addUndirected(i, j, w, elbow.get(1));
                    }
                }
            }
        }
    }

    private void addUndirected(int i, int j, double w, Coordinate via) {
        if (i == j || i < 0 || j < 0 || i >= adj.size() || j >= adj.size()) {
            return;
        }
        adj.get(i).add(j);
        adjW.get(i).add(w);
        adjVia.get(i).add(via);
        adj.get(j).add(i);
        adjW.get(j).add(w);
        adjVia.get(j).add(via);
    }

    private void addPolygon(Polygon polygon, double simplify, double offset) {
        Geometry source = polygon;
        try {
            Geometry inflated = polygon.buffer(offset, 2);
            if (inflated != null && !inflated.isEmpty()) {
                source = DouglasPeuckerSimplifier.simplify(inflated, simplify);
            } else {
                source = DouglasPeuckerSimplifier.simplify(polygon, simplify);
            }
        } catch (RuntimeException e) {
            try {
                source = DouglasPeuckerSimplifier.simplify(polygon, simplify);
            } catch (RuntimeException ignored) {
                source = polygon;
            }
        }
        if (source instanceof Polygon) {
            addRing(((Polygon) source).getExteriorRing().getCoordinates());
        } else {
            for (int i = 0; i < source.getNumGeometries(); i++) {
                Geometry g = source.getGeometryN(i);
                if (g instanceof Polygon) {
                    addRing(((Polygon) g).getExteriorRing().getCoordinates());
                }
            }
        }
    }

    private void addRing(Coordinate[] ring) {
        if (ring == null || ring.length < 2 || nodes.size() >= MAX_NODES) {
            return;
        }
        List<Coordinate> corners = corners(ring);
        if (corners.size() < 2) {
            return;
        }
        List<Integer> ids = new ArrayList<>();
        for (int i = 0; i < corners.size(); i++) {
            Coordinate a = corners.get(i);
            Coordinate b = corners.get((i + 1) % corners.size());
            ids.add(nodeId(a));
            double len = a.distance(b);
            int parts = len > DENSIFY_M ? Math.min(6, (int) Math.floor(len / DENSIFY_M)) : 1;
            for (int k = 1; k < parts; k++) {
                if (nodes.size() >= MAX_NODES) {
                    break;
                }
                double t = k / (double) parts;
                ids.add(nodeId(new Coordinate(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))));
            }
            if (nodes.size() >= MAX_NODES) {
                break;
            }
        }
        for (int i = 0; i < ids.size(); i++) {
            int a = ids.get(i);
            int b = ids.get((i + 1) % ids.size());
            if (a != b) {
                ringEdges.add(new int[]{a, b});
            }
        }
    }

    private static List<Coordinate> corners(Coordinate[] ring) {
        int n = ring.length;
        while (n >= 2 && ring[0].distance(ring[n - 1]) < 1e-6) {
            n--;
        }
        List<Coordinate> out = new ArrayList<>();
        if (n < 3) {
            for (int i = 0; i < n; i++) {
                out.add(new Coordinate(ring[i]));
            }
            return out;
        }
        for (int i = 0; i < n; i++) {
            Coordinate a = ring[(i - 1 + n) % n];
            Coordinate b = ring[i];
            Coordinate c = ring[(i + 1) % n];
            if (turnDeg(a, b, c) < CORNER_TURN_DEG) {
                continue;
            }
            if (out.isEmpty() || out.get(out.size() - 1).distance(b) >= 1.0) {
                out.add(new Coordinate(b));
            }
        }
        if (out.size() >= 2 && out.get(0).distance(out.get(out.size() - 1)) < 1.0) {
            out.remove(out.size() - 1);
        }
        if (out.size() >= 3) {
            return out;
        }
        out.clear();
        for (int i = 0; i < n; i++) {
            Coordinate b = ring[i];
            if (out.isEmpty() || out.get(out.size() - 1).distance(b) >= 2.5) {
                out.add(new Coordinate(b));
            }
        }
        return out;
    }

    private int nodeId(Coordinate c) {
        String key = Math.round(c.x) + ":" + Math.round(c.y);
        Integer existing = index.get(key);
        if (existing != null) {
            return existing;
        }
        int id = nodes.size();
        index.put(key, id);
        nodes.add(new Coordinate(c));
        return id;
    }

    private List<Link> endpointLinks(Coordinate c, int limit) {
        List<Link> out = new ArrayList<>();
        if (nodes.isEmpty()) {
            return out;
        }
        Envelope env = new Envelope(c);
        for (double r : new double[]{28, 56, 90, 140, 220}) {
            env.init(c);
            env.expandBy(r);
            @SuppressWarnings("unchecked")
            List<Integer> near = nodeTree.query(env);
            if (near == null) {
                continue;
            }
            List<Integer> ordered = new ArrayList<>(near);
            ordered.sort((i, j) -> Double.compare(nodes.get(i).distance(c), nodes.get(j).distance(c)));
            for (int i : ordered) {
                if (out.size() >= limit) {
                    break;
                }
                Link link = tryLink(c, nodes.get(i), i);
                if (link != null && !containsTo(out, i)) {
                    out.add(link);
                }
            }
            if (out.size() >= 10) {
                break;
            }
        }
        if (out.isEmpty()) {
            List<Integer> all = new ArrayList<>();
            for (int i = 0; i < nodes.size(); i++) {
                all.add(i);
            }
            all.sort((i, j) -> Double.compare(nodes.get(i).distance(c), nodes.get(j).distance(c)));
            for (int i = 0; i < Math.min(12, all.size()); i++) {
                int id = all.get(i);
                Link link = tryLink(c, nodes.get(id), id);
                if (link == null) {
                    List<Coordinate> elbow = OrthoPaths.bestElbow(obstacles, c, nodes.get(id));
                    if (elbow != null && c.distance(nodes.get(id)) <= 120) {
                        double w = OrthoPaths.length(elbow);
                        if (elbow.size() == 3) {
                            w = sumCost(elbow);
                        } else {
                            w = obstacles.travelCost(c, nodes.get(id));
                        }
                        if (Double.isFinite(w)) {
                            link = new Link(id, w, elbow.size() == 3 ? elbow.get(1) : null);
                        }
                    }
                }
                if (link != null && !containsTo(out, id)) {
                    out.add(link);
                }
                if (out.size() >= 4) {
                    break;
                }
            }
        }
        return out;
    }

    private Link tryLink(Coordinate from, Coordinate to, int toId) {
        if (from.distance(to) < 0.3) {
            return new Link(toId, 0.3, null);
        }
        if (OrthoPaths.usefulChord(obstacles, from, to)) {
            double w = obstacles.travelCost(from, to);
            return Double.isFinite(w) ? new Link(toId, w, null) : null;
        }
        if (from.distance(to) <= 36 && OrthoPaths.legal(obstacles, from, to)
                && (OrthoPaths.axisAligned(from, to) || from.distance(to) <= 22)) {
            double w = obstacles.travelCost(from, to);
            return Double.isFinite(w) ? new Link(toId, w, null) : null;
        }
        List<Coordinate> elbow = OrthoPaths.usefulElbow(obstacles, from, to);
        if (elbow != null && elbow.size() == 3) {
            double w = sumCost(elbow);
            if (Double.isFinite(w)) {
                return new Link(toId, w, elbow.get(1));
            }
        }
        return null;
    }

    private double sumCost(List<Coordinate> path) {
        double s = 0;
        for (int i = 1; i < path.size(); i++) {
            double w = obstacles.travelCost(path.get(i - 1), path.get(i));
            if (!Double.isFinite(w)) {
                return Double.POSITIVE_INFINITY;
            }
            s += w;
        }
        return s;
    }

    private static boolean containsTo(List<Link> links, int to) {
        for (Link link : links) {
            if (link.to == to) {
                return true;
            }
        }
        return false;
    }

    private static List<Coordinate> two(Coordinate a, Coordinate b) {
        List<Coordinate> line = new ArrayList<>(2);
        line.add(new Coordinate(a));
        line.add(new Coordinate(b));
        return line;
    }

    private static double turnDeg(Coordinate a, Coordinate b, Coordinate c) {
        return OrthoPaths.turnDeg(a, b, c);
    }

    private static final class Link {
        final int to;
        final double w;
        final Coordinate via;

        Link(int to, double w, Coordinate via) {
            this.to = to;
            this.w = w;
            this.via = via;
        }
    }

    private static final class Node implements Comparable<Node> {
        final int i;
        final double f;

        Node(int i, double f) {
            this.i = i;
            this.f = f;
        }

        @Override
        public int compareTo(Node o) {
            return Double.compare(f, o.f);
        }
    }
}
