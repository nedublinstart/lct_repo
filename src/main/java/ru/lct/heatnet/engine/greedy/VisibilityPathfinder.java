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
 * Граф видимости по углам кварталов: рёбра — прямые хорды вдоль тротуаров
 * и короткие пересечения проезжей ≥ 45°. Без сеточной лесенки.
 */
public final class VisibilityPathfinder {

    private static final double SIMPLIFY_M = 6.0;
    private static final double DENSIFY_M = 90.0;
    private static final int MAX_NODES = 2200;
    private static final double CORNER_TURN_DEG = 14;

    private final ObstacleIndex obstacles;
    private final List<Coordinate> nodes = new ArrayList<>();
    private final List<List<Integer>> adj = new ArrayList<>();
    private final List<List<Double>> adjW = new ArrayList<>();
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
        double direct = start.distance(goal);
        if (direct <= obstacles.maxOpenEdgeM()
                && !obstacles.segmentHitsAvoid(start, goal, 0, true)
                && obstacles.allowsTravel(start, goal)) {
            List<Coordinate> line = new ArrayList<>(2);
            line.add(new Coordinate(start));
            line.add(new Coordinate(goal));
            return line;
        }
        if (nodes.isEmpty()) {
            return null;
        }
        int s = nodes.size();
        int g = nodes.size() + 1;
        boolean[] goalLink = new boolean[nodes.size()];
        double[] goalW = new double[nodes.size()];
        List<Integer> startLinks = links(start, 40);
        List<Integer> goalLinks = links(goal, 40);
        for (int i : goalLinks) {
            double w = obstacles.travelCost(nodes.get(i), goal);
            if (Double.isFinite(w)) {
                goalLink[i] = true;
                goalW[i] = w;
            }
        }
        if (startLinks.isEmpty() || goalLinks.isEmpty()) {
            return null;
        }
        int n = nodes.size() + 2;
        double[] dist = new double[n];
        int[] parent = new int[n];
        byte[] seen = new byte[n];
        java.util.Arrays.fill(dist, Double.POSITIVE_INFINITY);
        java.util.Arrays.fill(parent, -1);
        dist[s] = 0;
        PriorityQueue<Node> pq = new PriorityQueue<>();
        pq.add(new Node(s, start.distance(goal)));
        int iter = 0;
        int limit = Math.max(8000, n * 12);
        double maxOpen = obstacles.maxOpenEdgeM();
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
                for (int ni : startLinks) {
                    double w = obstacles.travelCost(start, nodes.get(ni));
                    if (Double.isFinite(w)) {
                        relax(s, ni, w, nodes.get(ni), dist, parent, pq, goal);
                    }
                }
                continue;
            }
            Coordinate at = nodes.get(cur.i);
            List<Integer> nbs = adj.get(cur.i);
            List<Double> ws = adjW.get(cur.i);
            for (int k = 0; k < nbs.size(); k++) {
                int ni = nbs.get(k);
                relax(cur.i, ni, ws.get(k), nodes.get(ni), dist, parent, pq, goal);
            }
            if (goalLink[cur.i]) {
                relax(cur.i, g, goalW[cur.i], goal, dist, parent, pq, goal);
            } else if (at.distance(goal) <= maxOpen
                    && !obstacles.segmentHitsAvoid(at, goal, 0, true)
                    && obstacles.allowsTravel(at, goal)) {
                relax(cur.i, g, obstacles.travelCost(at, goal), goal, dist, parent, pq, goal);
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

    private void relax(int from, int to, double step, Coordinate b, double[] dist, int[] parent,
                       PriorityQueue<Node> pq, Coordinate goal) {
        if (!Double.isFinite(step)) {
            return;
        }
        double nd = dist[from] + step;
        if (nd + 1e-6 < dist[to]) {
            dist[to] = nd;
            parent[to] = from;
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
        for (Polygon p : polys) {
            addPolygon(p, simplify);
        }
        for (int i = 0; i < nodes.size(); i++) {
            adj.add(new ArrayList<>());
            adjW.add(new ArrayList<>());
            nodeTree.insert(new Envelope(nodes.get(i)), i);
        }
        nodeTree.build();
        for (int[] e : ringEdges) {
            if (e[0] < adj.size() && e[1] < adj.size() && e[0] != e[1]) {
                double w = nodes.get(e[0]).distance(nodes.get(e[1]));
                adj.get(e[0]).add(e[1]);
                adjW.get(e[0]).add(w);
                adj.get(e[1]).add(e[0]);
                adjW.get(e[1]).add(w);
            }
        }
        int n = nodes.size();
        double maxStreet = obstacles.maxStreetEdgeM();
        double maxOpen = obstacles.maxOpenEdgeM();
        for (int i = 0; i < n; i++) {
            Coordinate a = nodes.get(i);
            Envelope env = new Envelope(a);
            env.expandBy(maxOpen);
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
                if (d < 1.2 || d > maxOpen) {
                    continue;
                }
                if (obstacles.segmentHitsAvoid(a, b, 0, true)) {
                    continue;
                }
                SpecialLayer.Travel t = obstacles.special().inspect(a, b);
                if (!t.allowed) {
                    continue;
                }
                if (t.special && d > maxStreet) {
                    continue;
                }
                adj.get(i).add(j);
                adjW.get(i).add(t.cost);
                adj.get(j).add(i);
                adjW.get(j).add(t.cost);
            }
        }
    }

    private void addPolygon(Polygon polygon, double simplify) {
        Geometry source = polygon;
        try {
            Geometry inflated = polygon.buffer(0.4, 2);
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
            int parts = len > DENSIFY_M ? Math.min(3, (int) Math.floor(len / DENSIFY_M)) : 1;
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

    @SuppressWarnings("unchecked")
    private List<Integer> links(Coordinate c, int limit) {
        List<Integer> out = new ArrayList<>();
        Envelope env = new Envelope(c);
        for (double r : new double[]{40, 90, 160, 240, 360}) {
            env.init(c);
            env.expandBy(r);
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
                Coordinate b = nodes.get(i);
                if (obstacles.segmentHitsAvoid(c, b, 0, true)) {
                    continue;
                }
                if (!obstacles.allowsTravel(c, b)) {
                    continue;
                }
                if (!out.contains(i)) {
                    out.add(i);
                }
            }
            if (out.size() >= 12) {
                break;
            }
        }
        return out;
    }

    private static double turnDeg(Coordinate a, Coordinate b, Coordinate c) {
        double ux = b.x - a.x;
        double uy = b.y - a.y;
        double vx = c.x - b.x;
        double vy = c.y - b.y;
        double nu = Math.hypot(ux, uy);
        double nv = Math.hypot(vx, vy);
        if (nu < 1e-6 || nv < 1e-6) {
            return 0;
        }
        double cos = (ux * vx + uy * vy) / (nu * nv);
        cos = Math.max(-1, Math.min(1, cos));
        return Math.toDegrees(Math.acos(cos));
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
