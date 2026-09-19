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
 * Граф видимости по контурам кварталов: прямые рёбра вдоль улиц, без срезания дворов.
 */
public final class VisibilityPathfinder {

    private static final double SIMPLIFY_M = 5.0;
    private static final double DENSIFY_M = 18.0;
    private static final double MAX_EDGE_M = 160.0;
    private static final int MAX_NODES = 2200;

    private final ObstacleIndex obstacles;
    private final List<Coordinate> nodes = new ArrayList<>();
    private final List<List<Integer>> adj = new ArrayList<>();
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
        if (!obstacles.segmentHitsAvoid(start, goal)) {
            List<Coordinate> direct = new ArrayList<>(2);
            direct.add(new Coordinate(start));
            direct.add(new Coordinate(goal));
            return direct;
        }
        if (nodes.isEmpty()) {
            return null;
        }
        int s = nodes.size();
        int g = nodes.size() + 1;
        boolean[] goalLink = new boolean[nodes.size()];
        List<Integer> startLinks = links(start, 28);
        List<Integer> goalLinks = links(goal, 28);
        for (int i : goalLinks) {
            goalLink[i] = true;
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
                    relax(s, ni, start, nodes.get(ni), dist, parent, pq, goal);
                }
                continue;
            }
            Coordinate at = nodes.get(cur.i);
            for (int ni : adj.get(cur.i)) {
                relax(cur.i, ni, at, nodes.get(ni), dist, parent, pq, goal);
            }
            if (goalLink[cur.i] || !obstacles.segmentHitsAvoid(at, goal)) {
                relax(cur.i, g, at, goal, dist, parent, pq, goal);
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
        return path;
    }

    private void relax(int from, int to, Coordinate a, Coordinate b, double[] dist, int[] parent,
                       PriorityQueue<Node> pq, Coordinate goal) {
        double nd = dist[from] + a.distance(b);
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
        if (estimate > MAX_NODES * 3) {
            simplify = 8.0;
        }
        for (Polygon p : polys) {
            addPolygon(p, simplify);
        }
        for (int i = 0; i < nodes.size(); i++) {
            adj.add(new ArrayList<>());
            nodeTree.insert(new Envelope(nodes.get(i)), i);
        }
        nodeTree.build();
        for (int[] e : ringEdges) {
            if (e[0] < adj.size() && e[1] < adj.size() && e[0] != e[1]) {
                adj.get(e[0]).add(e[1]);
                adj.get(e[1]).add(e[0]);
            }
        }
        int n = nodes.size();
        for (int i = 0; i < n; i++) {
            Coordinate a = nodes.get(i);
            Envelope env = new Envelope(a);
            env.expandBy(MAX_EDGE_M);
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
                if (d < 1.2 || d > MAX_EDGE_M) {
                    continue;
                }
                if (!obstacles.segmentHitsAvoid(a, b)) {
                    adj.get(i).add(j);
                    adj.get(j).add(i);
                }
            }
        }
    }

    private void addPolygon(Polygon polygon, double simplify) {
        Geometry source = polygon;
        try {
            Geometry inflated = polygon.buffer(0.7, 6);
            if (inflated instanceof Polygon) {
                source = DouglasPeuckerSimplifier.simplify(inflated, simplify);
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
        List<Integer> ids = new ArrayList<>();
        for (int i = 0; i < ring.length - 1; i++) {
            Coordinate a = ring[i];
            Coordinate b = ring[i + 1];
            addDense(ids, a, b);
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

    private void addDense(List<Integer> ids, Coordinate a, Coordinate b) {
        ids.add(nodeId(a));
        double len = a.distance(b);
        int parts = Math.max(1, (int) Math.floor(len / DENSIFY_M));
        for (int k = 1; k < parts; k++) {
            double t = k / (double) parts;
            ids.add(nodeId(new Coordinate(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))));
        }
    }

    private int nodeId(Coordinate c) {
        String key = Math.round(c.x * 2) + ":" + Math.round(c.y * 2);
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
        for (double r : new double[]{40, 80, 130, 200}) {
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
                if (!obstacles.segmentHitsAvoid(c, nodes.get(i))) {
                    out.add(i);
                }
            }
            if (out.size() >= 6) {
                break;
            }
        }
        return out;
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
