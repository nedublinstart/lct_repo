package ru.lct.heatnet.scene;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.appendix.AppendixModel;

/**
 * Если во входе нет upstream_object_id, восстанавливаем цепочку к источнику
 * по геометрии существующей сети.
 */
public final class NetworkTopology {

    private NetworkTopology() {
    }

    public static void infer(Scene scene, AppendixModel appendix) {
        if (scene.segments.isEmpty()) {
            return;
        }
        double snap = appendix.getRouting().endpointSnapM;
        List<TopoNode> nodes = new ArrayList<>();
        Map<String, TopoNode> byId = new HashMap<>();
        for (HeatSource src : scene.sources) {
            add(nodes, byId, new TopoNode(src.id, "source", src.point.getCoordinate()));
        }
        for (Chamber ch : scene.chambers) {
            add(nodes, byId, new TopoNode(ch.id, "chamber", ch.point.getCoordinate()));
        }
        List<TopoEdge> edges = new ArrayList<>();
        int virt = 0;
        for (ExistingSegment seg : scene.segments) {
            Coordinate start = seg.line.getCoordinateN(0);
            Coordinate end = seg.line.getCoordinateN(seg.line.getNumPoints() - 1);
            TopoNode a = closest(nodes, start, snap);
            TopoNode b = closest(nodes, end, snap);
            if (a == null) {
                a = new TopoNode("virt-" + (++virt), "virtual", start);
                add(nodes, byId, a);
            }
            if (b == null) {
                b = new TopoNode("virt-" + (++virt), "virtual", end);
                add(nodes, byId, b);
            }
            TopoEdge e = new TopoEdge(seg, a, b, Math.max(0.2, seg.line.getLength()));
            a.edges.add(e);
            b.edges.add(e);
            edges.add(e);
        }
        if (scene.sources.isEmpty()) {
            return;
        }
        Map<String, Double> dist = new HashMap<>();
        Map<String, TopoEdge> prevEdge = new HashMap<>();
        for (TopoNode n : nodes) {
            dist.put(n.id, Double.POSITIVE_INFINITY);
        }
        PriorityQueue<Dist> queue = new PriorityQueue<>(Comparator.comparingDouble(d -> d.d));
        for (HeatSource src : scene.sources) {
            dist.put(src.id, 0.0);
            queue.add(new Dist(src.id, 0));
        }
        while (!queue.isEmpty()) {
            Dist cur = queue.poll();
            if (cur.d > dist.getOrDefault(cur.id, Double.POSITIVE_INFINITY) + 1e-9) {
                continue;
            }
            TopoNode node = byId.get(cur.id);
            if (node == null) {
                continue;
            }
            for (TopoEdge e : node.edges) {
                TopoNode other = e.a.id.equals(node.id) ? e.b : e.a;
                double nd = cur.d + e.length;
                if (nd + 1e-9 < dist.getOrDefault(other.id, Double.POSITIVE_INFINITY)) {
                    dist.put(other.id, nd);
                    prevEdge.put(other.id, e);
                    queue.add(new Dist(other.id, nd));
                }
            }
        }
        for (TopoEdge e : edges) {
            if (e.seg.nextId != null && !e.seg.nextId.isBlank()) {
                continue;
            }
            double da = dist.getOrDefault(e.a.id, Double.POSITIVE_INFINITY);
            double db = dist.getOrDefault(e.b.id, Double.POSITIVE_INFINITY);
            TopoNode up = da <= db ? e.a : e.b;
            if ("chamber".equals(up.kind) || "source".equals(up.kind)) {
                e.seg.nextId = up.id;
            } else {
                TopoEdge towards = prevEdge.get(up.id);
                e.seg.nextId = towards != null ? towards.seg.id : up.id;
            }
        }
        for (Chamber ch : scene.chambers) {
            TopoNode n = byId.get(ch.id);
            ch.incidentCount = n == null ? 0 : n.edges.size();
            int maxDn = 0;
            if (n != null) {
                for (TopoEdge e : n.edges) {
                    maxDn = Math.max(maxDn, e.seg.dn);
                }
                if (ch.nextId == null || ch.nextId.isBlank()) {
                    TopoEdge up = prevEdge.get(n.id);
                    if (up != null) {
                        ch.nextId = up.seg.id;
                    }
                }
            }
            if (ch.dn <= 0) {
                ch.dn = maxDn;
            }
        }
    }

    private static void add(List<TopoNode> nodes, Map<String, TopoNode> byId, TopoNode n) {
        nodes.add(n);
        byId.put(n.id, n);
    }

    private static TopoNode closest(List<TopoNode> nodes, Coordinate c, double snap) {
        TopoNode best = null;
        double bestD = snap;
        for (TopoNode n : nodes) {
            double d = n.c.distance(c);
            if (d <= bestD) {
                bestD = d;
                best = n;
            }
        }
        return best;
    }

    private static final class TopoNode {
        final String id;
        final String kind;
        final Coordinate c;
        final List<TopoEdge> edges = new ArrayList<>();

        TopoNode(String id, String kind, Coordinate c) {
            this.id = id;
            this.kind = kind;
            this.c = new Coordinate(c);
        }
    }

    private static final class TopoEdge {
        final ExistingSegment seg;
        final TopoNode a;
        final TopoNode b;
        final double length;

        TopoEdge(ExistingSegment seg, TopoNode a, TopoNode b, double length) {
            this.seg = seg;
            this.a = a;
            this.b = b;
            this.length = length;
        }
    }

    private static final class Dist {
        final String id;
        final double d;

        Dist(String id, double d) {
            this.id = id;
            this.d = d;
        }
    }
}
