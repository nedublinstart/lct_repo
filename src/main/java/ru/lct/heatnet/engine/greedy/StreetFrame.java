package ru.lct.heatnet.engine.greedy;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.index.strtree.STRtree;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.simplify.DouglasPeuckerSimplifier;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import ru.lct.heatnet.engine.steiner.PathMetric;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.Scene;

/**
 * Каркас прокладки: рельсы вдоль тротуаров параллельно оси дороги, пересечения
 * перпендикулярно проезжей, точки привязки у фасадов. Кратчайший путь — Дейкстра
 * по этому графу (не A* по сетке). Геометрия уже ∥/⊥ улицам, без северных Г-шек.
 */
public final class StreetFrame implements PathMetric {

    private static final Logger log = LoggerFactory.getLogger(StreetFrame.class);

    static final double SNAP_M = 8.0;
    static final double MERGE_M = 3.2;
    static final double LINK_M = 24.0;
    static final double ALIGN_DEG = 14.0;
    static final double RING_OFFSET_M = 1.2;
    static final double SIDEWALK_IN_M = 1.8;
    static final double SHORT_STUB_M = 18.0;
    static final double ENDPOINT_M = 48.0;
    static final double ENDPOINT_EXT_M = 90.0;
    static final int MAX_NODES = 12_000;
    static final int MAX_ENDPOINT_LINKS = 18;
    static final double LATTICE_M = 12.0;

    private final ObstacleIndex obstacles;
    private final List<Coordinate> axes = new ArrayList<>();
    private final List<Coordinate> nodes = new ArrayList<>();
    private final List<List<Integer>> adj = new ArrayList<>();
    private final List<List<Double>> adjW = new ArrayList<>();
    private final Map<String, Integer> index = new HashMap<>();
    private final STRtree nodeTree = new STRtree();
    private int edgeCount;

    private StreetFrame(ObstacleIndex obstacles) {
        this.obstacles = obstacles;
    }

    public static StreetFrame build(ObstacleIndex obstacles, Scene scene) {
        StreetFrame frame = new StreetFrame(obstacles);
        Envelope env = scene == null ? null : scene.envelopeMeters;
        List<ExistingSegment> segs = scene == null ? List.of() : scene.segments;
        frame.buildGraph(env, segs);
        return frame;
    }

    public int nodeCount() {
        return nodes.size();
    }

    public int edgeCount() {
        return edgeCount;
    }

    public List<Coordinate> nodes() {
        List<Coordinate> copy = new ArrayList<>(nodes.size());
        for (Coordinate c : nodes) {
            copy.add(new Coordinate(c));
        }
        return copy;
    }

    public int largestComponentSize() {
        boolean[] seen = new boolean[nodes.size()];
        int best = 0;
        for (int i = 0; i < nodes.size(); i++) {
            if (seen[i] || i >= adj.size()) {
                continue;
            }
            int size = 0;
            ArrayDeque<Integer> q = new ArrayDeque<>();
            q.add(i);
            seen[i] = true;
            while (!q.isEmpty()) {
                int u = q.removeFirst();
                size++;
                if (u >= adj.size()) {
                    continue;
                }
                for (int v : adj.get(u)) {
                    if (!seen[v]) {
                        seen[v] = true;
                        q.add(v);
                    }
                }
            }
            if (size > best) {
                best = size;
            }
        }
        return best;
    }

    public int componentCount() {
        boolean[] seen = new boolean[nodes.size()];
        int parts = 0;
        for (int i = 0; i < nodes.size(); i++) {
            if (seen[i] || i >= adj.size() || adj.get(i).isEmpty()) {
                continue;
            }
            parts++;
            ArrayDeque<Integer> q = new ArrayDeque<>();
            q.add(i);
            seen[i] = true;
            while (!q.isEmpty()) {
                int u = q.removeFirst();
                for (int v : adj.get(u)) {
                    if (!seen[v]) {
                        seen[v] = true;
                        q.add(v);
                    }
                }
            }
        }
        return parts;
    }

    public List<Coordinate> axes() {
        List<Coordinate> copy = new ArrayList<>(axes.size());
        for (Coordinate c : axes) {
            copy.add(new Coordinate(c));
        }
        return copy;
    }

    @Override
    public List<Coordinate> find(Coordinate a, Coordinate b) {
        if (a == null || b == null) {
            return null;
        }
        if (a.distance(b) < 0.05) {
            return two(a, b);
        }
        if (frameChord(a, b)) {
            return two(a, b);
        }
        List<Coordinate> path = dijkstra(a, b);
        if (path != null && path.size() >= 2) {
            List<Coordinate> aligned = PathSmoother.collapseColinear(alignToAxes(path), obstacles);
            if (aligned != null && aligned.size() >= 2 && !obstacles.pathHitsAvoid(aligned, 1)) {
                return aligned;
            }
            return PathSmoother.collapseColinear(path, obstacles);
        }
        List<Coordinate> elbow = streetElbow(a, b);
        if (elbow != null) {
            return PathSmoother.collapseColinear(elbow, obstacles);
        }
        return null;
    }

    @Override
    public double cost(Coordinate a, Coordinate b) {
        List<Coordinate> path = find(a, b);
        if (path == null || path.size() < 2) {
            return Double.POSITIVE_INFINITY;
        }
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

    @Override
    public double length(Coordinate a, Coordinate b) {
        List<Coordinate> path = find(a, b);
        return OrthoPaths.length(path);
    }

    private void buildGraph(Envelope env, List<ExistingSegment> segs) {
        List<SpecialLayer.Corridor> corridors = obstacles.special().corridors();
        for (SpecialLayer.Corridor c : corridors) {
            addAxis(c.axis);
        }
        if (axes.isEmpty()) {
            for (Polygon p : obstacles.avoidPolygons()) {
                addAxis(polygonAxis(p));
            }
        }
        List<List<Coordinate>> rails = new ArrayList<>();
        for (SpecialLayer.Corridor c : corridors) {
            addRoadRails(c, rails);
        }
        addFacadeRails(rails);
        addExistingRails(segs, rails);
        if (corridors.isEmpty()) {
            addOpenLattice(env, rails);
        } else {
            addStreetLattice(env, rails);
        }
        for (List<Coordinate> rail : rails) {
            for (Coordinate p : rail) {
                nodeId(p);
            }
        }
        for (int i = 0; i < nodes.size(); i++) {
            adj.add(new ArrayList<>());
            adjW.add(new ArrayList<>());
            nodeTree.insert(new Envelope(nodes.get(i)), i);
        }
        nodeTree.build();
        for (List<Coordinate> rail : rails) {
            List<Integer> ids = new ArrayList<>();
            Coordinate prev = null;
            for (Coordinate p : rail) {
                if (p == null || (prev != null && prev.distance(p) < 0.4)) {
                    continue;
                }
                int id = existingId(p);
                if (id >= 0) {
                    ids.add(id);
                    prev = p;
                }
            }
            for (int i = 1; i < ids.size(); i++) {
                tryEdge(ids.get(i - 1), ids.get(i), false);
            }
        }
        linkNearby();
        pinRoadNodesToSidewalk();
        bridgeComponents();
        pruneIllegalEdges();
        log.info("Каркас улиц: {} узлов, {} рёбер, {} осей дорог, {} коридоров, {} компонент, крупнейшая {}",
                nodes.size(), edgeCount, axes.size(), corridors.size(), componentCount(), largestComponentSize());
    }

    private void addRoadRails(SpecialLayer.Corridor c, List<List<Coordinate>> rails) {
        if (c == null || c.geom == null || c.axis == null || c.origin == null) {
            return;
        }
        Coordinate u = unit(c.axis);
        Coordinate v = new Coordinate(-u.y, u.x);
        double tMin = 0;
        double tMax = 0;
        Coordinate[] pts = c.geom.getCoordinates();
        boolean any = false;
        for (Coordinate q : pts) {
            double t = (q.x - c.origin.x) * u.x + (q.y - c.origin.y) * u.y;
            if (!any) {
                tMin = t;
                tMax = t;
                any = true;
            } else {
                tMin = Math.min(tMin, t);
                tMax = Math.max(tMax, t);
            }
        }
        if (!any || tMax - tMin < 6) {
            return;
        }
        List<Coordinate> left = new ArrayList<>();
        List<Coordinate> right = new ArrayList<>();
        double step = SNAP_M;
        int n = Math.max(2, (int) Math.ceil((tMax - tMin) / step));
        for (int i = 0; i <= n; i++) {
            double t = tMin + (tMax - tMin) * i / n;
            Coordinate center = new Coordinate(c.origin.x + t * u.x, c.origin.y + t * u.y);
            Coordinate l = walkSidewalk(center, v, 1);
            Coordinate r = walkSidewalk(center, v, -1);
            if (l != null) {
                left.add(l);
            }
            if (r != null) {
                right.add(r);
            }
            if (l != null && r != null) {
                List<Coordinate> cross = new ArrayList<>(2);
                cross.add(l);
                cross.add(r);
                rails.add(cross);
            }
        }
        if (left.size() >= 2) {
            rails.add(left);
        }
        if (right.size() >= 2) {
            rails.add(right);
        }
    }

    private Coordinate walkSidewalk(Coordinate center, Coordinate v, int side) {
        if (center == null || v == null) {
            return null;
        }
        double max = Math.max(obstacles.sidewalkM() + 10, 14);
        for (double d = 0.4; d <= max; d += 0.45) {
            Coordinate q = new Coordinate(center.x + side * d * v.x, center.y + side * d * v.y);
            if (obstacles.blocked(q)) {
                return null;
            }
            if (!obstacles.inRoad(q)) {
                Coordinate extra = new Coordinate(
                        center.x + side * (d + SIDEWALK_IN_M) * v.x,
                        center.y + side * (d + SIDEWALK_IN_M) * v.y);
                if (!obstacles.blocked(extra) && !obstacles.inRoad(extra)) {
                    return extra;
                }
                return q;
            }
        }
        return null;
    }

    private void addFacadeRails(List<List<Coordinate>> rails) {
        double offset = RING_OFFSET_M;
        try {
            offset = Math.max(0.8, Math.min(1.6, obstacles.sidewalkM() * 0.35));
        } catch (RuntimeException ignored) {
        }
        for (Polygon p : obstacles.avoidPolygons()) {
            List<Coordinate> ring = facadeRing(p, offset);
            if (ring.size() >= 2) {
                rails.add(ring);
            }
        }
    }

    private List<Coordinate> facadeRing(Polygon polygon, double offset) {
        List<Coordinate> out = new ArrayList<>();
        Geometry source = polygon;
        try {
            Geometry inflated = polygon.buffer(offset, 2);
            if (inflated != null && !inflated.isEmpty()) {
                source = DouglasPeuckerSimplifier.simplify(inflated, 2.4);
            }
        } catch (RuntimeException e) {
            try {
                source = DouglasPeuckerSimplifier.simplify(polygon, 2.4);
            } catch (RuntimeException ignored) {
                source = polygon;
            }
        }
        List<Coordinate> raw = new ArrayList<>();
        if (source instanceof Polygon) {
            collectRing(((Polygon) source).getExteriorRing().getCoordinates(), raw);
        } else {
            for (int i = 0; i < source.getNumGeometries(); i++) {
                Geometry g = source.getGeometryN(i);
                if (g instanceof Polygon) {
                    collectRing(((Polygon) g).getExteriorRing().getCoordinates(), raw);
                }
            }
        }
        Coordinate prev = null;
        for (Coordinate c : densify(raw, SNAP_M)) {
            Coordinate q = pullToWalkable(c);
            if (q == null) {
                continue;
            }
            if (prev != null && prev.distance(q) < 0.6) {
                continue;
            }
            out.add(q);
            prev = q;
        }
        return out;
    }

    private Coordinate pullToWalkable(Coordinate c) {
        if (c == null) {
            return null;
        }
        if (!obstacles.blocked(c) && !obstacles.inRoad(c)) {
            return new Coordinate(c);
        }
        if (!axes.isEmpty()) {
            for (Coordinate axis : axes) {
                Coordinate v = new Coordinate(-axis.y, axis.x);
                for (int s : new int[]{1, -1}) {
                    for (double d = 0.6; d <= 6; d += 0.6) {
                        Coordinate q = new Coordinate(c.x + s * d * v.x, c.y + s * d * v.y);
                        if (!obstacles.blocked(q) && !obstacles.inRoad(q)) {
                            return q;
                        }
                    }
                }
            }
        }
        Coordinate free = obstacles.nearestFree(c, 10);
        if (free != null && !obstacles.blocked(free) && !obstacles.inRoad(free) && c.distance(free) <= 10) {
            return free;
        }
        return null;
    }

    private void addExistingRails(List<ExistingSegment> segs, List<List<Coordinate>> rails) {
        if (segs == null) {
            return;
        }
        for (ExistingSegment seg : segs) {
            if (seg == null || seg.line == null || seg.line.isEmpty()) {
                continue;
            }
            List<Coordinate> rail = new ArrayList<>();
            LineString ls = seg.line;
            LengthIndexedLine lil = new LengthIndexedLine(ls);
            double len = ls.getLength();
            int n = Math.max(1, (int) Math.ceil(len / SNAP_M));
            for (int i = 0; i <= n; i++) {
                Coordinate c = lil.extractPoint(len * i / n);
                Coordinate q = obstacles.inRoad(c) || obstacles.blocked(c) ? pullToWalkable(c) : new Coordinate(c);
                if (q == null && !obstacles.blocked(c)) {
                    q = new Coordinate(c);
                }
                if (q != null) {
                    rail.add(q);
                }
            }
            if (rail.size() >= 2) {
                rails.add(rail);
            }
        }
    }

    private void addStreetLattice(Envelope env, List<List<Coordinate>> rails) {
        Envelope box = env == null || env.isNull() ? boundsFromNodesAndAvoids() : new Envelope(env);
        if (box == null || box.isNull()) {
            return;
        }
        box.expandBy(24);
        double step = 10.0;
        int nx = Math.max(2, (int) Math.ceil(box.getWidth() / step) + 1);
        int ny = Math.max(2, (int) Math.ceil(box.getHeight() / step) + 1);
        if (nx * ny > MAX_NODES) {
            double scale = Math.sqrt((nx * ny) / (double) MAX_NODES);
            nx = Math.max(2, (int) (nx / scale));
            ny = Math.max(2, (int) (ny / scale));
            step = Math.max(step, Math.max(box.getWidth() / Math.max(1, nx - 1),
                    box.getHeight() / Math.max(1, ny - 1)));
        }
        double stepX = box.getWidth() / Math.max(1, nx - 1);
        double stepY = box.getHeight() / Math.max(1, ny - 1);
        Coordinate[][] at = new Coordinate[nx][ny];
        for (int x = 0; x < nx; x++) {
            for (int y = 0; y < ny; y++) {
                Coordinate c = new Coordinate(box.getMinX() + x * stepX, box.getMinY() + y * stepY);
                if (obstacles.blocked(c) || obstacles.inRoad(c) || !nearStreet(c)) {
                    continue;
                }
                at[x][y] = c;
            }
        }
        for (int x = 0; x < nx; x++) {
            for (int y = 0; y < ny; y++) {
                if (at[x][y] == null) {
                    continue;
                }
                if (y + 1 < ny && at[x][y + 1] != null) {
                    rails.add(List.of(at[x][y], at[x][y + 1]));
                }
                if (x + 1 < nx && at[x + 1][y] != null) {
                    rails.add(List.of(at[x][y], at[x + 1][y]));
                }
            }
        }
    }

    private void addOpenLattice(Envelope env, List<List<Coordinate>> rails) {
        Envelope box = env == null || env.isNull() ? boundsFromNodesAndAvoids() : new Envelope(env);
        if (box == null || box.isNull()) {
            return;
        }
        box.expandBy(36);
        int nx = Math.max(2, (int) Math.ceil(box.getWidth() / LATTICE_M) + 1);
        int ny = Math.max(2, (int) Math.ceil(box.getHeight() / LATTICE_M) + 1);
        if (nx * ny > MAX_NODES) {
            double scale = Math.sqrt((nx * ny) / (double) MAX_NODES);
            nx = Math.max(2, (int) (nx / scale));
            ny = Math.max(2, (int) (ny / scale));
        }
        double stepX = box.getWidth() / Math.max(1, nx - 1);
        double stepY = box.getHeight() / Math.max(1, ny - 1);
        int[][] id = new int[nx][ny];
        Coordinate[][] at = new Coordinate[nx][ny];
        for (int x = 0; x < nx; x++) {
            for (int y = 0; y < ny; y++) {
                Coordinate c = new Coordinate(box.getMinX() + x * stepX, box.getMinY() + y * stepY);
                if (obstacles.blocked(c) || obstacles.inRoad(c)) {
                    id[x][y] = -1;
                    continue;
                }
                id[x][y] = 1;
                at[x][y] = c;
            }
        }
        for (int x = 0; x < nx; x++) {
            List<Coordinate> col = new ArrayList<>();
            for (int y = 0; y < ny; y++) {
                if (id[x][y] < 0) {
                    if (col.size() >= 2) {
                        rails.add(new ArrayList<>(col));
                    }
                    col.clear();
                } else {
                    col.add(at[x][y]);
                }
            }
            if (col.size() >= 2) {
                rails.add(col);
            }
        }
        for (int y = 0; y < ny; y++) {
            List<Coordinate> row = new ArrayList<>();
            for (int x = 0; x < nx; x++) {
                if (id[x][y] < 0) {
                    if (row.size() >= 2) {
                        rails.add(new ArrayList<>(row));
                    }
                    row.clear();
                } else {
                    row.add(at[x][y]);
                }
            }
            if (row.size() >= 2) {
                rails.add(row);
            }
        }
    }

    private void pruneIllegalEdges() {
        for (int i = 0; i < adj.size(); i++) {
            List<Integer> next = adj.get(i);
            List<Double> ww = adjW.get(i);
            for (int k = next.size() - 1; k >= 0; k--) {
                int j = next.get(k);
                if (j < i) {
                    continue;
                }
                if (!obstacles.segmentHitsAvoid(nodes.get(i), nodes.get(j), 0, true)) {
                    continue;
                }
                next.remove(k);
                ww.remove(k);
                List<Integer> back = adj.get(j);
                int ix = back.indexOf(i);
                if (ix >= 0) {
                    back.remove(ix);
                    adjW.get(j).remove(ix);
                    edgeCount--;
                }
            }
        }
    }

    private void pinRoadNodesToSidewalk() {
        for (int i = 0; i < nodes.size(); i++) {
            Coordinate a = nodes.get(i);
            if (!obstacles.inRoad(a)) {
                continue;
            }
            Envelope q = new Envelope(a);
            q.expandBy(40);
            List<Integer> near = copyHits(nodeTree.query(q));
            int best = -1;
            double bestD = 40;
            for (int j : near) {
                if (j == i) {
                    continue;
                }
                Coordinate b = nodes.get(j);
                if (obstacles.inRoad(b) || obstacles.blocked(b)) {
                    continue;
                }
                double d = a.distance(b);
                if (d >= bestD) {
                    continue;
                }
                if (obstacles.segmentHitsAvoid(a, b, OrthoPaths.HIT_WIDTH_M, true)) {
                    continue;
                }
                bestD = d;
                best = j;
            }
            if (best < 0) {
                continue;
            }
            double w = obstacles.travelCost(a, nodes.get(best));
            if (!Double.isFinite(w)) {
                w = bestD * 1.6;
            }
            addUndirected(i, best, w);
        }
    }

    private void bridgeComponents() {
        int[] comp = componentIds();
        double reach = 36;
        for (int i = 0; i < nodes.size(); i++) {
            Coordinate a = nodes.get(i);
            Envelope q = new Envelope(a);
            q.expandBy(reach);
            List<Integer> near = copyHits(nodeTree.query(q));
            if (near.isEmpty()) {
                continue;
            }
            near.sort(Comparator.comparingDouble(j -> nodes.get(j).distance(a)));
            int checked = 0;
            for (int j : near) {
                if (j <= i || comp[i] == comp[j]) {
                    continue;
                }
                if (checked++ > 10) {
                    break;
                }
                Coordinate b = nodes.get(j);
                double d = a.distance(b);
                if (d < 1.0 || d > reach) {
                    continue;
                }
                if (obstacles.segmentHitsAvoid(a, b, OrthoPaths.HIT_WIDTH_M, true)) {
                    continue;
                }
                Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
                if (obstacles.inRoad(mid) && !headingOk(a, b)) {
                    continue;
                }
                boolean shortJoin = d <= 16 && nearStreet(a) && nearStreet(b) && !obstacles.inRoad(mid);
                if (shortJoin || alongStreet(a, b) || headingOk(a, b)) {
                    tryEdge(i, j, true);
                }
            }
        }
    }

    private int[] componentIds() {
        int[] id = new int[nodes.size()];
        Arrays.fill(id, -1);
        int c = 0;
        for (int i = 0; i < nodes.size(); i++) {
            if (id[i] >= 0) {
                continue;
            }
            ArrayDeque<Integer> q = new ArrayDeque<>();
            q.add(i);
            id[i] = c;
            while (!q.isEmpty()) {
                int u = q.removeFirst();
                if (u >= adj.size()) {
                    continue;
                }
                for (int v : adj.get(u)) {
                    if (id[v] < 0) {
                        id[v] = c;
                        q.add(v);
                    }
                }
            }
            c++;
        }
        return id;
    }

    private Envelope boundsFromNodesAndAvoids() {
        Envelope env = new Envelope();
        for (Polygon p : obstacles.avoidPolygons()) {
            env.expandToInclude(p.getEnvelopeInternal());
        }
        return env;
    }

    private void linkNearby() {
        double cross = obstacles.maxStreetEdgeM() + 8;
        double reach = Math.max(LINK_M, Math.min(cross, 64));
        for (int i = 0; i < nodes.size(); i++) {
            Coordinate a = nodes.get(i);
            Envelope q = new Envelope(a);
            q.expandBy(reach);
            List<Integer> near = copyHits(nodeTree.query(q));
            if (near.isEmpty()) {
                continue;
            }
            near.sort(Comparator.comparingDouble(j -> nodes.get(j).distance(a)));
            int checked = 0;
            for (int j : near) {
                if (j <= i) {
                    continue;
                }
                if (checked++ > 28) {
                    break;
                }
                Coordinate b = nodes.get(j);
                double d = a.distance(b);
                if (d < 0.8 || d > reach) {
                    continue;
                }
                if (d <= LINK_M) {
                    tryEdge(i, j, nearStreet(a) && nearStreet(b));
                    continue;
                }
                SpecialLayer.Travel t = obstacles.special().inspect(a, b);
                if (t.allowed && t.special && t.crossingAngleDeg + 1e-6 >= OrthoPaths.PERP_MIN_DEG
                        && d <= cross && headingOk(a, b)) {
                    tryEdge(i, j, false);
                }
            }
        }
    }

    private void tryEdge(int i, int j, boolean stubOk) {
        if (i == j || i < 0 || j < 0 || i >= nodes.size() || j >= nodes.size()) {
            return;
        }
        if (i < adj.size() && adj.get(i).contains(j)) {
            return;
        }
        Coordinate a = nodes.get(i);
        Coordinate b = nodes.get(j);
        if (!walkableEdge(a, b, stubOk)) {
            return;
        }
        double w = obstacles.travelCost(a, b);
        if (!Double.isFinite(w)) {
            if (alongStreet(a, b) || (stubOk && nearStreet(a) && nearStreet(b))) {
                w = a.distance(b);
            } else {
                return;
            }
        }
        addUndirected(i, j, w);
    }

    private void addUndirected(int i, int j, double w) {
        if (i == j || i < 0 || j < 0 || i >= adj.size() || j >= adj.size()) {
            return;
        }
        if (adj.get(i).contains(j)) {
            return;
        }
        adj.get(i).add(j);
        adjW.get(i).add(w);
        adj.get(j).add(i);
        adjW.get(j).add(w);
        edgeCount++;
    }

    private boolean walkableEdge(Coordinate a, Coordinate b, boolean stubOk) {
        if (a == null || b == null) {
            return false;
        }
        double d = a.distance(b);
        if (d < 0.25) {
            return true;
        }
        if (obstacles.segmentHitsAvoid(a, b, OrthoPaths.HIT_WIDTH_M, true)) {
            return false;
        }
        if (d <= 6 && !obstacles.inRoad(new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5))) {
            return true;
        }
        Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        if (d <= 14 && nearStreet(a) && nearStreet(b) && !obstacles.inRoad(mid)) {
            return true;
        }
        if (alongStreet(a, b)) {
            return true;
        }
        if (alongFacade(a, b) && headingOk(a, b)) {
            return true;
        }
        boolean travelOk = obstacles.allowsTravel(a, b);
        if (stubOk && d <= LINK_M && nearStreet(a) && nearStreet(b)
                && !obstacles.inRoad(new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5))) {
            return true;
        }
        if (!travelOk) {
            return false;
        }
        SpecialLayer.Travel t = obstacles.special().inspect(a, b);
        if (t.allowed && t.special && t.crossingAngleDeg + 1e-6 >= OrthoPaths.PERP_MIN_DEG
                && d <= obstacles.maxStreetEdgeM() + 10 && headingOk(a, b)) {
            return true;
        }
        return headingOk(a, b) && d <= LINK_M;
    }

    boolean frameChord(Coordinate a, Coordinate b) {
        if (!OrthoPaths.legal(obstacles, a, b)) {
            return false;
        }
        double d = a.distance(b);
        if (d <= 12) {
            return true;
        }
        SpecialLayer.Travel t = obstacles.special().inspect(a, b);
        if (t.allowed && t.special && t.crossingAngleDeg + 1e-6 >= OrthoPaths.PERP_MIN_DEG
                && d <= obstacles.maxStreetEdgeM() + 8) {
            return true;
        }
        if (alongStreet(a, b) && d <= 120) {
            return true;
        }
        return alongFacade(a, b) && headingOk(a, b) && d <= 80;
    }

    boolean alongStreet(Coordinate a, Coordinate b) {
        if (a == null || b == null) {
            return false;
        }
        Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        if (obstacles.inRoad(mid)) {
            return false;
        }
        double reach = obstacles.sidewalkM() + 10;
        SpecialLayer.Corridor c = obstacles.special().nearestCorridor(mid, reach);
        if (c == null || c.axis == null) {
            return false;
        }
        double ang = SpecialLayer.crossingAngleDeg(a, b, c.axis);
        return ang <= ALIGN_DEG;
    }

    boolean nearStreet(Coordinate c) {
        if (c == null || obstacles.inRoad(c) || obstacles.blocked(c)) {
            return false;
        }
        return obstacles.special().nearestCorridor(c, obstacles.sidewalkM() + 12) != null;
    }

    private boolean alongFacade(Coordinate a, Coordinate b) {
        return obstacles.alongAvoid(a, b, OrthoPaths.FACADE_M);
    }

    boolean headingOk(Coordinate a, Coordinate b) {
        if (a == null || b == null) {
            return false;
        }
        if (axes.isEmpty()) {
            return OrthoPaths.nearlyAxis(a, b);
        }
        for (Coordinate axis : axes) {
            double ang = SpecialLayer.crossingAngleDeg(a, b, axis);
            if (ang <= ALIGN_DEG || ang >= 90.0 - ALIGN_DEG) {
                return true;
            }
        }
        return false;
    }

    private List<Coordinate> dijkstra(Coordinate start, Coordinate goal) {
        if (nodes.isEmpty()) {
            return null;
        }
        List<Link> startLinks = endpointLinks(start, ENDPOINT_M);
        List<Link> goalLinks = endpointLinks(goal, ENDPOINT_M);
        if (startLinks.isEmpty()) {
            startLinks = endpointLinks(start, ENDPOINT_EXT_M);
        }
        if (goalLinks.isEmpty()) {
            goalLinks = endpointLinks(goal, ENDPOINT_EXT_M);
        }
        if (startLinks.isEmpty()) {
            startLinks = forceLink(start);
        }
        if (goalLinks.isEmpty()) {
            goalLinks = forceLink(goal);
        }
        if (startLinks.isEmpty() || goalLinks.isEmpty()) {
            return null;
        }
        int n = nodes.size();
        int s = n;
        int g = n + 1;
        int total = n + 2;
        boolean[] toGoal = new boolean[n];
        double[] goalW = new double[n];
        for (Link link : goalLinks) {
            toGoal[link.to] = true;
            goalW[link.to] = link.w;
        }
        double[] dist = new double[total];
        int[] parent = new int[total];
        byte[] seen = new byte[total];
        Arrays.fill(dist, Double.POSITIVE_INFINITY);
        Arrays.fill(parent, -1);
        dist[s] = 0;
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingDouble(o -> o.d));
        pq.add(new Node(s, 0));
        int guard = 0;
        int limit = Math.max(8_000, nodes.size() * 8);
        while (!pq.isEmpty() && guard++ < limit) {
            Node cur = pq.poll();
            if (seen[cur.id] != 0) {
                continue;
            }
            seen[cur.id] = 1;
            if (cur.id == g) {
                break;
            }
            if (cur.d > dist[cur.id] + 1e-9) {
                continue;
            }
            if (cur.id == s) {
                for (Link link : startLinks) {
                    double nd = dist[s] + link.w;
                    if (nd + 1e-9 < dist[link.to]) {
                        dist[link.to] = nd;
                        parent[link.to] = s;
                        pq.add(new Node(link.to, nd));
                    }
                }
                continue;
            }
            if (cur.id < n) {
                List<Integer> next = adj.get(cur.id);
                List<Double> ww = adjW.get(cur.id);
                for (int k = 0; k < next.size(); k++) {
                    int v = next.get(k);
                    double nd = dist[cur.id] + ww.get(k);
                    if (nd + 1e-9 < dist[v]) {
                        dist[v] = nd;
                        parent[v] = cur.id;
                        pq.add(new Node(v, nd));
                    }
                }
                if (toGoal[cur.id]) {
                    double nd = dist[cur.id] + goalW[cur.id];
                    if (nd + 1e-9 < dist[g]) {
                        dist[g] = nd;
                        parent[g] = cur.id;
                        pq.add(new Node(g, nd));
                    }
                }
            }
        }
        if (!Double.isFinite(dist[g])) {
            return null;
        }
        List<Coordinate> path = new ArrayList<>();
        path.add(new Coordinate(goal));
        int cur = g;
        int hops = 0;
        while (cur != s && hops++ < total + 4) {
            int p = parent[cur];
            if (p < 0) {
                return null;
            }
            if (p != s) {
                path.add(new Coordinate(nodes.get(p)));
            }
            cur = p;
        }
        path.add(new Coordinate(start));
        Collections.reverse(path);
        return path;
    }

    private List<Link> endpointLinks(Coordinate p, double radius) {
        List<Link> out = new ArrayList<>();
        if (p == null || nodes.isEmpty()) {
            return out;
        }
        int snap = nearestWithin(p, MERGE_M + 1.5);
        if (snap >= 0) {
            double d = p.distance(nodes.get(snap));
            if (d < 0.4 || OrthoPaths.legal(obstacles, p, nodes.get(snap))) {
                out.add(new Link(snap, Math.max(0.4, d)));
            }
        }
        Envelope env = new Envelope(p);
        env.expandBy(radius);
        List<Integer> near = copyHits(nodeTree.query(env));
        if (near.isEmpty()) {
            return out;
        }
        near.sort(Comparator.comparingDouble(i -> nodes.get(i).distance(p)));
        for (int i : near) {
            if (out.size() >= MAX_ENDPOINT_LINKS) {
                break;
            }
            Coordinate q = nodes.get(i);
            double d = p.distance(q);
            if (d > radius) {
                continue;
            }
            boolean already = false;
            for (Link link : out) {
                if (link.to == i) {
                    already = true;
                    break;
                }
            }
            if (already) {
                continue;
            }
            if (d <= SHORT_STUB_M && OrthoPaths.legal(obstacles, p, q)
                    && !obstacles.segmentHitsAvoid(p, q, OrthoPaths.HIT_WIDTH_M, true)) {
                double w = obstacles.travelCost(p, q);
                out.add(new Link(i, Double.isFinite(w) ? w : d));
                continue;
            }
            if (d <= 28 && nearStreet(p) && nearStreet(q) && OrthoPaths.legal(obstacles, p, q)
                    && !obstacles.segmentHitsAvoid(p, q, OrthoPaths.HIT_WIDTH_M, true)) {
                double w = obstacles.travelCost(p, q);
                if (Double.isFinite(w)) {
                    out.add(new Link(i, w));
                }
                continue;
            }
            if (frameChord(p, q) || (headingOk(p, q) && OrthoPaths.legal(obstacles, p, q) && d <= radius)) {
                double w = obstacles.travelCost(p, q);
                if (Double.isFinite(w)) {
                    out.add(new Link(i, w));
                }
            }
        }
        return out;
    }

    private List<Link> forceLink(Coordinate p) {
        List<Link> out = new ArrayList<>();
        if (p == null || nodes.isEmpty()) {
            return out;
        }
        int best = -1;
        double bestD = ENDPOINT_EXT_M;
        for (int i = 0; i < nodes.size(); i++) {
            Coordinate q = nodes.get(i);
            if (obstacles.blocked(q)) {
                continue;
            }
            double d = p.distance(q);
            if (d > bestD) {
                continue;
            }
            if (obstacles.inRoad(q) && d > 10) {
                continue;
            }
            if (obstacles.segmentHitsAvoid(p, q, OrthoPaths.HIT_WIDTH_M, true)) {
                continue;
            }
            bestD = d;
            best = i;
        }
        if (best >= 0) {
            out.add(new Link(best, Math.max(0.8, bestD)));
        }
        return out;
    }

    private List<Coordinate> streetElbow(Coordinate a, Coordinate b) {
        Coordinate u = axisNear(a, b);
        u = unit(u);
        Coordinate v = new Coordinate(-u.y, u.x);
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double du = dx * u.x + dy * u.y;
        double dv = dx * v.x + dy * v.y;
        Coordinate alongU = new Coordinate(a.x + du * u.x, a.y + du * u.y);
        Coordinate alongV = new Coordinate(a.x + dv * v.x, a.y + dv * v.y);
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        for (Coordinate corner : new Coordinate[]{alongU, alongV}) {
            if (a.distance(corner) < 0.4 || b.distance(corner) < 0.4) {
                continue;
            }
            if (!OrthoPaths.legal(obstacles, a, corner) || !OrthoPaths.legal(obstacles, corner, b)) {
                continue;
            }
            if (!headingOk(a, corner) || !headingOk(corner, b)) {
                continue;
            }
            if (!frameChord(a, corner) && a.distance(corner) > SHORT_STUB_M) {
                continue;
            }
            if (!frameChord(corner, b) && corner.distance(b) > SHORT_STUB_M) {
                continue;
            }
            List<Coordinate> path = new ArrayList<>(3);
            path.add(new Coordinate(a));
            path.add(new Coordinate(corner));
            path.add(new Coordinate(b));
            double len = OrthoPaths.length(path);
            if (len < bestLen) {
                bestLen = len;
                best = path;
            }
        }
        return best;
    }

    private List<Coordinate> alignToAxes(List<Coordinate> path) {
        if (path == null || path.size() < 2) {
            return path;
        }
        List<Coordinate> out = new ArrayList<>();
        out.add(new Coordinate(path.get(0)));
        for (int i = 1; i < path.size(); i++) {
            Coordinate a = out.get(out.size() - 1);
            Coordinate b = path.get(i);
            if (a.distance(b) < 0.4) {
                continue;
            }
            if (headingOk(a, b) || a.distance(b) <= 8) {
                out.add(new Coordinate(b));
                continue;
            }
            List<Coordinate> elbow = streetElbow(a, b);
            if (elbow != null && elbow.size() >= 3) {
                for (int k = 1; k < elbow.size(); k++) {
                    Coordinate c = elbow.get(k);
                    if (out.get(out.size() - 1).distance(c) >= 0.4) {
                        out.add(new Coordinate(c));
                    }
                }
            } else {
                out.add(new Coordinate(b));
            }
        }
        return out;
    }

    private Coordinate axisNear(Coordinate a, Coordinate b) {
        Coordinate mid = new Coordinate((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        SpecialLayer.Corridor c = obstacles.special().nearestCorridor(mid, 80);
        if (c != null && c.axis != null) {
            return c.axis;
        }
        if (!axes.isEmpty()) {
            return axes.get(0);
        }
        return new Coordinate(1, 0);
    }

    private int nodeId(Coordinate c) {
        if (c == null || nodes.size() >= MAX_NODES) {
            return -1;
        }
        if (obstacles.blocked(c)) {
            return -1;
        }
        int near = nearestLinear(c, MERGE_M);
        if (near >= 0) {
            return near;
        }
        String key = Math.round(c.x / 2.0) + ":" + Math.round(c.y / 2.0);
        Integer existing = index.get(key);
        if (existing != null) {
            return existing;
        }
        int id = nodes.size();
        nodes.add(new Coordinate(c));
        index.put(key, id);
        return id;
    }

    private int existingId(Coordinate c) {
        int near = nearestLinear(c, MERGE_M + 0.4);
        if (near >= 0) {
            return near;
        }
        String key = Math.round(c.x / 2.0) + ":" + Math.round(c.y / 2.0);
        Integer existing = index.get(key);
        return existing == null ? -1 : existing;
    }

    private int nearestWithin(Coordinate c, double snapM) {
        if (c == null || nodes.isEmpty()) {
            return -1;
        }
        Envelope env = new Envelope(c);
        env.expandBy(snapM);
        List<Integer> near = copyHits(nodeTree.query(env));
        int best = -1;
        double bestD = snapM;
        if (near != null) {
            for (int i : near) {
                double d = nodes.get(i).distance(c);
                if (d <= bestD) {
                    bestD = d;
                    best = i;
                }
            }
        }
        return best;
    }

    private int nearestLinear(Coordinate c, double snapM) {
        int best = -1;
        double bestD = snapM;
        for (int i = 0; i < nodes.size(); i++) {
            double d = nodes.get(i).distance(c);
            if (d <= bestD) {
                bestD = d;
                best = i;
            }
        }
        return best;
    }

    private void addAxis(Coordinate raw) {
        if (raw == null) {
            return;
        }
        Coordinate u = unit(raw);
        if (u == null) {
            return;
        }
        if (u.x < -1e-9 || (Math.abs(u.x) < 1e-9 && u.y < 0)) {
            u = new Coordinate(-u.x, -u.y);
        }
        for (Coordinate a : axes) {
            double dot = Math.abs(a.x * u.x + a.y * u.y);
            if (dot >= Math.cos(Math.toRadians(10))) {
                return;
            }
        }
        axes.add(u);
    }

    private static Coordinate polygonAxis(Polygon p) {
        if (p == null || p.isEmpty()) {
            return null;
        }
        try {
            Geometry rect = new org.locationtech.jts.algorithm.MinimumDiameter(p).getMinimumRectangle();
            Coordinate[] c = rect.getCoordinates();
            double best = -1;
            Coordinate axis = null;
            for (int i = 0; i < c.length - 1; i++) {
                double dx = c[i + 1].x - c[i].x;
                double dy = c[i + 1].y - c[i].y;
                double n = Math.hypot(dx, dy);
                if (n > best) {
                    best = n;
                    if (n > 1e-6) {
                        axis = new Coordinate(dx / n, dy / n);
                    }
                }
            }
            return axis;
        } catch (RuntimeException e) {
            return null;
        }
    }

    private static Coordinate unit(Coordinate c) {
        if (c == null) {
            return new Coordinate(1, 0);
        }
        double n = Math.hypot(c.x, c.y);
        if (n < 1e-9) {
            return new Coordinate(1, 0);
        }
        return new Coordinate(c.x / n, c.y / n);
    }

    private static void collectRing(Coordinate[] ring, List<Coordinate> out) {
        if (ring == null || ring.length < 2) {
            return;
        }
        int n = ring.length;
        while (n >= 2 && ring[0].distance(ring[n - 1]) < 1e-6) {
            n--;
        }
        for (int i = 0; i < n; i++) {
            out.add(new Coordinate(ring[i]));
        }
        if (n >= 2) {
            out.add(new Coordinate(ring[0]));
        }
    }

    static List<Coordinate> densify(List<Coordinate> path, double step) {
        List<Coordinate> out = new ArrayList<>();
        if (path == null || path.isEmpty()) {
            return out;
        }
        out.add(new Coordinate(path.get(0)));
        for (int i = 1; i < path.size(); i++) {
            Coordinate a = path.get(i - 1);
            Coordinate b = path.get(i);
            double len = a.distance(b);
            int parts = Math.max(1, (int) Math.floor(len / step));
            for (int k = 1; k < parts; k++) {
                double t = k / (double) parts;
                out.add(new Coordinate(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
            }
            out.add(new Coordinate(b));
        }
        return out;
    }

    @SuppressWarnings("unchecked")
    private static List<Integer> copyHits(List<?> raw) {
        if (raw == null || raw.isEmpty()) {
            return new ArrayList<>();
        }
        return new ArrayList<>((List<Integer>) raw);
    }

    private static List<Coordinate> two(Coordinate a, Coordinate b) {
        List<Coordinate> out = new ArrayList<>(2);
        out.add(new Coordinate(a));
        out.add(new Coordinate(b));
        return out;
    }

    private static final class Link {
        final int to;
        final double w;

        Link(int to, double w) {
            this.to = to;
            this.w = w;
        }
    }

    private static final class Node {
        final int id;
        final double d;

        Node(int id, double d) {
            this.id = id;
            this.d = d;
        }
    }
}
