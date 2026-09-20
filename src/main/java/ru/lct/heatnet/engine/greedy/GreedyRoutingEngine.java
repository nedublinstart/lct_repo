package ru.lct.heatnet.engine.greedy;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.ProgressListener;
import ru.lct.heatnet.engine.RoutingEngine;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;

public class GreedyRoutingEngine implements RoutingEngine {

    @Override
    public List<Variant> route(Scene scene, AppendixModel appendix, CalculationMode mode, ProgressListener progress) {
        progress.progress(15, "Индексирую препятствия");
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);
        progress.progress(25, "Строю каркас улиц");
        StreetFrame frame = StreetFrame.build(obstacles, scene);
        NetworkSnapper snapper = new NetworkSnapper(scene, appendix);

        List<Variant> variants = new ArrayList<>();
        progress.progress(35, "Вариант 1: раздельное подключение");
        variants.add(independent(scene, appendix, frame, obstacles, snapper, false,
                "independent", "Раздельное подключение",
                "Каждый ОКС идёт к ближайшей точке существующей сети своей трассой."));

        progress.progress(55, "Вариант 2: совместное подключение");
        variants.add(joint(scene, appendix, frame, obstacles, snapper,
                "joint", "Совместное подключение",
                "ОКС связываются деревом (MST) и одним врезом садятся на существующую сеть."));

        progress.progress(75, "Вариант 3: врезка в камеры");
        variants.add(independent(scene, appendix, frame, obstacles, snapper, true,
                "chambers", "Врезка в существующие камеры",
                "Предпочитаем существующие тепловые камеры, а не врезку в середину трубы."));
        return variants;
    }

    private Variant independent(Scene scene, AppendixModel appendix, StreetFrame frame, ObstacleIndex obstacles,
                                NetworkSnapper snapper, boolean chambersOnly,
                                String code, String title, String description) {
        Variant v = base(code, title, description);
        AtomicInteger ids = new AtomicInteger(1);
        for (ProspectiveOks oks : scene.oks) {
            if (oks.connection == null || oks.flowTph <= 0) {
                v.unconnectedOks.add(oks.id);
                v.notes.add("Нет точки подключения или расхода: " + oks.id);
                continue;
            }
            NetworkSnapper.Snap snap = snapper.snap(oks.connection, chambersOnly);
            if (snap == null && chambersOnly) {
                snap = snapper.snap(oks.connection, false);
            }
            if (snap == null) {
                v.unconnectedOks.add(oks.id);
                continue;
            }
            List<Coordinate> path = frame.find(oks.connection.getCoordinate(), snap.coordinate);
            if (path == null) {
                v.unconnectedOks.add(oks.id);
                v.notes.add("Маршрут не найден для " + oks.id);
                continue;
            }
            path = PathSmoother.collapseColinear(path, obstacles);
            addPipeAndTap(v, appendix, obstacles, ids, oks.id, oks.flowTph, path, snap);
        }
        return v;
    }

    private Variant joint(Scene scene, AppendixModel appendix, StreetFrame frame, ObstacleIndex obstacles,
                          NetworkSnapper snapper, String code, String title, String description) {
        List<ProspectiveOks> oks = new ArrayList<>();
        Variant v = base(code, title, description);
        for (ProspectiveOks o : scene.oks) {
            if (o.connection != null && o.flowTph > 0) {
                oks.add(o);
            } else {
                v.unconnectedOks.add(o.id);
            }
        }
        if (oks.isEmpty()) {
            return v;
        }
        if (oks.size() == 1) {
            return independent(scene, appendix, frame, obstacles, snapper, false, code, title, description);
        }
        int n = oks.size();
        int net = n;
        double[][] w = new double[n + 1][n + 1];
        List<Coordinate>[][] paths = new List[n + 1][n + 1];
        NetworkSnapper.Snap[] snaps = new NetworkSnapper.Snap[n];
        for (int i = 0; i < n; i++) {
            Arrays.fill(w[i], Double.POSITIVE_INFINITY);
        }
        Arrays.fill(w[net], Double.POSITIVE_INFINITY);
        for (int i = 0; i < n; i++) {
            NetworkSnapper.Snap snap = snapper.snap(oks.get(i).connection, false);
            snaps[i] = snap;
            if (snap == null) {
                continue;
            }
            List<Coordinate> path = frame.find(oks.get(i).connection.getCoordinate(), snap.coordinate);
            if (path == null) {
                continue;
            }
            path = PathSmoother.collapseColinear(path, obstacles);
            paths[i][net] = path;
            paths[net][i] = path;
            w[i][net] = length(path);
            w[net][i] = w[i][net];
        }
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                List<Coordinate> path = frame.find(oks.get(i).connection.getCoordinate(), oks.get(j).connection.getCoordinate());
                if (path == null) {
                    continue;
                }
                path = PathSmoother.collapseColinear(path, obstacles);
                paths[i][j] = path;
                paths[j][i] = path;
                w[i][j] = length(path);
                w[j][i] = w[i][j];
            }
        }
        int[] parent = prim(w, net);
        AtomicInteger ids = new AtomicInteger(1);
        double[] edgeFlow = new double[n + 1];
        for (int i = 0; i < n; i++) {
            if (parent[i] < 0 && paths[i][net] == null) {
                v.unconnectedOks.add(oks.get(i).id);
                continue;
            }
            int cur = i;
            while (cur != net && cur >= 0) {
                edgeFlow[cur] += oks.get(i).flowTph;
                cur = parent[cur] >= 0 ? parent[cur] : net;
                if (cur == i) {
                    break;
                }
            }
        }
        for (int i = 0; i < n; i++) {
            int p = parent[i];
            if (p < 0) {
                p = net;
            }
            List<Coordinate> path = paths[i][p];
            if (path == null) {
                if (!v.unconnectedOks.contains(oks.get(i).id)) {
                    v.unconnectedOks.add(oks.get(i).id);
                }
                continue;
            }
            double flow = Math.max(edgeFlow[i], oks.get(i).flowTph);
            if (p == net) {
                NetworkSnapper.Snap snap = snaps[i];
                if (snap == null) {
                    v.unconnectedOks.add(oks.get(i).id);
                    continue;
                }
                addPipeAndTap(v, appendix, obstacles, ids, oks.get(i).id, flow, path, snap);
            } else {
                addPipe(v, appendix, obstacles, ids, oks.get(i).id, oks.get(p).id, flow, path);
            }
        }
        return v;
    }

    private static int[] prim(double[][] w, int start) {
        int n = w.length;
        boolean[] used = new boolean[n];
        double[] dist = new double[n];
        int[] parent = new int[n];
        Arrays.fill(dist, Double.POSITIVE_INFINITY);
        Arrays.fill(parent, -1);
        dist[start] = 0;
        for (int it = 0; it < n; it++) {
            int v = -1;
            for (int i = 0; i < n; i++) {
                if (!used[i] && (v < 0 || dist[i] < dist[v])) {
                    v = i;
                }
            }
            if (v < 0 || dist[v] == Double.POSITIVE_INFINITY) {
                break;
            }
            used[v] = true;
            for (int u = 0; u < n; u++) {
                if (!used[u] && w[v][u] < dist[u]) {
                    dist[u] = w[v][u];
                    parent[u] = v;
                }
            }
        }
        return parent;
    }

    private void addPipeAndTap(Variant v, AppendixModel appendix, ObstacleIndex obstacles, AtomicInteger ids,
                               String oksId, double flow, List<Coordinate> path, NetworkSnapper.Snap snap) {
        addPipe(v, appendix, obstacles, ids, oksId, snap.objectId, flow, path);
        TapPoint tap = new TapPoint();
        tap.id = "TAP-" + ids.getAndIncrement();
        tap.geometryMeters = snap.point;
        tap.existingObjectId = snap.objectId;
        tap.existingObjectKind = snap.kind;
        tap.extraFlowTph = flow;
        tap.cost = "chamber".equals(snap.kind) || "source".equals(snap.kind)
                ? appendix.getCosts().tapInChamber
                : appendix.getCosts().tapInPipe;
        v.taps.add(tap);
    }

    private NewSegment addPipe(Variant v, AppendixModel appendix, ObstacleIndex obstacles, AtomicInteger ids,
                               String fromId, String toId, double flow, List<Coordinate> path) {
        return PipeEmitter.emit(v, obstacles, ids, fromId, toId, flow, path);
    }

    private static double length(List<Coordinate> path) {
        double s = 0;
        for (int i = 1; i < path.size(); i++) {
            s += path.get(i - 1).distance(path.get(i));
        }
        return s;
    }

    private static Variant base(String code, String title, String description) {
        Variant v = new Variant();
        v.code = code;
        v.title = title;
        v.description = description;
        v.segments.sort(Comparator.comparing(s -> s.id == null ? "" : s.id));
        return v;
    }
}
