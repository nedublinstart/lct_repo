package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import org.locationtech.jts.geom.Coordinate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.ProgressListener;
import ru.lct.heatnet.engine.RoutingEngine;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.greedy.GridPathfinder;
import ru.lct.heatnet.engine.greedy.NetworkSnapper;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.VisibilityPathfinder;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;

/**
 * TZ-стек: видимый граф → кластеризация PCST-style → Mehlhorn Steiner → потоки/диаметры снаружи.
 */
@Component
public class SteinerRoutingEngine implements RoutingEngine {

    private static final Logger log = LoggerFactory.getLogger(SteinerRoutingEngine.class);

    @Override
    public List<Variant> route(Scene scene, AppendixModel appendix, CalculationMode mode, ProgressListener progress) {
        progress.progress(18, "Индексирую препятствия");
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);
        Map<String, Coordinate> portsAt = ports(scene, obstacles);
        progress.progress(28, "Строю граф видимости");
        GridPathfinder grid = GridPathfinder.build(scene, appendix, obstacles);
        VisibilityPathfinder visibility = VisibilityPathfinder.build(obstacles);
        MetricPathCache cache = new MetricPathCache(grid, visibility, obstacles,
                appendix.getRouting().turnKeepDeg, scene.envelopeMeters);
        TapCatalog catalog = TapCatalog.build(scene, appendix, obstacles);

        List<OksPort> ports = new ArrayList<>();
        for (ProspectiveOks o : scene.oks) {
            if (o.connection == null || o.flowTph <= 0) {
                continue;
            }
            Coordinate at = portsAt.getOrDefault(o.id, o.connection.getCoordinate());
            ports.add(new OksPort(o, o.connection.getCoordinate(), at));
        }
        warmup(ports, catalog, cache);

        AtomicInteger ids = new AtomicInteger(1);
        int maxDeg = appendix.getRouting().maxChamberDegree;
        List<Variant> variants = new ArrayList<>();
        Set<String> seen = new HashSet<>();
        int[] marks = {55, 70, 85};
        int i = 0;
        for (Strategy strategy : Strategy.values()) {
            progress.progress(marks[Math.min(i, marks.length - 1)], strategy.title);
            Variant v = build(strategy, ports, catalog, cache, obstacles, appendix, ids, maxDeg);
            if (v != null && seen.add(fingerprint(v))) {
                variants.add(v);
            }
            i++;
        }
        if (variants.size() < 3) {
            progress.progress(90, "Запасной вариант: раздельные врезки");
            Variant indep = independent(ports, scene, appendix, cache, obstacles, ids);
            if (indep != null && seen.add(fingerprint(indep))) {
                variants.add(indep);
            }
        }
        log.info("Вариантов: {}, ОКС: {}", variants.size(), ports.size());
        return variants;
    }

    private Variant build(Strategy strategy, List<OksPort> ports, TapCatalog catalog, PathMetric cache,
                          ObstacleIndex obstacles, AppendixModel appendix, AtomicInteger ids, int maxDeg) {
        ForestEmitter emitter = new ForestEmitter(appendix, obstacles, ids);
        if (ports.isEmpty()) {
            return emitter.finish(strategy);
        }
        OksClusterer clusterer = new OksClusterer(cache, catalog, strategy, appendix);
        List<OksClusterer.Cluster> clusters = clusterer.cluster(ports);
        DegreeBoard degrees = new DegreeBoard(maxDeg);
        List<OksPort> leftover = new ArrayList<>();
        for (OksClusterer.Cluster cluster : clusters) {
            SteinerTree tree = bestTree(cluster, catalog, cache, strategy, appendix, degrees, maxDeg, false);
            if (tree != null && !tree.failed() && tree.unconnected.isEmpty()) {
                degrees.attach(tree.tap, Math.max(1, tree.tapChildren));
                leftover.addAll(emitter.emit(tree));
            } else if (tree != null && !tree.failed()) {
                degrees.attach(tree.tap, Math.max(1, tree.tapChildren));
                leftover.addAll(emitter.emit(tree));
            } else {
                leftover.addAll(cluster.members);
            }
        }
        leftover = retrySingletons(leftover, catalog, cache, strategy, appendix, degrees, maxDeg, emitter);
        Set<String> connected = connectedOks(emitter.variant());
        for (OksPort p : leftover) {
            if (!connected.contains(p.id())) {
                emitter.unconnected(p);
            }
        }
        Variant v = emitter.finish(strategy);
        v.notes.add(0, "Кластеров: " + clusters.size() + ", врезок: " + v.taps.size());
        return v;
    }

    private List<OksPort> retrySingletons(List<OksPort> leftover, TapCatalog catalog, PathMetric cache,
                                          Strategy strategy, AppendixModel appendix, DegreeBoard degrees,
                                          int maxDeg, ForestEmitter emitter) {
        List<OksPort> failed = new ArrayList<>();
        Set<String> seen = new HashSet<>();
        Set<String> already = connectedOks(emitter.variant());
        for (OksPort p : leftover) {
            if (!seen.add(p.id()) || already.contains(p.id())) {
                continue;
            }
            OksClusterer.Cluster leaf = OksClusterer.Cluster.leaf(p);
            SteinerTree tree = bestTree(leaf, catalog, cache, strategy, appendix, degrees, maxDeg, true);
            if (tree == null || tree.failed()) {
                failed.add(p);
                continue;
            }
            degrees.attach(tree.tap, Math.max(1, tree.tapChildren));
            failed.addAll(emitter.emit(tree));
            already.add(p.id());
        }
        return failed;
    }

    private static Set<String> connectedOks(Variant variant) {
        Set<String> ids = new HashSet<>();
        for (ru.lct.heatnet.engine.NewSegment seg : variant.segments) {
            if (seg.fromId != null) {
                ids.add(seg.fromId);
            }
        }
        return ids;
    }

    private SteinerTree bestTree(OksClusterer.Cluster cluster, TapCatalog catalog, PathMetric cache,
                                 Strategy strategy, AppendixModel appendix, DegreeBoard degrees, int maxDeg,
                                 boolean allTaps) {
        List<TapCandidate> local = allTaps
                ? new ArrayList<>(catalog.all())
                : catalog.shortlist(cluster.centroid, cluster.flow, strategy, appendix);
        SteinerTree bestFull = null;
        SteinerTree bestPartial = null;
        double bestFullScore = Double.POSITIVE_INFINITY;
        double bestPartialScore = Double.POSITIVE_INFINITY;
        for (TapCandidate tap : local) {
            if (!degrees.canAttach(tap, 1)) {
                continue;
            }
            SteinerTree tree = MehlhornSteiner.connect(cluster.members, tap, cache, maxDeg);
            if (tree == null || tree.failed()) {
                continue;
            }
            if (!degrees.canAttach(tap, Math.max(1, tree.tapChildren))) {
                continue;
            }
            double recon = catalog.reconPenalty(tap, cluster.flow, strategy, appendix);
            double score = tree.cost + recon + strategy.tapFee + 50_000 * tree.unconnected.size();
            if (tree.unconnected.isEmpty()) {
                if (score < bestFullScore) {
                    bestFullScore = score;
                    bestFull = tree;
                }
            } else if (score < bestPartialScore) {
                bestPartialScore = score;
                bestPartial = tree;
            }
        }
        if (bestFull != null) {
            return bestFull;
        }
        if (!allTaps) {
            return bestTree(cluster, catalog, cache, strategy, appendix, degrees, maxDeg, true);
        }
        return bestPartial;
    }

    private Variant independent(List<OksPort> ports, Scene scene, AppendixModel appendix, PathMetric cache,
                                ObstacleIndex obstacles, AtomicInteger ids) {
        ForestEmitter emitter = new ForestEmitter(appendix, obstacles, ids);
        NetworkSnapper snapper = new NetworkSnapper(scene, appendix);
        DegreeBoard degrees = new DegreeBoard(appendix.getRouting().maxChamberDegree);
        for (OksPort p : ports) {
            NetworkSnapper.Snap snap = snapper.snap(p.oks.connection, false);
            if (snap == null) {
                emitter.unconnected(p);
                continue;
            }
            TapCandidate tap = TapCandidate.of(snap, 0, 0);
            if (!degrees.canAttach(tap, 1)) {
                emitter.unconnected(p);
                continue;
            }
            SteinerTree tree = MehlhornSteiner.connect(List.of(p), tap, cache, appendix.getRouting().maxChamberDegree);
            if (tree == null || tree.failed()) {
                emitter.unconnected(p);
                continue;
            }
            degrees.attach(tap, 1);
            emitter.emit(tree);
        }
        Variant v = emitter.finish(Strategy.MIN_COST);
        v.code = "independent";
        v.title = "Раздельные врезки";
        v.description = "Каждый ОКС идёт к ближайшей точке существующей сети своей трассой.";
        return v;
    }

    private void warmup(List<OksPort> ports, TapCatalog catalog, PathMetric cache) {
        for (OksPort p : ports) {
            List<TapCandidate> near = new ArrayList<>(catalog.all());
            near.sort(Comparator.comparingDouble(t -> t.coordinate.distance(p.at)));
            int n = Math.min(4, near.size());
            for (int i = 0; i < n; i++) {
                cache.find(p.at, near.get(i).coordinate);
            }
        }
        for (int i = 0; i < ports.size(); i++) {
            for (int j = i + 1; j < ports.size(); j++) {
                if (ports.get(i).at.distance(ports.get(j).at) <= 800) {
                    cache.find(ports.get(i).at, ports.get(j).at);
                }
            }
        }
    }

    private Map<String, Coordinate> ports(Scene scene, ObstacleIndex obstacles) {
        Map<String, Coordinate> ports = new HashMap<>();
        for (ProspectiveOks o : scene.oks) {
            if (o.connection == null) {
                continue;
            }
            Coordinate origin = o.connection.getCoordinate();
            Coordinate target = nearestNetwork(scene, origin);
            Coordinate at = obstacles.exitToStreet(origin, target, 1.6);
            ports.put(o.id, at);
        }
        return ports;
    }

    private static Coordinate nearestNetwork(Scene scene, Coordinate from) {
        Coordinate best = null;
        double bestD = Double.POSITIVE_INFINITY;
        for (ExistingSegment seg : scene.segments) {
            org.locationtech.jts.operation.distance.DistanceOp op =
                    new org.locationtech.jts.operation.distance.DistanceOp(seg.line, GeoJsonGeometries.GF.createPoint(from));
            Coordinate[] pts = op.nearestPoints();
            double d = from.distance(pts[0]);
            if (d < bestD) {
                bestD = d;
                best = pts[0];
            }
        }
        for (Chamber ch : scene.chambers) {
            double d = from.distance(ch.point.getCoordinate());
            if (d < bestD) {
                bestD = d;
                best = ch.point.getCoordinate();
            }
        }
        return best;
    }

    static String fingerprint(Variant v) {
        List<String> taps = new ArrayList<>();
        for (ru.lct.heatnet.engine.TapPoint t : v.taps) {
            Coordinate c = t.geometryMeters.getCoordinate();
            taps.add(t.existingObjectId + ":" + Math.round(c.x / 8) + ":" + Math.round(c.y / 8));
        }
        taps.sort(String::compareTo);
        List<String> lost = new ArrayList<>(v.unconnectedOks);
        lost.sort(String::compareTo);
        return String.join(",", taps) + "|" + String.join(",", lost) + "|" + Math.round(v.segments.size());
    }
}
