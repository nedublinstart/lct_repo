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
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.StreetFrame;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;

/**
 * TZ-стек: скелет улиц (∥/⊥ осям дорог, без решётки) → двоичная кластеризация
 * → Mehlhorn Steiner → объединение леса без дублей.
 */
@Component
public class SteinerRoutingEngine implements RoutingEngine {

    private static final Logger log = LoggerFactory.getLogger(SteinerRoutingEngine.class);

    @Override
    public List<Variant> route(Scene scene, AppendixModel appendix, CalculationMode mode, ProgressListener progress) {
        progress.progress(18, "Индексирую препятствия");
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);
        progress.progress(24, "Строю каркас улиц по осям дорог");
        StreetFrame frame = StreetFrame.build(obstacles, scene);
        Map<String, Coordinate> portsAt = ports(scene, obstacles, frame);
        progress.progress(34, "Дерево Штейнера на каркасе");
        MetricPathCache cache = new MetricPathCache(frame, obstacles);
        TapCatalog catalog = TapCatalog.build(scene, appendix, obstacles);

        List<OksPort> ports = new ArrayList<>();
        for (ProspectiveOks o : scene.oks) {
            if (o.connection == null || o.flowTph <= 0) {
                continue;
            }
            Coordinate at = portsAt.getOrDefault(o.id, o.connection.getCoordinate());
            ports.add(new OksPort(o, o.connection.getCoordinate(), at));
        }
        progress.progress(38, "Прогреваю пути к сети");
        warmup(ports, catalog, cache, progress);

        AtomicInteger ids = new AtomicInteger(1);
        int maxDeg = appendix.getRouting().maxChamberDegree;
        List<Variant> variants = new ArrayList<>();
        Set<String> seen = new HashSet<>();
        int[] marks = {55, 70, 85};
        int i = 0;
        for (Strategy strategy : Strategy.values()) {
            progress.progress(marks[Math.min(i, marks.length - 1)], strategy.title);
            Variant v = build(strategy, ports, catalog, cache, obstacles, appendix, ids, maxDeg, frame, scene);
            if (v != null && seen.add(fingerprint(v))) {
                variants.add(v);
            }
            i++;
        }
        if (variants.size() < 3) {
            progress.progress(90, "Запасной вариант: раздельные врезки");
            Variant indep = independent(ports, catalog, cache, obstacles, appendix, ids, maxDeg, frame, scene);
            if (indep != null && seen.add(fingerprint(indep))) {
                variants.add(indep);
            }
        }
        log.info("Вариантов: {}, ОКС: {}", variants.size(), ports.size());
        return variants;
    }

    private Variant build(Strategy strategy, List<OksPort> ports, TapCatalog catalog, PathMetric cache,
                          ObstacleIndex obstacles, AppendixModel appendix, AtomicInteger ids, int maxDeg,
                          StreetFrame frame, Scene scene) {
        ForestEmitter emitter = new ForestEmitter(appendix, obstacles, ids);
        if (ports.isEmpty()) {
            return emitter.finish(strategy);
        }
        OksClusterer clusterer = new OksClusterer(cache, catalog, strategy, appendix);
        List<OksClusterer.Cluster> clusters = clusterer.cluster(ports);
        DegreeBoard degrees = new DegreeBoard(maxDeg);
        Map<String, Double> extra = new HashMap<>();
        List<OksPort> leftover = new ArrayList<>();
        for (OksClusterer.Cluster cluster : clusters) {
            SteinerTree tree = bestTree(cluster, catalog, cache, strategy, appendix, degrees, maxDeg, false, extra);
            if (tree != null && !tree.failed()) {
                degrees.attach(tree.tap, Math.max(1, tree.tapChildren));
                catalog.commit(tree.tap, connectedFlow(tree), extra);
                leftover.addAll(emitter.emit(tree));
            } else {
                leftover.addAll(cluster.members);
            }
        }
        leftover = retrySingletons(leftover, catalog, cache, strategy, appendix, degrees, maxDeg, emitter, extra);
        unifyForest(emitter, obstacles, ids, ports);
        ItpSnapper.snap(emitter.variant(), obstacles, ids, ports, frame);
        ItpSnapper.consolidate(emitter.variant(), obstacles, ids, ports, frame, appendix);
        ItpSnapper.retapIfCheaper(emitter.variant(), obstacles, ids, ports, frame, appendix, catalog);
        ForestCompactor.compact(emitter.variant(), obstacles, ids);
        ItpSnapper.straightenStubs(emitter.variant(), obstacles, ports);
        SteinerForestUnifier.stitchToExisting(emitter.variant(), scene, obstacles, frame, ids, ports);
        ItpSnapper.snap(emitter.variant(), obstacles, ids, ports, frame);
        ForestCompactor.compact(emitter.variant(), obstacles, ids);
        SteinerForestUnifier.stitchToExisting(emitter.variant(), scene, obstacles, frame, ids, ports);
        ItpSnapper.straightenStubs(emitter.variant(), obstacles, ports);
        SteinerForestUnifier.dropDuplicateTaps(emitter.variant());
        Set<String> connected = connectedOks(emitter.variant());
        for (OksPort p : ports) {
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
                                          int maxDeg, ForestEmitter emitter, Map<String, Double> extra) {
        List<OksPort> failed = new ArrayList<>();
        Set<String> seen = new HashSet<>();
        Set<String> already = connectedOks(emitter.variant());
        for (OksPort p : leftover) {
            if (!seen.add(p.id()) || already.contains(p.id())) {
                continue;
            }
            OksClusterer.Cluster leaf = OksClusterer.Cluster.leaf(p);
            SteinerTree tree = bestTree(leaf, catalog, cache, strategy, appendix, degrees, maxDeg, true, extra);
            if (tree == null || tree.failed()) {
                failed.add(p);
                continue;
            }
            degrees.attach(tree.tap, Math.max(1, tree.tapChildren));
            catalog.commit(tree.tap, p.flow(), extra);
            failed.addAll(emitter.emit(tree));
            already.add(p.id());
        }
        return failed;
    }

    private static void unifyForest(ForestEmitter emitter, ObstacleIndex obstacles, AtomicInteger ids,
                                    List<OksPort> ports) {
        Set<String> oksIds = new HashSet<>();
        Map<String, Double> oksFlow = new HashMap<>();
        for (OksPort p : ports) {
            oksIds.add(p.id());
            oksFlow.put(p.id(), p.flow());
        }
        SteinerForestUnifier.unify(emitter.variant(), obstacles, ids, oksIds, oksFlow);
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
                                 boolean allTaps, Map<String, Double> extra) {
        Map<String, Double> already = extra == null ? Map.of() : extra;
        List<TapCandidate> local = allTaps
                ? new ArrayList<>(catalog.all())
                : catalog.shortlist(cluster.centroid, cluster.flow, strategy, appendix, already);
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
            double connected = connectedFlow(tree);
            double pathM = Double.isFinite(tree.length) && tree.length > 0 ? tree.length : tree.cost;
            double score = catalog.money(tap, connected > 1e-9 ? connected : cluster.flow, pathM, already, strategy, appendix)
                    + 100_000_000.0 * tree.unconnected.size();
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
            return bestTree(cluster, catalog, cache, strategy, appendix, degrees, maxDeg, true, extra);
        }
        return bestPartial;
    }

    private Variant independent(List<OksPort> ports, TapCatalog catalog, PathMetric cache,
                                ObstacleIndex obstacles, AppendixModel appendix, AtomicInteger ids, int maxDeg,
                                StreetFrame frame, Scene scene) {
        ForestEmitter emitter = new ForestEmitter(appendix, obstacles, ids);
        DegreeBoard degrees = new DegreeBoard(maxDeg);
        Map<String, Double> extra = new HashMap<>();
        List<OksPort> ordered = new ArrayList<>(ports);
        ordered.sort(Comparator.comparingDouble((OksPort p) -> -p.flow()));
        for (OksPort p : ordered) {
            SteinerTree tree = bestTree(OksClusterer.Cluster.leaf(p), catalog, cache, Strategy.MIN_COST,
                    appendix, degrees, maxDeg, true, extra);
            if (tree == null || tree.failed()) {
                emitter.unconnected(p);
                continue;
            }
            degrees.attach(tree.tap, Math.max(1, tree.tapChildren));
            catalog.commit(tree.tap, p.flow(), extra);
            emitter.emit(tree);
        }
        unifyForest(emitter, obstacles, ids, ports);
        ItpSnapper.snap(emitter.variant(), obstacles, ids, ports, frame);
        ItpSnapper.consolidate(emitter.variant(), obstacles, ids, ports, frame, appendix);
        ItpSnapper.retapIfCheaper(emitter.variant(), obstacles, ids, ports, frame, appendix, catalog);
        ForestCompactor.compact(emitter.variant(), obstacles, ids);
        ItpSnapper.straightenStubs(emitter.variant(), obstacles, ports);
        SteinerForestUnifier.stitchToExisting(emitter.variant(), scene, obstacles, frame, ids, ports);
        ItpSnapper.snap(emitter.variant(), obstacles, ids, ports, frame);
        ForestCompactor.compact(emitter.variant(), obstacles, ids);
        SteinerForestUnifier.stitchToExisting(emitter.variant(), scene, obstacles, frame, ids, ports);
        ItpSnapper.straightenStubs(emitter.variant(), obstacles, ports);
        SteinerForestUnifier.dropDuplicateTaps(emitter.variant());
        Set<String> connected = connectedOks(emitter.variant());
        for (OksPort p : ports) {
            if (!connected.contains(p.id())) {
                emitter.unconnected(p);
            }
        }
        Variant v = emitter.finish(Strategy.MIN_COST);
        v.code = "independent";
        v.title = "Раздельные врезки";
        v.description = "Каждый ОКС идёт к ближайшей достижимой точке существующей сети своей трассой.";
        return v;
    }

    private void warmup(List<OksPort> ports, TapCatalog catalog, PathMetric cache, ProgressListener progress) {
        int total = Math.max(1, ports.size());
        for (int p = 0; p < ports.size(); p++) {
            if (p == 0 || p == ports.size() - 1 || p % 3 == 0) {
                progress.progress(38 + Math.min(12, (12 * p) / total),
                        "Прогреваю пути " + (p + 1) + "/" + total);
            }
            OksPort port = ports.get(p);
            List<TapCandidate> near = new ArrayList<>(catalog.all());
            near.sort(Comparator.comparingDouble(t -> t.coordinate.distance(port.at)));
            int n = Math.min(4, near.size());
            for (int i = 0; i < n; i++) {
                cache.find(port.at, near.get(i).coordinate);
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

    private Map<String, Coordinate> ports(Scene scene, ObstacleIndex obstacles, StreetFrame frame) {
        Map<String, Coordinate> ports = new HashMap<>();
        for (ProspectiveOks o : scene.oks) {
            if (o.connection == null) {
                continue;
            }
            Coordinate origin = o.connection.getCoordinate();
            Coordinate goal = nearestNetwork(scene, origin);
            Coordinate at = obstacles.exitToStreet(origin, goal, 2.2);
            Coordinate seed = at != null ? at : origin;
            Coordinate local = frame.attachNear(seed, origin);
            Coordinate chosen = local != null ? local : seed;
            if (local != null && goal != null && frame.find(local, goal) == null) {
                Coordinate alt = obstacles.exitToStreet(origin, goal, 3.2);
                Coordinate altAt = alt == null ? null : frame.attachNear(alt, origin);
                if (altAt != null && frame.find(altAt, goal) != null
                        && origin.distance(altAt) <= origin.distance(local) + 12) {
                    chosen = altAt;
                }
            }
            ports.put(o.id, chosen);
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

    private static double connectedFlow(SteinerTree tree) {
        return flowOf(tree.connected);
    }

    private static double flowOf(List<OksPort> ports) {
        if (ports == null || ports.isEmpty()) {
            return 0;
        }
        double s = 0;
        for (OksPort p : ports) {
            s += p.flow();
        }
        return s;
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
