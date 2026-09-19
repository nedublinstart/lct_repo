package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.engine.greedy.PipeEmitter;
import ru.lct.heatnet.geo.GeoJsonGeometries;

public final class ForestEmitter {

    private final AppendixModel appendix;
    private final ObstacleIndex obstacles;
    private final AtomicInteger ids;
    private final Variant variant = new Variant();
    private final Map<String, String> chamberAt = new HashMap<>();

    public ForestEmitter(AppendixModel appendix, ObstacleIndex obstacles, AtomicInteger ids) {
        this.appendix = appendix;
        this.obstacles = obstacles;
        this.ids = ids;
    }

    public void emit(SteinerTree tree) {
        if (tree == null) {
            return;
        }
        for (OksPort p : tree.unconnected) {
            unconnected(p);
        }
        if (tree.failed()) {
            for (OksPort p : tree.connected) {
                unconnected(p);
            }
            return;
        }
        Map<Integer, String> nodeIds = new HashMap<>();
        String tapNode = attachTap(tree.tap, totalFlow(tree));
        for (SteinerTree.Node n : tree.nodes) {
            if (n.tap) {
                nodeIds.put(n.id, tapNode);
            } else if (n.port != null) {
                nodeIds.put(n.id, n.port.id());
            } else {
                nodeIds.put(n.id, chamber(n.at, false));
            }
        }
        for (SteinerTree.Branch b : tree.branches) {
            List<Coordinate> path = new ArrayList<>();
            SteinerTree.Node from = tree.nodes.get(b.from);
            if (from.port != null && from.port.origin != null && from.port.origin.distance(from.at) > 0.4) {
                path.add(new Coordinate(from.port.origin));
            }
            if (b.path != null) {
                path.addAll(b.path);
            }
            if (path.size() < 2) {
                continue;
            }
            String fromId = nodeIds.get(b.from);
            String toId = nodeIds.get(b.to);
            if (fromId == null || toId == null) {
                continue;
            }
            PipeEmitter.emit(variant, obstacles, ids, fromId, toId, Math.max(0.01, b.flow), path);
        }
    }

    public void unconnected(OksPort p) {
        variant.unconnectedOks.add(p.id());
        variant.unconnectedFlows.put(p.id(), p.flow());
        variant.notes.add("Маршрут не найден для ОКС " + p.id());
    }

    public Variant finish(Strategy strategy) {
        variant.code = strategy.code;
        variant.title = strategy.title;
        variant.description = strategy.description;
        return variant;
    }

    public Variant variant() {
        return variant;
    }

    private double totalFlow(SteinerTree tree) {
        double s = 0;
        for (OksPort p : tree.connected) {
            s += p.flow();
        }
        return s;
    }

    private String chamber(Coordinate c, boolean atTap) {
        String key = Math.round(c.x) + ":" + Math.round(c.y) + ":" + atTap;
        String existing = chamberAt.get(key);
        if (existing != null) {
            return existing;
        }
        NewChamber ch = new NewChamber();
        ch.id = "CH-" + ids.getAndIncrement();
        ch.geometryMeters = GeoJsonGeometries.GF.createPoint(snapChamber(c));
        ch.atTap = atTap;
        variant.chambers.add(ch);
        chamberAt.put(key, ch.id);
        return ch.id;
    }

    private Coordinate snapChamber(Coordinate c) {
        if (!obstacles.blocked(c) && !obstacles.inRoad(c)) {
            return new Coordinate(c);
        }
        return new Coordinate(c);
    }

    private String attachTap(TapCandidate tap, double flow) {
        for (TapPoint t : variant.taps) {
            if (t.existingObjectId.equals(tap.existingId)
                    && t.geometryMeters.getCoordinate().distance(tap.coordinate) < 1.0) {
                t.extraFlowTph += flow;
                return t.nodeId != null ? t.nodeId : t.id;
            }
        }
        String nodeId;
        if (tap.chamber) {
            nodeId = tap.existingId;
        } else {
            nodeId = chamber(tap.coordinate, true);
        }
        TapPoint t = new TapPoint();
        t.id = "TI-" + ids.getAndIncrement();
        t.nodeId = nodeId;
        t.geometryMeters = GeoJsonGeometries.GF.createPoint(tap.coordinate);
        t.existingObjectId = tap.existingId;
        t.existingObjectKind = tap.existingKind;
        t.existingDiameter = tap.existingDn;
        t.extraFlowTph = flow;
        t.cost = tap.chamber ? appendix.getCosts().tapInChamber : appendix.getCosts().tapInPipe;
        variant.taps.add(t);
        return nodeId;
    }
}
