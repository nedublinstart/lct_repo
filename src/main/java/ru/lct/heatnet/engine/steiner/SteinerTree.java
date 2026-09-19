package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;

public final class SteinerTree {
    public TapCandidate tap;
    public final List<OksPort> connected = new ArrayList<>();
    public final List<OksPort> unconnected = new ArrayList<>();
    public final List<Node> nodes = new ArrayList<>();
    public final List<Branch> branches = new ArrayList<>();
    public double cost;
    public double length;
    public int tapChildren;

    public static SteinerTree unconnected(List<OksPort> ports) {
        SteinerTree t = new SteinerTree();
        if (ports != null) {
            t.unconnected.addAll(ports);
        }
        t.cost = Double.POSITIVE_INFINITY;
        t.length = Double.POSITIVE_INFINITY;
        return t;
    }

    public boolean failed() {
        return connected.isEmpty() || !Double.isFinite(cost) || tap == null;
    }

    public static final class Node {
        public int id;
        public Coordinate at;
        public OksPort port;
        public boolean tap;
        public boolean junction;
    }

    public static final class Branch {
        public int from;
        public int to;
        public List<Coordinate> path;
        public double flow;
        public double cost;
    }
}
