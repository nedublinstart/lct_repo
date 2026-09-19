package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.appendix.AppendixModel;

/**
 * Агломеративная кластеризация ОКС: объединяем, если cost_joint &lt; cost_separate (в рублях).
 */
public final class OksClusterer {

    private final PathMetric metric;
    private final TapCatalog taps;
    private final Strategy strategy;
    private final AppendixModel appendix;

    public OksClusterer(PathMetric metric, TapCatalog taps, Strategy strategy, AppendixModel appendix) {
        this.metric = metric;
        this.taps = taps;
        this.strategy = strategy;
        this.appendix = appendix;
    }

    public List<Cluster> cluster(List<OksPort> ports) {
        List<Cluster> clusters = new ArrayList<>();
        for (OksPort p : ports) {
            clusters.add(Cluster.leaf(p));
        }
        while (clusters.size() > 1) {
            int bestI = -1;
            int bestJ = -1;
            double bestSavings = strategy.minSavings;
            double bestDist = Double.POSITIVE_INFINITY;
            for (int i = 0; i < clusters.size(); i++) {
                for (int j = i + 1; j < clusters.size(); j++) {
                    Cluster a = clusters.get(i);
                    Cluster b = clusters.get(j);
                    double d = a.centroid.distance(b.centroid);
                    if (d > strategy.mergeRadiusM) {
                        continue;
                    }
                    if (metric.find(a.centroid, b.centroid) == null) {
                        continue;
                    }
                    if (strategy.forceMerge) {
                        if (d < bestDist) {
                            bestDist = d;
                            bestI = i;
                            bestJ = j;
                            bestSavings = 1;
                        }
                        continue;
                    }
                    double separate = costOf(a) + costOf(b);
                    double joint = unionCost(a, b);
                    if (!Double.isFinite(joint) || !Double.isFinite(separate)) {
                        continue;
                    }
                    double savings = separate - joint;
                    if (savings > bestSavings + 1e-6 || (Math.abs(savings - bestSavings) < 1e-6 && d < bestDist)) {
                        bestSavings = savings;
                        bestDist = d;
                        bestI = i;
                        bestJ = j;
                    }
                }
            }
            if (bestI < 0) {
                break;
            }
            Cluster merged = Cluster.merge(clusters.get(bestI), clusters.get(bestJ));
            if (bestI > bestJ) {
                clusters.remove(bestI);
                clusters.remove(bestJ);
            } else {
                clusters.remove(bestJ);
                clusters.remove(bestI);
            }
            clusters.add(merged);
        }
        clusters = fitCapacity(clusters);
        clusters.sort(Comparator.comparingDouble((Cluster c) -> -c.flow));
        return clusters;
    }

    public ScoredTap pickTap(Cluster cluster, DegreeBoard degrees) {
        List<TapCandidate> local = taps.shortlist(cluster.centroid, cluster.flow, strategy, appendix);
        ScoredTap best = null;
        double unit = appendix.newPerM(200);
        double tapFee = strategy == Strategy.MIN_TAPS
                ? appendix.getCosts().tapInPipe * 5.0
                : appendix.getCosts().tapInPipe;
        for (TapCandidate tap : local) {
            if (!degrees.canAttach(tap, 1)) {
                continue;
            }
            double mst = MehlhornSteiner.terminalMstLength(cluster.members, tap, metric);
            if (!Double.isFinite(mst)) {
                continue;
            }
            double recon = taps.reconRubles(tap, cluster.flow, appendix);
            if (strategy == Strategy.MIN_RECON) {
                recon *= 3.0;
            }
            double score = mst * unit + recon + tapFee;
            if (best == null || score < best.score) {
                best = new ScoredTap(tap, score, mst);
            }
        }
        return best;
    }

    public SteinerTree buildTree(Cluster cluster, TapCandidate tap, int maxDegree) {
        return MehlhornSteiner.connect(cluster.members, tap, metric, maxDegree);
    }

    private List<Cluster> fitCapacity(List<Cluster> clusters) {
        List<Cluster> out = new ArrayList<>();
        for (Cluster cluster : clusters) {
            out.addAll(splitIfNeeded(cluster));
        }
        return out;
    }

    /**
     * Если ближайшая точка сети не принимает весь расход без реконструкции,
     * ближайшие ОКС оставляем на ней, дальние — отдельным кластером на участок с запасом.
     */
    private List<Cluster> splitIfNeeded(Cluster cluster) {
        if (cluster.members.size() <= 1) {
            return List.of(cluster);
        }
        TapCandidate nearest = taps.nearest(cluster.centroid);
        if (nearest == null) {
            return List.of(cluster);
        }
        if (taps.reconRubles(nearest, cluster.flow, appendix) <= 1.0) {
            return List.of(cluster);
        }
        double cap = taps.spareOnWalk(nearest, appendix);
        if (cap <= 1e-6) {
            return List.of(cluster);
        }
        List<OksPort> remaining = new ArrayList<>(cluster.members);
        Coordinate at = nearest.coordinate;
        remaining.sort(Comparator.comparingDouble(p -> p.at.distance(at)));
        Cluster keep = new Cluster();
        Cluster overflow = new Cluster();
        for (OksPort p : remaining) {
            double next = keep.flow + p.flow();
            boolean fits = next <= cap + 1e-6 && taps.reconRubles(nearest, next, appendix) <= 1.0;
            if (keep.members.isEmpty() || fits) {
                keep.members.add(p);
                keep.flow = next;
            } else {
                overflow.members.add(p);
            }
        }
        refresh(keep);
        refresh(overflow);
        if (overflow.members.isEmpty() || overflow.members.size() == cluster.members.size()) {
            return List.of(cluster);
        }
        List<Cluster> parts = new ArrayList<>();
        parts.add(keep);
        parts.addAll(splitIfNeeded(overflow));
        return parts;
    }

    private static void refresh(Cluster cluster) {
        cluster.cachedCost = Double.NaN;
        cluster.cachedTap = null;
        cluster.flow = 0;
        if (cluster.members.isEmpty()) {
            return;
        }
        double x = 0;
        double y = 0;
        for (OksPort p : cluster.members) {
            cluster.flow += p.flow();
            x += p.at.x;
            y += p.at.y;
        }
        cluster.centroid = new Coordinate(x / cluster.members.size(), y / cluster.members.size());
    }

    private double costOf(Cluster cluster) {
        if (!Double.isNaN(cluster.cachedCost)) {
            return cluster.cachedCost;
        }
        ScoredTap picked = pickTap(cluster, DegreeBoard.unbounded());
        if (picked == null) {
            cluster.cachedCost = Double.POSITIVE_INFINITY;
            cluster.cachedTap = null;
            return cluster.cachedCost;
        }
        cluster.cachedCost = picked.score;
        cluster.cachedTap = picked.tap;
        return cluster.cachedCost;
    }

    private double unionCost(Cluster a, Cluster b) {
        Cluster u = Cluster.merge(a, b);
        ScoredTap picked = pickTap(u, DegreeBoard.unbounded());
        return picked == null ? Double.POSITIVE_INFINITY : picked.score;
    }

    public static final class Cluster {
        public final List<OksPort> members = new ArrayList<>();
        public Coordinate centroid = new Coordinate();
        public double flow;
        double cachedCost = Double.NaN;
        TapCandidate cachedTap;

        static Cluster leaf(OksPort p) {
            Cluster c = new Cluster();
            c.members.add(p);
            c.flow = p.flow();
            c.centroid = new Coordinate(p.at);
            return c;
        }

        static Cluster merge(Cluster a, Cluster b) {
            Cluster c = new Cluster();
            c.members.addAll(a.members);
            c.members.addAll(b.members);
            c.flow = a.flow + b.flow;
            c.centroid = new Coordinate(
                    (a.centroid.x * a.members.size() + b.centroid.x * b.members.size()) / c.members.size(),
                    (a.centroid.y * a.members.size() + b.centroid.y * b.members.size()) / c.members.size());
            return c;
        }
    }

    public static final class ScoredTap {
        public final TapCandidate tap;
        public final double score;
        public final double mst;

        ScoredTap(TapCandidate tap, double score, double mst) {
            this.tap = tap;
            this.score = score;
            this.mst = mst;
        }
    }
}
