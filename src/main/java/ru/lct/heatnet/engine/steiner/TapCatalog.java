package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.linearref.LengthIndexedLine;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.costing.DiameterSelector;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.HeatSource;
import ru.lct.heatnet.scene.Scene;

public final class TapCatalog {

    private final List<TapCandidate> all;
    private final Map<String, ExistingSegment> segs;
    private final Map<String, Chamber> chambers;
    private final Set<String> sources;
    private final Map<String, String> next;
    private final DiameterSelector diameters = new DiameterSelector();

    private TapCatalog(List<TapCandidate> all, Map<String, ExistingSegment> segs, Map<String, Chamber> chambers,
                       Set<String> sources, Map<String, String> next) {
        this.all = all;
        this.segs = segs;
        this.chambers = chambers;
        this.sources = sources;
        this.next = next;
    }

    public static TapCatalog build(Scene scene, AppendixModel appendix, ObstacleIndex obstacles) {
        Map<String, ExistingSegment> segs = new HashMap<>();
        Map<String, Chamber> chambers = new HashMap<>();
        Set<String> sources = new HashSet<>();
        Map<String, String> next = new HashMap<>();
        for (ExistingSegment s : scene.segments) {
            segs.put(s.id, s);
            if (s.nextId != null) {
                next.put(s.id, s.nextId);
            }
        }
        for (Chamber c : scene.chambers) {
            chambers.put(c.id, c);
            if (c.nextId != null) {
                next.put(c.id, c.nextId);
            }
        }
        for (HeatSource s : scene.sources) {
            sources.add(s.id);
        }
        List<TapCandidate> list = new ArrayList<>();
        int maxDeg = appendix.getRouting().maxChamberDegree;
        for (Chamber ch : scene.chambers) {
            if (ch.incidentCount >= maxDeg) {
                continue;
            }
            TapCandidate t = TapCandidate.chamber(ch, incomingFlow(ch, scene));
            if (!obstacles.blocked(t.coordinate)) {
                list.add(t);
            }
        }
        double step = Math.max(20, appendix.getRouting().candidateStepM);
        for (ExistingSegment seg : scene.segments) {
            double len = seg.line.getLength();
            int n = Math.max(1, (int) Math.round(len / step));
            LengthIndexedLine lil = new LengthIndexedLine(seg.line);
            for (int i = 0; i <= n; i++) {
                Coordinate c = lil.extractPoint(len * i / n);
                boolean nearChamber = false;
                for (Chamber ch : scene.chambers) {
                    if (ch.point.getCoordinate().distance(c) <= appendix.getRouting().chamberSnapM) {
                        nearChamber = true;
                        break;
                    }
                }
                if (!nearChamber && !obstacles.blocked(c)) {
                    list.add(TapCandidate.segment(seg, c));
                }
            }
        }
        return new TapCatalog(list, segs, chambers, sources, next);
    }

    public List<TapCandidate> all() {
        return all;
    }

    public List<TapCandidate> shortlist(Coordinate mean, double flow, Strategy strategy, AppendixModel appendix) {
        List<TapCandidate> ranked = new ArrayList<>(all);
        ranked.sort(Comparator.comparingDouble(t -> score(t, mean, flow, strategy, appendix)));
        int keep = Math.max(strategy.tapShortlist, 8);
        if (ranked.size() <= keep) {
            return ranked;
        }
        List<TapCandidate> out = new ArrayList<>(ranked.subList(0, keep));
        List<TapCandidate> spare = new ArrayList<>(all);
        spare.sort(Comparator.comparingDouble((TapCandidate t) -> -t.spare(capacity(t.existingDn, appendix)))
                .thenComparingDouble(t -> t.coordinate.distance(mean)));
        for (int i = 0; i < Math.min(6, spare.size()); i++) {
            TapCandidate t = spare.get(i);
            if (!contains(out, t)) {
                out.add(t);
            }
        }
        return out;
    }

    public double reconLength(TapCandidate tap, double extraFlow, AppendixModel appendix) {
        if (tap == null || extraFlow <= 1e-9) {
            return 0;
        }
        String cur = tap.existingId;
        if (tap.chamber) {
            Chamber ch = chambers.get(cur);
            cur = ch != null ? ch.nextId : next.get(cur);
        }
        double recon = 0;
        int guard = 0;
        while (cur != null && !sources.contains(cur) && guard++ < 10_000) {
            ExistingSegment seg = segs.get(cur);
            if (seg != null) {
                double total = seg.existingFlowTph + extraFlow;
                int required = diameters.select(total, appendix);
                if (required > seg.dn) {
                    recon += seg.line.getLength();
                }
                cur = next.get(cur);
                continue;
            }
            Chamber ch = chambers.get(cur);
            cur = ch != null ? ch.nextId : next.get(cur);
        }
        return recon;
    }

    public double reconPenalty(TapCandidate tap, double extraFlow, Strategy strategy, AppendixModel appendix) {
        return strategy.reconWeight * reconLength(tap, extraFlow, appendix);
    }

    private double score(TapCandidate tap, Coordinate mean, double flow, Strategy strategy, AppendixModel appendix) {
        double s = tap.coordinate.distance(mean);
        if (tap.chamber) {
            s *= 0.88;
        }
        double cap = capacity(tap.existingDn, appendix);
        double spare = tap.spare(cap);
        if (spare + 1e-9 < flow) {
            s += 80 + strategy.reconWeight * 25;
        } else {
            s -= Math.min(120, spare * 0.15 + tap.existingDn * 0.05);
        }
        s += reconPenalty(tap, flow, strategy, appendix);
        return s;
    }

    static double capacity(int dn, AppendixModel appendix) {
        if (dn <= 0) {
            return 0;
        }
        AppendixModel.DiameterSpec spec = appendix.diameter(dn);
        return spec == null ? 0 : spec.capacityTph;
    }

    private static double incomingFlow(Chamber ch, Scene scene) {
        double max = 0;
        for (ExistingSegment seg : scene.segments) {
            if (ch.id.equals(seg.nextId)) {
                max = Math.max(max, seg.existingFlowTph);
            }
        }
        return max;
    }

    private static boolean contains(List<TapCandidate> list, TapCandidate tap) {
        for (TapCandidate t : list) {
            if (t.key().equals(tap.key())) {
                return true;
            }
        }
        return false;
    }
}
