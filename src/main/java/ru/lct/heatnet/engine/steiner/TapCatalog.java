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
        return shortlist(mean, flow, strategy, appendix, Map.of());
    }

    public List<TapCandidate> shortlist(Coordinate mean, double flow, Strategy strategy, AppendixModel appendix,
                                        Map<String, Double> already) {
        Map<String, Double> extra = already == null ? Map.of() : already;
        List<TapCandidate> ranked = new ArrayList<>(all);
        ranked.sort(Comparator.comparingDouble(t -> money(t, flow, t.coordinate.distance(mean), extra, strategy, appendix)));
        int keep = Math.max(strategy.tapShortlist, 8);
        List<TapCandidate> out = ranked.size() <= keep ? new ArrayList<>(ranked) : new ArrayList<>(ranked.subList(0, keep));
        List<TapCandidate> spare = new ArrayList<>(all);
        spare.sort(Comparator.comparingDouble((TapCandidate t) -> -t.spare(capacity(t.existingDn, appendix)))
                .thenComparingDouble(t -> t.coordinate.distance(mean)));
        for (int i = 0; i < Math.min(8, spare.size()); i++) {
            TapCandidate t = spare.get(i);
            if (!contains(out, t)) {
                out.add(t);
            }
        }
        addNearest(out, mean, 12, t -> t.existingDn >= 400);
        addNearest(out, mean, 12, t -> t.existingDn >= 500);
        List<TapCandidate> clean = new ArrayList<>();
        for (TapCandidate t : all) {
            if (reconRubles(t, flow, extra, appendix) <= 1.0) {
                clean.add(t);
            }
        }
        clean.sort(Comparator.comparingDouble(t -> t.coordinate.distance(mean)));
        for (int i = 0; i < Math.min(12, clean.size()); i++) {
            if (!contains(out, clean.get(i))) {
                out.add(clean.get(i));
            }
        }
        return out;
    }

    public double reconLength(TapCandidate tap, double extraFlow, AppendixModel appendix) {
        return reconLength(tap, extraFlow, Map.of(), appendix);
    }

    public double reconLength(TapCandidate tap, double extraFlow, Map<String, Double> already, AppendixModel appendix) {
        return walkRecon(tap, extraFlow, already, appendix, false);
    }

    public double reconRubles(TapCandidate tap, double extraFlow, AppendixModel appendix) {
        return reconRubles(tap, extraFlow, Map.of(), appendix);
    }

    public double reconRubles(TapCandidate tap, double extraFlow, Map<String, Double> already, AppendixModel appendix) {
        return walkRecon(tap, extraFlow, already, appendix, true);
    }

    public void commit(TapCandidate tap, double extraFlow, Map<String, Double> extra) {
        if (tap == null || extraFlow <= 1e-9 || extra == null) {
            return;
        }
        for (String id : upstreamIds(tap)) {
            extra.merge(id, extraFlow, Double::sum);
        }
    }

    /**
     * Оценка в рублях: новая труба по DN кластера + реконструкция с учётом уже посаженного расхода.
     */
    public double money(TapCandidate tap, double extraFlow, double pathM, Map<String, Double> already,
                        Strategy strategy, AppendixModel appendix) {
        int dn = diameters.select(Math.max(0.01, extraFlow), appendix);
        double pipes = Math.max(0, pathM) * appendix.newPerM(dn);
        double recon = reconRubles(tap, extraFlow, already == null ? Map.of() : already, appendix);
        if (strategy == Strategy.MIN_RECON) {
            recon *= 3.0;
        }
        double tapFee = strategy == Strategy.MIN_TAPS
                ? appendix.getCosts().tapInPipe * 5.0
                : appendix.getCosts().tapInPipe;
        return pipes + recon + tapFee;
    }

    public double reconPenalty(TapCandidate tap, double extraFlow, Strategy strategy, AppendixModel appendix) {
        return strategy.reconWeight * reconLength(tap, extraFlow, appendix);
    }

    private void addNearest(List<TapCandidate> out, Coordinate mean, int limit,
                            java.util.function.Predicate<TapCandidate> filter) {
        List<TapCandidate> pool = new ArrayList<>();
        for (TapCandidate t : all) {
            if (filter.test(t)) {
                pool.add(t);
            }
        }
        pool.sort(Comparator.comparingDouble(t -> t.coordinate.distance(mean)));
        for (int i = 0; i < Math.min(limit, pool.size()); i++) {
            if (!contains(out, pool.get(i))) {
                out.add(pool.get(i));
            }
        }
    }

    private double walkRecon(TapCandidate tap, double extraFlow, Map<String, Double> already,
                             AppendixModel appendix, boolean rubles) {
        if (tap == null || extraFlow <= 1e-9) {
            return 0;
        }
        double recon = 0;
        for (String id : upstreamIds(tap)) {
            ExistingSegment seg = segs.get(id);
            if (seg == null) {
                continue;
            }
            double total = seg.existingFlowTph + extraFlow + (already == null ? 0 : already.getOrDefault(id, 0.0));
            int required = diameters.select(total, appendix);
            if (required > seg.dn) {
                recon += rubles ? seg.line.getLength() * appendix.reconPerM(required) : seg.line.getLength();
            }
        }
        return recon;
    }

    private List<String> upstreamIds(TapCandidate tap) {
        List<String> ids = new ArrayList<>();
        String cur = tap.existingId;
        if (tap.chamber) {
            Chamber ch = chambers.get(cur);
            cur = ch != null ? ch.nextId : next.get(cur);
        }
        int guard = 0;
        while (cur != null && !sources.contains(cur) && guard++ < 10_000) {
            ExistingSegment seg = segs.get(cur);
            if (seg != null) {
                ids.add(cur);
                cur = next.get(cur);
                continue;
            }
            Chamber ch = chambers.get(cur);
            cur = ch != null ? ch.nextId : next.get(cur);
        }
        return ids;
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
