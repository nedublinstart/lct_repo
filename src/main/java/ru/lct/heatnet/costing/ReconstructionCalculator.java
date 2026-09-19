package ru.lct.heatnet.costing;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.ReconstructionChamber;
import ru.lct.heatnet.engine.ReconstructionSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.HeatSource;
import ru.lct.heatnet.scene.Scene;

public class ReconstructionCalculator {

    private final DiameterSelector diameters = new DiameterSelector();

    public void apply(Variant variant, Scene scene, AppendixModel appendix) {
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

        Map<String, Double> extra = new HashMap<>();
        for (TapPoint tap : variant.taps) {
            String cur = tap.existingObjectId;
            int guard = 0;
            while (cur != null && !sources.contains(cur) && guard++ < 10_000) {
                extra.merge(cur, tap.extraFlowTph, Double::sum);
                cur = next.get(cur);
            }
        }

        Set<String> reconChambers = new HashSet<>();
        for (ExistingSegment seg : scene.segments) {
            double add = extra.getOrDefault(seg.id, 0.0);
            if (add <= 1e-9) {
                continue;
            }
            int required = diameters.select(seg.existingFlowTph + add, appendix);
            if (required <= seg.dn) {
                continue;
            }
            ReconstructionSegment r = new ReconstructionSegment();
            r.id = seg.id;
            r.geometryMeters = seg.line;
            r.existingDn = seg.dn;
            r.requiredDn = required;
            r.extraFlowTph = add;
            r.lengthM = seg.line.getLength();
            r.cost = r.lengthM * appendix.getCosts().reconPipe(required);
            variant.reconstructionSegments.add(r);
            String nxt = seg.nextId;
            if (nxt != null && chambers.containsKey(nxt)) {
                reconChambers.add(nxt);
            }
        }
        for (TapPoint tap : variant.taps) {
            if ("chamber".equals(tap.existingObjectKind) && chambers.containsKey(tap.existingObjectId)) {
                reconChambers.add(tap.existingObjectId);
            }
        }
        for (String id : reconChambers) {
            Chamber ch = chambers.get(id);
            if (ch == null) {
                continue;
            }
            int required = 0;
            for (ReconstructionSegment r : variant.reconstructionSegments) {
                ExistingSegment s = segs.get(r.id);
                if (s != null && id.equals(s.nextId)) {
                    required = Math.max(required, r.requiredDn);
                }
            }
            if (required == 0) {
                required = diameters.select(extra.getOrDefault(id, 0.0), appendix);
            }
            ReconstructionChamber rc = new ReconstructionChamber();
            rc.id = id;
            rc.geometryMeters = ch.point;
            rc.existingDn = 0;
            rc.requiredDn = required;
            rc.cost = appendix.getCosts().reconChamber(Math.max(required, 150));
            variant.reconstructionChambers.add(rc);
        }
    }
}
