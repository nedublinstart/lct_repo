package ru.lct.heatnet.costing;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.NewSegment;
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
            if (isChamber(tap.existingObjectKind)) {
                Chamber ch = chambers.get(cur);
                cur = ch != null ? ch.nextId : next.get(cur);
            }
            int guard = 0;
            while (cur != null && !sources.contains(cur) && guard++ < 10_000) {
                extra.merge(cur, tap.extraFlowTph, Double::sum);
                cur = next.get(cur);
            }
        }

        for (ExistingSegment seg : scene.segments) {
            double add = extra.getOrDefault(seg.id, 0.0);
            if (add <= 1e-9) {
                continue;
            }
            double total = seg.existingFlowTph + add;
            int required = diameters.select(total, appendix);
            if (required <= seg.dn) {
                continue;
            }
            ReconstructionSegment r = new ReconstructionSegment();
            r.id = "RE-" + seg.id;
            r.geometryMeters = seg.line;
            r.existingDn = seg.dn;
            r.requiredDn = required;
            r.existingFlowTph = seg.existingFlowTph;
            r.extraFlowTph = add;
            r.calculatedFlowTph = total;
            r.lengthM = seg.line.getLength();
            r.cost = r.lengthM * appendix.reconPerM(required);
            variant.reconstructionSegments.add(r);
        }

        for (TapPoint tap : variant.taps) {
            if (!isChamber(tap.existingObjectKind)) {
                continue;
            }
            Chamber ch = chambers.get(tap.existingObjectId);
            if (ch == null) {
                continue;
            }
            int required = Math.max(ch.dn, tap.requiredDiameter);
            for (NewSegment seg : variant.segments) {
                if (touchesTap(tap, seg)) {
                    required = Math.max(required, seg.dn);
                }
            }
            for (ReconstructionSegment r : variant.reconstructionSegments) {
                ExistingSegment s = segs.get(r.id.replaceFirst("^RE-", ""));
                if (s != null && tap.existingObjectId.equals(s.nextId)) {
                    required = Math.max(required, r.requiredDn);
                }
            }
            if (required <= ch.dn) {
                continue;
            }
            ReconstructionChamber rc = new ReconstructionChamber();
            rc.id = "RC-" + ch.id;
            rc.geometryMeters = ch.point;
            rc.existingDn = ch.dn;
            rc.requiredDn = required;
            rc.cost = appendix.getCosts().reconChamber(required);
            variant.reconstructionChambers.add(rc);
        }
    }

    private static boolean isChamber(String kind) {
        return "heat_chamber".equals(kind) || "chamber".equals(kind);
    }

    /** Только трубы, которые физически приходят в эту врезку, не весь вариант. */
    private static boolean touchesTap(TapPoint tap, NewSegment seg) {
        if (tap == null || seg == null) {
            return false;
        }
        return same(tap.id, seg.fromId) || same(tap.id, seg.toId)
                || same(tap.nodeId, seg.fromId) || same(tap.nodeId, seg.toId)
                || same(tap.existingObjectId, seg.fromId) || same(tap.existingObjectId, seg.toId);
    }

    private static boolean same(String a, String b) {
        return a != null && a.equals(b);
    }
}
