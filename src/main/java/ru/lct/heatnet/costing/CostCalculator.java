package ru.lct.heatnet.costing;

import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.ReconstructionChamber;
import ru.lct.heatnet.engine.ReconstructionSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;

public class CostCalculator {

    public void apply(Variant variant, Scene scene, AppendixModel appendix) {
        apply(variant, appendix, scene);
    }

    public void apply(Variant variant, AppendixModel appendix) {
        apply(variant, appendix, null);
    }

    public void apply(Variant variant, AppendixModel appendix, Scene scene) {
        double pipes = 0;
        double newLen = 0;
        for (NewSegment seg : variant.segments) {
            double unit = appendix.newPerM(seg.dn);
            double kSpec = seg.kSpec > 0 ? seg.kSpec : ("special".equals(seg.layingMethod) ? appendix.getCosts().specialMul("special") : 1.0);
            double kDepth = 1.0;
            if (seg.depthM != null && seg.depthM > 3.0) {
                kDepth = 1.0 + appendix.getDepth().costFactorPerMDepth * (seg.depthM - 3.0);
            }
            double c = seg.lengthM * unit * kSpec * kDepth;
            seg.cost = c;
            pipes += c;
            newLen += seg.lengthM;
        }
        double chambers = 0;
        for (NewChamber ch : variant.chambers) {
            if (ch.dn <= 0) {
                int max = 0;
                for (NewSegment seg : variant.segments) {
                    if (ch.id.equals(seg.fromId) || ch.id.equals(seg.toId) || ch.atTap) {
                        max = Math.max(max, seg.dn);
                    }
                }
                ch.dn = max;
            }
            ch.cost = appendix.getCosts().chamber(Math.max(ch.dn, 50));
            chambers += ch.cost;
        }
        double taps = 0;
        for (TapPoint tap : variant.taps) {
            tap.cost = appendix.getCosts().tapInPipe;
            if (tap.requiredDiameter <= 0) {
                int req = 0;
                for (NewSegment seg : variant.segments) {
                    if (connects(tap, seg)) {
                        req = Math.max(req, seg.dn);
                    }
                }
                tap.requiredDiameter = req;
            }
            taps += tap.cost;
        }
        double recon = 0;
        double reconLen = 0;
        for (ReconstructionSegment r : variant.reconstructionSegments) {
            recon += r.cost;
            reconLen += r.lengthM;
        }
        double reconCh = 0;
        for (ReconstructionChamber r : variant.reconstructionChambers) {
            reconCh += r.cost;
        }
        double penalty = 0;
        for (String oksId : variant.unconnectedOks) {
            double g = variant.unconnectedFlows.getOrDefault(oksId, 0.0);
            if (g <= 0 && scene != null) {
                for (ProspectiveOks o : scene.oks) {
                    if (oksId.equals(o.id)) {
                        g = o.flowTph;
                    }
                }
            }
            penalty += appendix.getCosts().unconnectedFixed + appendix.getCosts().unconnectedPerTph * g;
        }
        variant.newLengthM = newLen;
        variant.reconLengthM = reconLen;
        variant.constructionCost = pipes;
        variant.penalty = penalty;
        variant.totalCost = pipes + chambers + taps + recon + reconCh + penalty;
        variant.costBreakdown.put("construction_cost", pipes);
        variant.costBreakdown.put("chamber_construction_cost", chambers);
        variant.costBreakdown.put("tie_in_cost", taps);
        variant.costBreakdown.put("reconstruction_cost", recon);
        variant.costBreakdown.put("chamber_reconstruction_cost", reconCh);
        variant.costBreakdown.put("unconnected_penalty", penalty);
        variant.costBreakdown.put("calculated_cost", variant.totalCost);
    }

    private static boolean connects(TapPoint tap, NewSegment seg) {
        if (seg.toId == null) {
            return false;
        }
        if (seg.toId.equals(tap.nodeId) || seg.toId.equals(tap.id) || seg.toId.equals(tap.existingObjectId)) {
            return true;
        }
        return tap.nodeId != null && (seg.fromId != null && seg.fromId.equals(tap.nodeId));
    }
}
