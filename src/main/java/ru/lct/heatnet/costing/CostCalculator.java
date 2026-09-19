package ru.lct.heatnet.costing;

import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.ReconstructionChamber;
import ru.lct.heatnet.engine.ReconstructionSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;

public class CostCalculator {

    public void apply(Variant variant, AppendixModel appendix) {
        double pipes = 0;
        double special = 0;
        double newLen = 0;
        for (NewSegment seg : variant.segments) {
            double unit = appendix.getCosts().pipePerM(seg.dn);
            double mul = appendix.getCosts().specialMul(seg.layingMethod);
            double c = seg.lengthM * unit * mul;
            if (seg.depthM != null && seg.depthM > 0) {
                c *= 1.0 + appendix.getDepth().costFactorPerMDepth * seg.depthM;
            }
            seg.cost = c;
            pipes += c;
            newLen += seg.lengthM;
            if (seg.specialReason != null) {
                special += c * 0.05;
            }
        }
        double chambers = 0;
        for (NewChamber ch : variant.chambers) {
            if (ch.dn <= 0 && !variant.segments.isEmpty()) {
                ch.dn = variant.segments.get(0).dn;
            }
            ch.cost = appendix.getCosts().chamber(Math.max(ch.dn, 80));
            chambers += ch.cost;
        }
        double taps = 0;
        for (TapPoint tap : variant.taps) {
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
        double penalty = variant.unconnectedOks.size() * appendix.getCosts().unconnectedPenalty;
        variant.newLengthM = newLen;
        variant.reconLengthM = reconLen;
        variant.constructionCost = pipes + special + chambers + taps + recon + reconCh;
        variant.penalty = penalty;
        variant.totalCost = variant.constructionCost + penalty;
        variant.costBreakdown.put("new_pipes", pipes);
        variant.costBreakdown.put("special_pass", special);
        variant.costBreakdown.put("new_chambers", chambers);
        variant.costBreakdown.put("taps", taps);
        variant.costBreakdown.put("reconstruction_pipes", recon);
        variant.costBreakdown.put("reconstruction_chambers", reconCh);
        variant.costBreakdown.put("unconnected_penalty", penalty);
    }
}
