package ru.lct.heatnet.costing;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.NewSegment;

public class DiameterSelector {

    public int select(double flowTph, AppendixModel appendix) {
        List<AppendixModel.DiameterSpec> specs = new ArrayList<>(appendix.getDiameters());
        specs.sort(Comparator.comparingInt(s -> s.dn));
        for (AppendixModel.DiameterSpec spec : specs) {
            if (spec.capacityTph + 1e-9 >= flowTph) {
                return spec.dn;
            }
        }
        return specs.isEmpty() ? 150 : specs.get(specs.size() - 1).dn;
    }

    public void apply(List<NewSegment> segments, AppendixModel appendix) {
        for (NewSegment seg : segments) {
            int dn = select(seg.flowTph, appendix);
            AppendixModel.DiameterSpec spec = appendix.diameter(dn);
            while (spec != null && seg.lengthM > spec.maxRunM + 1e-6) {
                int next = bump(dn, appendix);
                if (next == dn) {
                    break;
                }
                dn = next;
                spec = appendix.diameter(dn);
            }
            seg.dn = dn;
        }
    }

    private int bump(int dn, AppendixModel appendix) {
        int best = dn;
        for (AppendixModel.DiameterSpec spec : appendix.getDiameters()) {
            if (spec.dn > dn && (best == dn || spec.dn < best)) {
                best = spec.dn;
            }
        }
        return best;
    }
}
