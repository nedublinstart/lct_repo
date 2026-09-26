package ru.lct.heatnet.costing;

import java.util.Comparator;
import java.util.List;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.Variant;

public class RankingCalculator {

    public void rank(List<Variant> variants, AppendixModel appendix) {
        VariantClaims.apply(variants);
        double cw = appendix.getRanking().costWeight;
        double lw = appendix.getRanking().lengthWeight;
        double costBase = appendix.getRanking().costBase > 0 ? appendix.getRanking().costBase : 25_000_000;
        double lengthBase = appendix.getRanking().lengthBase > 0 ? appendix.getRanking().lengthBase : 100;
        for (Variant v : variants) {
            double length = v.newLengthM;
            v.score = cw * (v.totalCost / costBase) + lw * (length / lengthBase);
        }
        variants.sort(Comparator.comparingDouble((Variant v) -> v.score)
                .thenComparingDouble(v -> v.totalCost)
                .thenComparing(v -> v.code == null ? "" : v.code));
    }
}
