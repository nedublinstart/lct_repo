package ru.lct.heatnet.costing;

import java.util.Comparator;
import java.util.List;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.Variant;

public class RankingCalculator {

    public void rank(List<Variant> variants, AppendixModel appendix) {
        double cw = appendix.getRanking().costWeight;
        double lw = appendix.getRanking().lengthWeight;
        double k = appendix.getRanking().lengthToCost;
        for (Variant v : variants) {
            double length = v.newLengthM + v.reconLengthM;
            v.score = cw * v.totalCost + lw * length * k;
        }
        variants.sort(Comparator.comparingDouble((Variant v) -> v.score)
                .thenComparingDouble(v -> v.totalCost)
                .thenComparing(v -> v.code));
    }
}
