package ru.lct.heatnet.engine.flow;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import ru.lct.heatnet.appendix.AppendixModel;

/** Таблицы 4.1, 4.2 и 8.2 в плоских массивах, упорядоченных по DN. */
final class Prices {

    final int[] dn;
    final double[] capacity;
    final double[] perM;
    final double[] reconPerM;
    final double[] maxRun;
    final double[] width;
    final double tap;
    final double penaltyFixed;
    final double penaltyPerTph;
    private final AppendixModel.CostsSpec costs;

    Prices(AppendixModel appendix) {
        List<AppendixModel.DiameterSpec> specs = new ArrayList<>(appendix.getDiameters());
        specs.sort(Comparator.comparingInt(s -> s.dn));
        int n = specs.size();
        dn = new int[n];
        capacity = new double[n];
        perM = new double[n];
        reconPerM = new double[n];
        maxRun = new double[n];
        width = new double[n];
        for (int i = 0; i < n; i++) {
            AppendixModel.DiameterSpec s = specs.get(i);
            dn[i] = s.dn;
            capacity[i] = s.capacityTph;
            perM[i] = appendix.newPerM(s.dn);
            reconPerM[i] = appendix.reconPerM(s.dn);
            maxRun[i] = s.maxRunM > 0 ? s.maxRunM : Double.POSITIVE_INFINITY;
            width[i] = s.widthM;
        }
        costs = appendix.getCosts();
        tap = costs.tapInPipe;
        penaltyFixed = costs.unconnectedFixed;
        penaltyPerTph = costs.unconnectedPerTph;
    }

    /** Индекс минимального DN с пропускной способностью не меньше расхода. */
    int index(double flow) {
        for (int i = 0; i < dn.length; i++) {
            if (capacity[i] + 1e-9 >= flow) {
                return i;
            }
        }
        return dn.length - 1;
    }

    int dnFor(double flow) {
        return dn[index(flow)];
    }

    int indexOf(int d) {
        for (int i = 0; i < dn.length; i++) {
            if (dn[i] >= d) {
                return i;
            }
        }
        return dn.length - 1;
    }

    double perM(int d) {
        return perM[indexOf(d)];
    }

    double reconPerM(int d) {
        return reconPerM[indexOf(d)];
    }

    double maxRun(int d) {
        return maxRun[indexOf(d)];
    }

    double width(int d) {
        return width[indexOf(d)];
    }

    double chamber(int d) {
        return costs.chamber(Math.max(d, dn.length == 0 ? 50 : dn[0]));
    }

    double penalty(double flow) {
        return penaltyFixed + penaltyPerTph * flow;
    }

    /** Следующий DN таблицы или тот же, если больше нет. */
    int bump(int d) {
        int i = indexOf(d);
        return i + 1 < dn.length ? dn[i + 1] : dn[i];
    }
}
