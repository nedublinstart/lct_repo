package ru.lct.heatnet.costing;

import java.util.ArrayList;
import java.util.List;
import ru.lct.heatnet.engine.Variant;

/**
 * Подпись «Минимальная стоимость» остаётся у трассы с наименьшим показателем S.
 * Почти тот же коридор (длина в пределах 40 м, не меньше врезок и не дешевле) на карту не попадает.
 */
public final class VariantClaims {

    private VariantClaims() {
    }

    public static void apply(List<Variant> variants) {
        if (variants == null || variants.isEmpty()) {
            return;
        }
        boolean ranMinCost = false;
        for (Variant variant : variants) {
            if ("mincost".equals(variant.code)) {
                ranMinCost = true;
            }
        }
        List<Variant> kept = new ArrayList<>();
        for (Variant variant : variants) {
            if (!dominated(variant, variants)) {
                kept.add(variant);
            }
        }
        variants.clear();
        variants.addAll(kept);
        if (!ranMinCost || variants.isEmpty()) {
            return;
        }
        Variant cheapest = cheapest(variants);
        for (Variant variant : variants) {
            if (variant == cheapest) {
                variant.title = "Минимальная стоимость";
                variant.description = "Наименьший S среди вариантов.";
            } else if ("Минимальная стоимость".equals(variant.title)) {
                variant.title = fallbackTitle(variant, cheapest);
                variant.description = "S выше, чем у варианта с наименьшей стоимостью.";
            }
        }
    }

    private static boolean dominated(Variant variant, List<Variant> all) {
        if (!managed(variant)) {
            return false;
        }
        for (Variant other : all) {
            if (other == variant || !managed(other)) {
                continue;
            }
            if (notWorse(other, variant) && strictlyBetter(other, variant)) {
                return true;
            }
        }
        return false;
    }

    private static boolean managed(Variant variant) {
        String code = variant.code == null ? "" : variant.code;
        return "mincost".equals(code) || "mintaps".equals(code) || "minrecon".equals(code) || "independent".equals(code);
    }

    private static boolean notWorse(Variant other, Variant variant) {
        return other.totalCost <= variant.totalCost + 1.0
                && taps(other) <= taps(variant)
                && Math.abs(other.newLengthM - variant.newLengthM) <= 40.0
                && unconnected(other) <= unconnected(variant);
    }

    private static boolean strictlyBetter(Variant other, Variant variant) {
        return other.totalCost + 1000.0 < variant.totalCost
                || taps(other) < taps(variant)
                || unconnected(other) < unconnected(variant);
    }

    private static Variant cheapest(List<Variant> variants) {
        Variant best = variants.get(0);
        for (Variant variant : variants) {
            double score = scoreOf(variant);
            double bestScore = scoreOf(best);
            if (score + 1e-9 < bestScore
                    || (Math.abs(score - bestScore) <= 1e-9 && taps(variant) < taps(best))) {
                best = variant;
            }
        }
        return best;
    }

    private static double scoreOf(Variant variant) {
        return 0.7 * variant.totalCost / 25_000_000.0 + 0.3 * variant.newLengthM / 100.0;
    }

    private static String fallbackTitle(Variant variant, Variant cheapest) {
        if (taps(variant) < taps(cheapest)) {
            return "Минимум врезок";
        }
        if (variant.newLengthM + 40.0 < cheapest.newLengthM) {
            return "Короче трасса";
        }
        return "Дополнительный контур";
    }

    /** Число мест присоединения: новые камеры врезки и разные существующие камеры. */
    private static int taps(Variant variant) {
        int sites = 0;
        if (variant.chambers != null) {
            for (ru.lct.heatnet.engine.NewChamber chamber : variant.chambers) {
                if (chamber.atTap) {
                    sites++;
                }
            }
        }
        java.util.Set<String> existing = new java.util.HashSet<>();
        if (variant.taps != null) {
            for (ru.lct.heatnet.engine.TapPoint tap : variant.taps) {
                if (tap.existingObjectId != null) {
                    existing.add(tap.existingObjectId);
                }
            }
        }
        sites += existing.size();
        if (sites == 0 && variant.taps != null) {
            return variant.taps.size();
        }
        return sites;
    }

    private static int unconnected(Variant variant) {
        return variant.unconnectedOks == null ? 0 : variant.unconnectedOks.size();
    }

}
