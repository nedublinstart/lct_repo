package ru.lct.heatnet.costing;

import java.util.ArrayList;
import java.util.List;
import ru.lct.heatnet.engine.Variant;

/**
 * Подпись «Минимальная стоимость» остаётся только у трассы с наименьшей полной стоимостью.
 * Вариант, который хуже другого и по деньгам, и по врезкам, и по реконструкции, на карту не попадает.
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
                variant.description = "Полная стоимость ниже остальных построенных трасс: труба, врезки, камеры и реконструкция.";
            } else if ("Минимальная стоимость".equals(variant.title)) {
                variant.title = fallbackTitle(variant, cheapest);
                variant.description = "Эта трасса дороже варианта с минимальной стоимостью.";
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
                && recon(other) <= recon(variant) + 1.0
                && unconnected(other) <= unconnected(variant);
    }

    private static boolean strictlyBetter(Variant other, Variant variant) {
        return other.totalCost + 1000.0 < variant.totalCost
                || taps(other) < taps(variant)
                || recon(other) + 1000.0 < recon(variant)
                || unconnected(other) < unconnected(variant);
    }

    private static Variant cheapest(List<Variant> variants) {
        Variant best = variants.get(0);
        for (Variant variant : variants) {
            if (variant.totalCost + 1.0 < best.totalCost
                    || (Math.abs(variant.totalCost - best.totalCost) <= 1.0 && taps(variant) < taps(best))) {
                best = variant;
            }
        }
        return best;
    }

    private static String fallbackTitle(Variant variant, Variant cheapest) {
        if (taps(variant) < taps(cheapest)) {
            return "Минимум врезок";
        }
        if (recon(variant) + 1000.0 < recon(cheapest)) {
            return "Минимум реконструкции";
        }
        return "Дополнительный контур";
    }

    private static int taps(Variant variant) {
        return variant.taps == null ? 0 : variant.taps.size();
    }

    private static int unconnected(Variant variant) {
        return variant.unconnectedOks == null ? 0 : variant.unconnectedOks.size();
    }

    private static double recon(Variant variant) {
        if (variant.costBreakdown == null) {
            return 0;
        }
        return num(variant, "reconstruction_cost") + num(variant, "chamber_reconstruction_cost");
    }

    private static double num(Variant variant, String key) {
        Double value = variant.costBreakdown.get(key);
        return value == null ? 0 : value;
    }
}
