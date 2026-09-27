package ru.lct.heatnet.costing;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;

class VariantClaimsTest {

    @Test
    void expensiveMinCostLosesTheNameAndTheSlot() {
        Variant recon = variant("minrecon", "Минимум реконструкции", 184_000_000, 2, 0);
        Variant taps = variant("mintaps", "Минимум врезок", 216_000_000, 1, 0);
        Variant cost = variant("mincost", "Минимальная стоимость", 235_000_000, 2, 0);

        List<Variant> all = new ArrayList<>();
        all.add(cost);
        all.add(taps);
        all.add(recon);
        VariantClaims.apply(all);

        assertThat(all).extracting(v -> v.code).containsExactlyInAnyOrder("minrecon", "mintaps");
        Variant cheapest = null;
        for (Variant variant : all) {
            if (cheapest == null || variant.totalCost < cheapest.totalCost) {
                cheapest = variant;
            }
        }
        assertThat(cheapest.code).isEqualTo("minrecon");
        assertThat(cheapest.title).isEqualTo("Минимальная стоимость");
        assertThat(all).noneMatch(v -> v.totalCost > 230_000_000);
        Variant tapsLeft = null;
        for (Variant variant : all) {
            if ("mintaps".equals(variant.code)) {
                tapsLeft = variant;
            }
        }
        assertThat(tapsLeft.title).isEqualTo("Минимум врезок");
    }

    @Test
    void shorterRouteStaysEvenIfItCostsMore() {
        Variant cost = variant("mincost", "Минимальная стоимость", 200_000_000, 2, 0);
        cost.newLengthM = 1800;
        Variant recon = variant("minrecon", "Минимальная длина", 230_000_000, 2, 0);
        recon.newLengthM = 1700;
        List<Variant> all = new ArrayList<>();
        all.add(cost);
        all.add(recon);
        VariantClaims.apply(all);

        assertThat(all).extracting(v -> v.code).containsExactlyInAnyOrder("mincost", "minrecon");
        Variant length = null;
        for (Variant variant : all) {
            if ("minrecon".equals(variant.code)) {
                length = variant;
            }
        }
        assertThat(length.title).isEqualTo("Минимальная длина");
        assertThat(cost.title).isEqualTo("Минимальная стоимость");
    }

    @Test
    void singleTapModeKeepsItsOwnTitle() {
        Variant taps = variant("mintaps", "Минимум врезок", 216_000_000, 1, 0);
        List<Variant> all = new ArrayList<>();
        all.add(taps);
        VariantClaims.apply(all);
        assertThat(all).hasSize(1);
        assertThat(all.get(0).title).isEqualTo("Минимум врезок");
    }

    private static Variant variant(String code, String title, double cost, int tapCount, double recon) {
        Variant variant = new Variant();
        variant.code = code;
        variant.title = title;
        variant.totalCost = cost;
        variant.costBreakdown.put("reconstruction_cost", recon);
        for (int i = 0; i < tapCount; i++) {
            TapPoint tap = new TapPoint();
            tap.id = code + "-" + i;
            variant.taps.add(tap);
        }
        return variant;
    }
}
