package ru.lct.heatnet.engine.steiner;

import java.util.ArrayList;
import java.util.List;

/**
 * Три содержательно разных варианта по ТЗ: стоимость / число врезок / реконструкция.
 */
public enum Strategy {
    MIN_COST(
            "mincost",
            "Минимальная стоимость",
            "Лес наименьшей полной стоимости по приложению: трубы по DN расхода, камеры, врезки, реконструкция и штрафы.",
            700.0,
            1.0,
            40.0,
            1.6,
            18,
            false
    ),
    MIN_TAPS(
            "mintaps",
            "Минимум врезок",
            "Каждая врезка сверх первой весит в цели ещё 50 млн ₽: ОКС собираются в общие деревья, врезок меньше, трубы длиннее.",
            2800.0,
            -1.0e6,
            420.0,
            1.0,
            10,
            true
    ),
    MIN_RECON(
            "minrecon",
            "Минимум реконструкции",
            "Реконструкция существующей сети весит в цели в 21 раз больше: врезки ближе к источнику и на участках с запасом.",
            700.0,
            1.0,
            30.0,
            22.0,
            18,
            false
    );

    public final String code;
    public final String title;
    public final String description;
    public final double mergeRadiusM;
    public final double minSavings;
    public final double tapFee;
    public final double reconWeight;
    public final int tapShortlist;
    public final boolean forceMerge;

    Strategy(String code, String title, String description, double mergeRadiusM, double minSavings,
             double tapFee, double reconWeight, int tapShortlist, boolean forceMerge) {
        this.code = code;
        this.title = title;
        this.description = description;
        this.mergeRadiusM = mergeRadiusM;
        this.minSavings = minSavings;
        this.tapFee = tapFee;
        this.reconWeight = reconWeight;
        this.tapShortlist = tapShortlist;
        this.forceMerge = forceMerge;
    }

    /** Пустой список — все режимы. Незнакомые коды пропускаются. */
    public static List<Strategy> select(List<String> codes) {
        if (codes == null || codes.isEmpty()) {
            return List.of(values());
        }
        List<Strategy> out = new ArrayList<>();
        for (String code : codes) {
            if (code == null || code.isBlank()) {
                continue;
            }
            String key = code.trim();
            for (Strategy strategy : values()) {
                if (strategy.code.equalsIgnoreCase(key) && !out.contains(strategy)) {
                    out.add(strategy);
                }
            }
        }
        return out.isEmpty() ? List.of(MIN_COST) : out;
    }

    public static String join(List<String> codes) {
        if (codes == null || codes.isEmpty()) {
            return "";
        }
        StringBuilder sb = new StringBuilder();
        for (String code : codes) {
            if (code == null || code.isBlank()) {
                continue;
            }
            if (sb.length() > 0) {
                sb.append(',');
            }
            sb.append(code.trim());
        }
        return sb.toString();
    }
}
