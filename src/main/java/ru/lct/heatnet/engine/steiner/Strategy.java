package ru.lct.heatnet.engine.steiner;

/**
 * Три содержательно разных варианта по ТЗ: стоимость / число врезок / реконструкция.
 */
public enum Strategy {
    MIN_COST(
            "mincost",
            "Минимальная стоимость",
            "Кластеризация ОКС, если совместное подключение дешевле раздельного; дерево Штейнера (Mehlhorn) и врезка с лучшим показателем 70/30.",
            1800.0,
            1.0,
            40.0,
            1.6,
            12,
            false
    ),
    MIN_TAPS(
            "mintaps",
            "Минимум врезок",
            "Штраф за каждую врезку: ОКС объединяются в общие деревья, пока есть путь. Меньше точек врезки, больше общих участков.",
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
            "Врезки в камеры и участки с запасом пропускной способности: меньше замены существующей сети.",
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
}
