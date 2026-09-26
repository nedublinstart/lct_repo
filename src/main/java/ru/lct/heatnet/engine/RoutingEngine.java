package ru.lct.heatnet.engine;

import java.util.List;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.Scene;

public interface RoutingEngine {
    List<Variant> route(Scene scene, AppendixModel appendix, CalculationMode mode, ProgressListener progress);

    /**
     * Те же трассы, но только отмеченные режимы ({@code mincost}, {@code mintaps}, {@code minrecon}).
     * Пустой список означает все режимы.
     */
    default List<Variant> route(Scene scene, AppendixModel appendix, CalculationMode mode, ProgressListener progress,
                                List<String> strategyCodes) {
        return route(scene, appendix, mode, progress);
    }
}
