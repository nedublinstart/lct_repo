package ru.lct.heatnet.engine;

import java.util.List;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.Scene;

public interface RoutingEngine {
    List<Variant> route(Scene scene, AppendixModel appendix, CalculationMode mode, ProgressListener progress);
}
