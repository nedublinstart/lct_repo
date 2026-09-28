package ru.lct.heatnet.costing;

import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.flow.NetworkReconstruction;
import ru.lct.heatnet.scene.Scene;

/**
 * Старый калькулятор реконструкции. В расчёт сервиса не входит: приложение от 26.09.2026
 * реконструкцию не считает.
 */
public class ReconstructionCalculator {

    public void apply(Variant variant, Scene scene, AppendixModel appendix) {
        NetworkReconstruction.apply(variant, scene, appendix);
    }
}
