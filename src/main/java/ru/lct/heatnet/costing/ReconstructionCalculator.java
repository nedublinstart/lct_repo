package ru.lct.heatnet.costing;

import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.flow.NetworkReconstruction;
import ru.lct.heatnet.scene.Scene;

/**
 * Реконструкция существующей сети по разделам 7 и 8.2: добавленный расход идёт от точки врезки к
 * источнику, на участке врезки — только на части от точки врезки к источнику; камера реконструируется,
 * только если в неё врезались.
 */
public class ReconstructionCalculator {

    public void apply(Variant variant, Scene scene, AppendixModel appendix) {
        NetworkReconstruction.apply(variant, scene, appendix);
    }
}
