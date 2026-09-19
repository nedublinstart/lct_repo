package ru.lct.heatnet.engine.depth;

import org.locationtech.jts.geom.Coordinate;
import org.springframework.stereotype.Component;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

@Component
public class DepthPostProcessor {

    public void apply(Variant variant, Scene scene, AppendixModel appendix) {
        AppendixModel.DepthSpec depth = appendix.getDepth();
        for (NewSegment seg : variant.segments) {
            AppendixModel.DiameterSpec spec = appendix.diameter(seg.dn);
            double minDepth = spec == null ? depth.minDepthM : spec.minDepthM;
            double z = -minDepth;
            for (SpatialConstraint c : scene.constraints) {
                if (c.geometry == null || seg.geometryMeters == null) {
                    continue;
                }
                if (!c.geometry.intersects(seg.geometryMeters)) {
                    continue;
                }
                Double util = depth.utilityDepthM.get(c.type);
                if (util == null) {
                    continue;
                }
                double height = spec == null ? 1.0 : spec.heightM;
                double below = util - depth.clearanceM - height / 2.0;
                z = Math.min(z, below);
            }
            z = snap(z, depth.stepM);
            double depthM = Math.min(depth.maxDepthM, Math.max(depth.minDepthM, -z));
            seg.depthM = depthM;
            Coordinate[] coords = seg.geometryMeters.getCoordinates();
            for (Coordinate coord : coords) {
                coord.z = -depthM;
            }
            seg.geometryMeters.geometryChanged();
        }
        variant.notes.add("Глубина назначена упрощённо: min(нормативная, обход коммуникаций снизу).");
    }

    private static double snap(double z, double step) {
        if (step <= 0) {
            return z;
        }
        return Math.round(z / step) * step;
    }
}
