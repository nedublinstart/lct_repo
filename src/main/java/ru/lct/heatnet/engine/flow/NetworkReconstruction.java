package ru.lct.heatnet.engine.flow;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.ReconstructionChamber;
import ru.lct.heatnet.engine.ReconstructionSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.Scene;

/**
 * Реконструкция существующей сети по разделам 7 и 8.2 для готового варианта: те же правила, что в
 * оценке поиска ({@link ExistingNet#recon}). Каждая реконструируемая часть — отдельная линия с
 * геометрией этой части; камера реконструируется только если в неё врезались.
 */
public final class NetworkReconstruction {

    private NetworkReconstruction() {
    }

    public static void apply(Variant variant, Scene scene, AppendixModel appendix) {
        variant.reconstructionSegments.clear();
        variant.reconstructionChambers.clear();
        if (variant.taps.isEmpty()) {
            return;
        }
        Prices prices = new Prices(appendix);
        ExistingNet net = new ExistingNet(scene);
        List<ExistingNet.Tap> taps = new ArrayList<>();
        for (TapPoint p : variant.taps) {
            ExistingNet.Tap t = tapOf(net, p);
            if (t != null) {
                taps.add(t);
            }
        }
        ExistingNet.Recon r = net.recon(taps, prices, true);
        Map<Integer, Integer> partsPerSeg = new HashMap<>();
        for (double[] part : r.parts) {
            partsPerSeg.merge((int) part[0], 1, Integer::sum);
        }
        Map<Integer, Integer> numbered = new HashMap<>();
        for (double[] part : r.parts) {
            ExistingNet.Seg s = net.segs.get((int) part[0]);
            int required = (int) part[4];
            ReconstructionSegment rs = new ReconstructionSegment();
            int k = numbered.merge(s.index, 1, Integer::sum);
            rs.id = partsPerSeg.get(s.index) > 1 ? "RE-" + s.id + "-" + k : "RE-" + s.id;
            rs.existingObjectId = s.id;
            double[] line = Geo.slice(s.line, part[1], part[2]);
            Coordinate[] cs = new Coordinate[line.length / 2];
            for (int i = 0; i < cs.length; i++) {
                cs[i] = new Coordinate(line[i * 2], line[i * 2 + 1]);
            }
            rs.geometryMeters = GeoJsonGeometries.GF.createLineString(cs);
            rs.existingDn = s.dn;
            rs.requiredDn = required;
            rs.existingFlowTph = s.flow;
            rs.extraFlowTph = part[3];
            rs.calculatedFlowTph = s.flow + part[3];
            rs.lengthM = part[2] - part[1];
            rs.cost = rs.lengthM * prices.reconPerM(required);
            variant.reconstructionSegments.add(rs);
        }
        for (int[] cp : r.chamberParts) {
            ExistingNet.Cham h = net.chambers.get(cp[0]);
            ReconstructionChamber rc = new ReconstructionChamber();
            rc.id = "RC-" + h.id;
            rc.existingObjectId = h.id;
            rc.geometryMeters = GeoJsonGeometries.GF.createPoint(new Coordinate(h.x, h.y));
            rc.existingDn = h.dn;
            rc.requiredDn = cp[1];
            rc.cost = prices.chamber(cp[1]);
            variant.reconstructionChambers.add(rc);
        }
    }

    /** Врезка варианта → позиция на участке от конца к источнику или камера. */
    private static ExistingNet.Tap tapOf(ExistingNet net, TapPoint p) {
        if (p.existingObjectId == null || p.geometryMeters == null) {
            return null;
        }
        Integer c = net.chamberIndex.get(p.existingObjectId);
        if (c != null && !"heat_network".equals(p.existingObjectKind)) {
            return new ExistingNet.Tap(-1, 0, c, p.extraFlowTph, p.requiredDiameter);
        }
        Integer s = net.segIndex.get(p.existingObjectId);
        if (s == null) {
            return null;
        }
        double[] line = net.segs.get(s).line;
        double px = p.geometryMeters.getX();
        double py = p.geometryMeters.getY();
        double best = Double.POSITIVE_INFINITY;
        double at = 0;
        double acc = 0;
        for (int i = 0; i + 3 < line.length; i += 2) {
            double l = Geo.dist(line[i], line[i + 1], line[i + 2], line[i + 3]);
            double t = Geo.footParam(line[i], line[i + 1], line[i + 2], line[i + 3], px, py);
            double d = Geo.dist(line[i] + t * (line[i + 2] - line[i]), line[i + 1] + t * (line[i + 3] - line[i + 1]), px, py);
            if (d < best) {
                best = d;
                at = acc + t * l;
            }
            acc += l;
        }
        return new ExistingNet.Tap(s, at, -1, p.extraFlowTph, p.requiredDiameter);
    }
}
