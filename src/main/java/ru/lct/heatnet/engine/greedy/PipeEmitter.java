package ru.lct.heatnet.engine.greedy;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.geo.GeoJsonGeometries;

/**
 * Режет трассу на обычную прокладку и короткие спецпроходы через ТДТП.
 */
public final class PipeEmitter {

    private PipeEmitter() {
    }

    public static NewSegment emit(Variant variant, ObstacleIndex obstacles, AtomicInteger ids,
                                  String fromId, String toId, double flow, List<Coordinate> path) {
        List<SpecialLayer.Piece> pieces = obstacles.splitByTransport(path);
        if (pieces.isEmpty()) {
            pieces = List.of(SpecialLayer.Piece.base(path));
        }
        for (SpecialLayer.Piece piece : pieces) {
            List<Coordinate> slim = PathSmoother.collapseColinear(piece.coords, obstacles);
            if (slim != null && slim.size() >= 2 && slim != piece.coords) {
                piece.coords.clear();
                piece.coords.addAll(slim);
            }
        }
        NewSegment first = null;
        String prev = fromId;
        for (int i = 0; i < pieces.size(); i++) {
            SpecialLayer.Piece piece = pieces.get(i);
            String next = i == pieces.size() - 1 ? toId : technical(variant, ids, piece.end(), nextReason(pieces, i));
            NewSegment seg = segment(ids, prev, next, flow, piece);
            variant.segments.add(seg);
            if (first == null) {
                first = seg;
            }
            prev = next;
        }
        return first;
    }

    private static String nextReason(List<SpecialLayer.Piece> pieces, int i) {
        if (i + 1 < pieces.size() && pieces.get(i + 1).special) {
            return "enter_special";
        }
        return "leave_special";
    }

    private static String technical(Variant variant, AtomicInteger ids, Coordinate at, String reason) {
        TechnicalNode node = new TechnicalNode();
        node.id = "TN-" + ids.getAndIncrement();
        node.geometryMeters = GeoJsonGeometries.GF.createPoint(at);
        node.reason = reason;
        variant.technicalNodes.add(node);
        return node.id;
    }

    private static NewSegment segment(AtomicInteger ids, String from, String to, double flow, SpecialLayer.Piece piece) {
        LineString ls = toLine(piece.coords);
        NewSegment seg = new NewSegment();
        seg.id = "NS-" + ids.getAndIncrement();
        seg.geometryMeters = ls;
        seg.lengthM = ls.getLength();
        seg.flowTph = flow;
        seg.fromId = from;
        seg.toId = to;
        if (piece.special) {
            seg.layingMethod = piece.method == null ? "special" : piece.method;
            seg.kSpec = piece.kSpec > 0 ? piece.kSpec : 1.6;
            seg.specialReason = piece.reason == null || piece.reason.isBlank() ? "road" : piece.reason;
        } else {
            seg.layingMethod = "base";
            seg.kSpec = 1.0;
        }
        return seg;
    }

    static LineString toLine(List<Coordinate> path) {
        List<Coordinate> pts = new ArrayList<>();
        Coordinate prev = null;
        for (Coordinate c : path) {
            if (prev != null && prev.distance(c) < 1e-4) {
                continue;
            }
            pts.add(new Coordinate(c));
            prev = c;
        }
        if (pts.size() == 1) {
            Coordinate extra = new Coordinate(pts.get(0));
            extra.x += 0.3;
            pts.add(extra);
        }
        return GeoJsonGeometries.GF.createLineString(pts.toArray(new Coordinate[0]));
    }
}
