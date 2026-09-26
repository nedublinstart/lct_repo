package ru.lct.heatnet.costing;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.linearref.LengthIndexedLine;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.NewChamber;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.TapPoint;
import ru.lct.heatnet.engine.TechnicalNode;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.geo.GeoJsonGeometries;

public class DiameterSelector {

    private static final Pattern ID_NUM = Pattern.compile("(\\d+)$");

    public int select(double flowTph, AppendixModel appendix) {
        List<AppendixModel.DiameterSpec> specs = new ArrayList<>(appendix.getDiameters());
        specs.sort(Comparator.comparingInt(s -> s.dn));
        for (AppendixModel.DiameterSpec spec : specs) {
            if (spec.capacityTph + 1e-9 >= flowTph) {
                return spec.dn;
            }
        }
        return specs.isEmpty() ? 150 : specs.get(specs.size() - 1).dn;
    }

    /** DN по расходу; DN, уже выбранный трассировщиком (например, по предельной длине), не уменьшается. */
    public void apply(List<NewSegment> segments, AppendixModel appendix) {
        for (NewSegment seg : segments) {
            int dn = Math.max(select(seg.flowTph, appendix), seg.dn);
            AppendixModel.DiameterSpec spec = appendix.diameter(dn);
            while (spec != null && seg.lengthM > spec.maxRunM + 1e-6) {
                int next = bump(dn, appendix);
                if (next == dn) {
                    break;
                }
                dn = next;
                spec = appendix.diameter(dn);
            }
            seg.dn = dn;
        }
    }

    /**
     * DN по расходу и предельной длине целого участка. Участок одного расхода не режется ради нового
     * отсчёта. По направлению к месту присоединения DN не уменьшается.
     */
    public void applyTree(Variant variant, AppendixModel appendix) {
        apply(variant.segments, appendix);
        if (variant.segments.isEmpty()) {
            return;
        }
        raiseWholeSegment(variant, appendix);
        for (int i = 0; i < 8; i++) {
            monotoneTowardTap(variant);
            bumpConsecutive(variant, appendix);
        }
        for (NewChamber ch : variant.chambers) {
            int max = ch.dn;
            for (NewSegment seg : variant.segments) {
                if (ch.id.equals(seg.fromId) || ch.id.equals(seg.toId)) {
                    max = Math.max(max, seg.dn);
                }
            }
            ch.dn = max;
        }
    }

    /** Слишком длинный участок целиком получает следующий DN. Середина участка диаметр не меняет. */
    private void raiseWholeSegment(Variant variant, AppendixModel appendix) {
        for (NewSegment seg : variant.segments) {
            AppendixModel.DiameterSpec spec = appendix.diameter(seg.dn);
            int guard = 0;
            while (spec != null && seg.lengthM > spec.maxRunM + 1e-6 && guard++ < 16) {
                int next = bump(seg.dn, appendix);
                if (next == seg.dn) {
                    break;
                }
                seg.dn = next;
                spec = appendix.diameter(seg.dn);
            }
        }
    }

    /** К месту присоединения DN не уменьшается: верхний участок не тоньше приходящих снизу. */
    private static void monotoneTowardTap(Variant variant) {
        Map<String, Integer> incoming = new HashMap<>();
        for (NewSegment seg : variant.segments) {
            if (seg.toId != null) {
                incoming.merge(seg.toId, seg.dn, Math::max);
            }
        }
        for (NewSegment seg : variant.segments) {
            int need = incoming.getOrDefault(seg.fromId, 0);
            if (need > seg.dn) {
                seg.dn = need;
            }
        }
    }

    private void bumpConsecutive(Variant variant, AppendixModel appendix) {
        Map<String, List<NewSegment>> incoming = new HashMap<>();
        for (NewSegment seg : variant.segments) {
            if (seg.toId != null) {
                incoming.computeIfAbsent(seg.toId, k -> new ArrayList<>()).add(seg);
            }
        }
        Set<String> roots = new HashSet<>();
        for (TapPoint tap : variant.taps) {
            if (tap.nodeId != null) {
                roots.add(tap.nodeId);
            }
            if (tap.existingObjectId != null) {
                roots.add(tap.existingObjectId);
            }
        }
        if (roots.isEmpty()) {
            return;
        }
        Map<NewSegment, Double> run = new HashMap<>();
        Set<String> seen = new HashSet<>();
        for (String root : roots) {
            dfsBump(root, incoming, run, seen, appendix, variant);
        }
    }

    private void dfsBump(String node, Map<String, List<NewSegment>> incoming, Map<NewSegment, Double> run,
                         Set<String> seen, AppendixModel appendix, Variant variant) {
        if (node == null || !seen.add(node)) {
            return;
        }
        for (NewSegment seg : incoming.getOrDefault(node, List.of())) {
            dfsBump(seg.fromId, incoming, run, seen, appendix, variant);
            List<NewSegment> below = incoming.getOrDefault(seg.fromId, List.of());
            double childRun = sameDnRun(below, seg.dn, run);
            AppendixModel.DiameterSpec spec = appendix.diameter(seg.dn);
            int guard = 0;
            while (spec != null && childRun + seg.lengthM > spec.maxRunM + 1e-6 && guard++ < 12) {
                int next = bump(seg.dn, appendix);
                if (next == seg.dn) {
                    break;
                }
                seg.dn = next;
                markDiameterStep(variant, seg.fromId);
                childRun = sameDnRun(below, seg.dn, run);
                spec = appendix.diameter(seg.dn);
            }
            run.put(seg, childRun + seg.lengthM);
        }
    }

    /** Отсчёт предельной длины продолжается через камеру, если DN не меняется. */
    private static double sameDnRun(List<NewSegment> below, int dn, Map<NewSegment, Double> run) {
        double childRun = 0;
        for (NewSegment in : below) {
            if (in.dn == dn) {
                childRun = Math.max(childRun, run.getOrDefault(in, 0.0));
            }
        }
        return childRun;
    }

    private NewSegment splitAt(NewSegment leafSide, double keepM, AtomicInteger ids, Variant variant) {
        if (leafSide.geometryMeters == null || keepM < 1.0 || keepM >= leafSide.lengthM - 1.0) {
            return null;
        }
        LengthIndexedLine lil = new LengthIndexedLine(leafSide.geometryMeters);
        LineString a = (LineString) lil.extractLine(0, keepM);
        LineString b = (LineString) lil.extractLine(keepM, leafSide.lengthM);
        if (a == null || b == null || a.getLength() < 0.5 || b.getLength() < 0.5) {
            return null;
        }
        String mid = "TN-" + ids.getAndIncrement();
        TechnicalNode node = new TechnicalNode();
        node.id = mid;
        node.geometryMeters = GeoJsonGeometries.GF.createPoint(a.getCoordinateN(a.getNumPoints() - 1));
        node.reason = "diameter_step";
        variant.technicalNodes.add(node);

        NewSegment tail = new NewSegment();
        tail.id = "NS-" + ids.getAndIncrement();
        tail.geometryMeters = b;
        tail.lengthM = b.getLength();
        tail.flowTph = leafSide.flowTph;
        tail.dn = leafSide.dn;
        tail.layingMethod = leafSide.layingMethod;
        tail.kSpec = leafSide.kSpec;
        tail.specialReason = leafSide.specialReason;
        tail.fromId = mid;
        tail.toId = leafSide.toId;
        leafSide.geometryMeters = a;
        leafSide.lengthM = a.getLength();
        leafSide.toId = mid;
        return tail;
    }

    private void markDiameterStep(Variant variant, String nodeId) {
        if (nodeId == null) {
            return;
        }
        for (TechnicalNode n : variant.technicalNodes) {
            if (nodeId.equals(n.id)) {
                n.reason = "diameter_step";
                return;
            }
        }
        for (NewChamber ch : variant.chambers) {
            if (nodeId.equals(ch.id)) {
                return;
            }
        }
        if (nodeId.startsWith("TN-") || nodeId.startsWith("CH-")) {
            return;
        }
    }

    private void bumpFrom(NewSegment seg, AppendixModel appendix) {
        int next = bump(seg.dn, appendix);
        if (next > seg.dn) {
            seg.dn = next;
        }
    }

    private int bump(int dn, AppendixModel appendix) {
        int best = dn;
        for (AppendixModel.DiameterSpec spec : appendix.getDiameters()) {
            if (spec.dn > dn && (best == dn || spec.dn < best)) {
                best = spec.dn;
            }
        }
        return best;
    }

    static AtomicInteger nextIds(Variant variant) {
        int max = 0;
        max = Math.max(max, maxNum(variant.segments, s -> s.id));
        max = Math.max(max, maxNum(variant.chambers, c -> c.id));
        max = Math.max(max, maxNum(variant.technicalNodes, n -> n.id));
        max = Math.max(max, maxNum(variant.taps, t -> t.id));
        return new AtomicInteger(max + 1);
    }

    private static <T> int maxNum(List<T> items, java.util.function.Function<T, String> id) {
        int max = 0;
        for (T item : items) {
            String s = id.apply(item);
            if (s == null) {
                continue;
            }
            Matcher m = ID_NUM.matcher(s);
            if (m.find()) {
                max = Math.max(max, Integer.parseInt(m.group(1)));
            }
        }
        return max;
    }
}
