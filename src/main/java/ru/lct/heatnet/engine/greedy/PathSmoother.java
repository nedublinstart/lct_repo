package ru.lct.heatnet.engine.greedy;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;

/**
 * Трасса без сеточной лесенки и без диагональных хорд через квартал:
 * вдоль фасада оставляем ребро, остальное схлопываем в прямые углы.
 */
public final class PathSmoother {

    private PathSmoother() {
    }

    public static List<Coordinate> smooth(List<Coordinate> raw, ObstacleIndex obstacles, double keepDeg) {
        return straighten(raw, obstacles, keepDeg);
    }

    public static List<Coordinate> straighten(List<Coordinate> raw, ObstacleIndex obstacles) {
        return straighten(raw, obstacles, 18);
    }

    public static List<Coordinate> straighten(List<Coordinate> raw, ObstacleIndex obstacles, double keepDeg) {
        if (raw == null || raw.size() < 2) {
            return copy(raw);
        }
        if (raw.size() == 2) {
            return twoPoint(raw, obstacles);
        }
        List<Coordinate> pts = dedupe(raw, 0.55);
        pts = collapseHeading(pts, Math.max(8, keepDeg * 0.5));
        pts = OrthoPaths.collapse(pts, obstacles);
        pts = collapseHeading(pts, Math.max(12, keepDeg));
        pts = dropColinear(pts);
        if (pts.size() < 2) {
            return copy(raw);
        }
        return pts;
    }

    private static List<Coordinate> twoPoint(List<Coordinate> raw, ObstacleIndex obstacles) {
        Coordinate a = raw.get(0);
        Coordinate b = raw.get(1);
        if (!OrthoPaths.longOpenDiagonal(a, b) && (a.distance(b) <= 28 || obstacles.alongAvoid(a, b, OrthoPaths.FACADE_M))) {
            return copy(raw);
        }
        if (OrthoPaths.usefulChord(obstacles, a, b)) {
            return copy(raw);
        }
        List<Coordinate> elbow = OrthoPaths.usefulElbow(obstacles, a, b);
        if (elbow == null && OrthoPaths.longOpenDiagonal(a, b)) {
            elbow = OrthoPaths.bestElbow(obstacles, a, b);
        }
        if (elbow != null && elbow.size() >= 2) {
            return elbow;
        }
        return copy(raw);
    }

    /**
     * Первый сегмент — ввод от ИТП: его нельзя вытягивать сквозь здание.
     */
    public static List<Coordinate> straightenKeepStub(List<Coordinate> raw, ObstacleIndex obstacles) {
        return straightenKeepStub(raw, obstacles, 18);
    }

    public static List<Coordinate> straightenKeepStub(List<Coordinate> raw, ObstacleIndex obstacles, double keepDeg) {
        if (raw == null || raw.size() <= 2) {
            return copy(raw);
        }
        Coordinate stub = new Coordinate(raw.get(0));
        List<Coordinate> rest = straighten(raw.subList(1, raw.size()), obstacles, keepDeg);
        List<Coordinate> out = new ArrayList<>();
        out.add(stub);
        if (rest == null || rest.isEmpty()) {
            out.add(new Coordinate(raw.get(raw.size() - 1)));
            return out;
        }
        int start = rest.get(0).distance(stub) < 0.45 ? 1 : 0;
        for (int i = start; i < rest.size(); i++) {
            out.add(new Coordinate(rest.get(i)));
        }
        if (out.size() < 2) {
            out.add(new Coordinate(raw.get(raw.size() - 1)));
        }
        return out;
    }

    public static List<Coordinate> collapseColinear(List<Coordinate> raw) {
        return collapseColinear(raw, null);
    }

    public static List<Coordinate> collapseColinear(List<Coordinate> raw, ObstacleIndex obstacles) {
        if (raw == null || raw.size() <= 2) {
            return copy(raw);
        }
        List<Coordinate> pts = dedupe(raw, 0.4);
        if (obstacles == null) {
            return collapseHeading(pts, 8);
        }
        List<Coordinate> out = new ArrayList<>();
        out.add(pts.get(0));
        for (int i = 1; i < pts.size() - 1; i++) {
            Coordinate a = out.get(out.size() - 1);
            Coordinate b = pts.get(i);
            Coordinate c = pts.get(i + 1);
            if (b.distance(a) < 0.5) {
                continue;
            }
            boolean keepTurn = OrthoPaths.turnDeg(a, b, c) >= 8 && deviation(a, c, b) >= 0.9;
            if (keepTurn || obstacles.segmentHitsAvoid(a, c, 0, true) || !obstacles.allowsTravel(a, c)) {
                out.add(b);
            }
        }
        out.add(pts.get(pts.size() - 1));
        return out;
    }

    /**
     * Короткие «лишние» изломы. Прямой угол улицы в диагональ не схлопываем —
     * хорда через перекрёсток/проезжую как раз та кривая на карте.
     */
    public static List<Coordinate> collapseStairs(List<Coordinate> raw, ObstacleIndex obstacles) {
        return copy(raw);
    }

    /**
     * Первый сегмент — ввод от ИТП: его нельзя вытягивать сквозь здание.
     * Остальное только схлопывается по коллинеарности, без северных Г-шек.
     */
    public static List<Coordinate> collapseKeepStub(List<Coordinate> raw) {
        return collapseKeepStub(raw, null);
    }

    public static List<Coordinate> collapseKeepStub(List<Coordinate> raw, ObstacleIndex obstacles) {
        if (raw == null || raw.size() <= 2) {
            return copy(raw);
        }
        Coordinate stub = new Coordinate(raw.get(0));
        List<Coordinate> rest = refine(raw.subList(1, raw.size()), obstacles);
        List<Coordinate> out = new ArrayList<>();
        out.add(stub);
        if (rest == null || rest.isEmpty()) {
            out.add(new Coordinate(raw.get(raw.size() - 1)));
            return out;
        }
        int start = rest.get(0).distance(stub) < 0.45 ? 1 : 0;
        for (int i = start; i < rest.size(); i++) {
            out.add(new Coordinate(rest.get(i)));
        }
        if (out.size() < 2) {
            out.add(new Coordinate(raw.get(raw.size() - 1)));
        }
        return out;
    }

    /**
     * Обход корпуса по границе + выкидывание промежуточных точек, если хорда короче.
     * Используется и в {@link StreetFrame#find} — без П-обхода и без дальних skip-ahead.
     */
    public static List<Coordinate> refine(List<Coordinate> raw, ObstacleIndex obstacles) {
        if (raw == null || raw.size() < 2) {
            return copy(raw);
        }
        List<Coordinate> pts = hugHits(dedupe(raw, 0.4), obstacles, 0);
        if (obstacles != null) {
            pts = OrthoPaths.collapse(pts, obstacles);
        }
        pts = collapseColinear(pts, obstacles);
        return dropIfShorter(pts, obstacles);
    }

    /**
     * Только на выдаче трубы, не в Дейкстре каркаса: хорда / Г / П вдоль фасада,
     * если выкинуть промежуточные вершины короче. OARSMT skip-ahead (Kahng–Robins).
     */
    public static List<Coordinate> emitPolish(List<Coordinate> raw, ObstacleIndex obstacles) {
        List<Coordinate> pts = refine(raw, obstacles);
        pts = skipAhead(pts, obstacles);
        pts = hugHits(pts, obstacles, 0.15);
        return dropIfShorter(pts, obstacles);
    }

    /**
     * Делим путь на точки и соединяем i…j, если новый путь короче старого.
     */
    public static List<Coordinate> skipAhead(List<Coordinate> raw, ObstacleIndex obstacles) {
        if (raw == null || raw.size() <= 2 || obstacles == null) {
            return copy(raw);
        }
        List<Coordinate> pts = dedupe(raw, 0.4);
        if (pts.size() <= 2 || OrthoPaths.length(pts) < 28) {
            return pts;
        }
        boolean changed = true;
        int guard = 0;
        while (changed && guard++ < 16) {
            changed = false;
            List<Coordinate> out = new ArrayList<>();
            out.add(new Coordinate(pts.get(0)));
            int i = 0;
            int inner = 0;
            while (i < pts.size() - 1 && inner++ < pts.size() + 4) {
                int bestJ = i + 1;
                List<Coordinate> bestSpan = null;
                double bestSave = 7.5;
                for (int j = pts.size() - 1; j >= i + 2; j--) {
                    double old = spanLength(pts, i, j);
                    if (old < 24) {
                        continue;
                    }
                    List<Coordinate> cand = emitSpan(obstacles, pts.get(i), pts.get(j),
                            i == 0, j == pts.size() - 1);
                    if (cand == null || cand.size() < 2) {
                        continue;
                    }
                    double neu = OrthoPaths.length(cand);
                    double save = old - neu;
                    if (save <= bestSave) {
                        continue;
                    }
                    double oldC = travel(obstacles, pts, i, j);
                    double newC = travel(obstacles, cand, 0, cand.size() - 1);
                    if (!Double.isFinite(newC) || newC + 4 >= oldC) {
                        continue;
                    }
                    bestSave = save;
                    bestJ = j;
                    bestSpan = cand;
                }
                if (bestSpan == null) {
                    out.add(new Coordinate(pts.get(i + 1)));
                    i++;
                    continue;
                }
                changed = true;
                for (int k = 1; k < bestSpan.size(); k++) {
                    Coordinate q = bestSpan.get(k);
                    if (q != null && out.get(out.size() - 1).distance(q) >= 0.35) {
                        out.add(new Coordinate(q));
                    }
                }
                if (bestJ <= i) {
                    break;
                }
                i = bestJ;
            }
            if (out.size() < 2) {
                return pts;
            }
            pts = out;
        }
        return pts;
    }

    /**
     * Если хорда без промежуточной вершины короче и ∥/⊥ улице или вдоль фасада — вершину выкидываем.
     */
    private static List<Coordinate> dropIfShorter(List<Coordinate> pts, ObstacleIndex obstacles) {
        if (pts == null || pts.size() <= 2 || obstacles == null) {
            return pts;
        }
        boolean changed = true;
        int guard = 0;
        while (changed && guard++ < 24) {
            changed = false;
            List<Coordinate> out = new ArrayList<>();
            out.add(pts.get(0));
            for (int i = 1; i < pts.size() - 1; i++) {
                Coordinate a = out.get(out.size() - 1);
                Coordinate b = pts.get(i);
                Coordinate c = pts.get(i + 1);
                if (b == null) {
                    continue;
                }
                double old = a.distance(b) + b.distance(c);
                double neu = a.distance(c);
                if (neu + 1.0 < old && chordOk(obstacles, a, c)) {
                    changed = true;
                    continue;
                }
                out.add(b);
            }
            out.add(pts.get(pts.size() - 1));
            pts = out;
        }
        return pts;
    }

    private static boolean chordOk(ObstacleIndex obstacles, Coordinate a, Coordinate c) {
        if (!OrthoPaths.legal(obstacles, a, c)) {
            return false;
        }
        if (OrthoPaths.usefulChord(obstacles, a, c)) {
            return true;
        }
        if (!OrthoPaths.nearlyAxis(a, c)) {
            return false;
        }
        Coordinate mid = new Coordinate((a.x + c.x) * 0.5, (a.y + c.y) * 0.5);
        return !obstacles.inRoad(mid);
    }

    private static List<Coordinate> hugHits(List<Coordinate> raw, ObstacleIndex obstacles, double width) {
        if (raw == null || raw.size() < 2 || obstacles == null) {
            return copy(raw);
        }
        List<Coordinate> out = new ArrayList<>();
        out.add(new Coordinate(raw.get(0)));
        for (int i = 1; i < raw.size(); i++) {
            Coordinate a = out.get(out.size() - 1);
            Coordinate b = raw.get(i);
            if (b == null) {
                continue;
            }
            if (a.distance(b) > 2.5 && obstacles.segmentHitsAvoid(a, b, Math.max(0, width), true)) {
                List<Coordinate> hug = obstacles.hugAround(a, b);
                if (hug != null && hug.size() >= 2) {
                    for (int k = 1; k < hug.size(); k++) {
                        Coordinate q = hug.get(k);
                        if (q != null && out.get(out.size() - 1).distance(q) >= 0.4) {
                            out.add(new Coordinate(q));
                        }
                    }
                    continue;
                }
            }
            if (out.get(out.size() - 1).distance(b) >= 0.4) {
                out.add(new Coordinate(b));
            }
        }
        if (out.size() < 2) {
            out.add(new Coordinate(raw.get(raw.size() - 1)));
        }
        return out;
    }

    private static List<Coordinate> copy(List<Coordinate> raw) {
        if (raw == null) {
            return null;
        }
        List<Coordinate> out = new ArrayList<>(raw.size());
        for (Coordinate c : raw) {
            if (c != null) {
                out.add(new Coordinate(c));
            }
        }
        return out;
    }

    static boolean visible(ObstacleIndex obstacles, Coordinate a, Coordinate b) {
        return OrthoPaths.legal(obstacles, a, b);
    }

    private static List<Coordinate> emitSpan(ObstacleIndex obstacles, Coordinate a, Coordinate b,
                                             boolean skipFirst, boolean skipLast) {
        if (a == null || b == null) {
            return null;
        }
        List<Coordinate> best = null;
        double bestLen = Double.POSITIVE_INFINITY;
        if (OrthoPaths.legal(obstacles, a, b)
                && (OrthoPaths.nearlyAxis(a, b) || a.distance(b) <= 36)) {
            best = consider(best, bestLen, two(a, b));
            if (best != null) {
                bestLen = OrthoPaths.length(best);
            }
        }
        best = consider(best, bestLen, OrthoPaths.shortcut(obstacles, a, b));
        if (best != null) {
            bestLen = OrthoPaths.length(best);
        }
        best = consider(best, bestLen, OrthoPaths.sidewalkU(obstacles, a, b));
        if (best != null) {
            bestLen = OrthoPaths.length(best);
        }
        best = consider(best, bestLen, obstacles.hugAround(a, b));
        if (best != null) {
            bestLen = OrthoPaths.length(best);
        }
        best = consider(best, bestLen, OrthoPaths.streetElbow(obstacles, a, b));
        if (best != null) {
            bestLen = OrthoPaths.length(best);
        }
        best = consider(best, bestLen, OrthoPaths.bestElbow(obstacles, a, b));
        if (best != null) {
            bestLen = OrthoPaths.length(best);
        }
        Coordinate ae = skipFirst ? obstacles.exitFacing(a, b, 1.2) : a;
        Coordinate be = skipLast ? obstacles.exitFacing(b, a, 1.2) : b;
        if (ae == null) {
            ae = a;
        }
        if (be == null) {
            be = b;
        }
        if (ae.distance(a) > 0.6 || be.distance(b) > 0.6 || skipFirst || skipLast) {
            best = consider(best, bestLen, joinEnds(a, OrthoPaths.sidewalkU(obstacles, ae, be), b));
            if (best != null) {
                bestLen = OrthoPaths.length(best);
            }
            best = consider(best, bestLen, joinEnds(a, OrthoPaths.streetElbow(obstacles, ae, be), b));
            if (best != null) {
                bestLen = OrthoPaths.length(best);
            }
            best = consider(best, bestLen, joinEnds(a, obstacles.hugAround(ae, be), b));
            if (best != null) {
                bestLen = OrthoPaths.length(best);
            }
            best = consider(best, bestLen, joinEnds(a, OrthoPaths.bestElbow(obstacles, ae, be), b));
        }
        if (best == null || hitsMiddle(obstacles, best, skipFirst, skipLast)) {
            return null;
        }
        return best;
    }

    private static List<Coordinate> consider(List<Coordinate> best, double bestLen, List<Coordinate> cand) {
        if (cand == null || cand.size() < 2) {
            return best;
        }
        double len = OrthoPaths.length(cand);
        if (len + 0.4 < bestLen) {
            return copy(cand);
        }
        return best;
    }

    private static List<Coordinate> joinEnds(Coordinate a, List<Coordinate> mid, Coordinate b) {
        if (mid == null || mid.size() < 2 || a == null || b == null) {
            return null;
        }
        List<Coordinate> out = new ArrayList<>();
        out.add(new Coordinate(a));
        for (Coordinate q : mid) {
            if (q != null && out.get(out.size() - 1).distance(q) >= 0.4) {
                out.add(new Coordinate(q));
            }
        }
        if (out.get(out.size() - 1).distance(b) >= 0.4) {
            out.add(new Coordinate(b));
        }
        return out.size() >= 2 ? out : null;
    }

    private static boolean hitsMiddle(ObstacleIndex obstacles, List<Coordinate> path,
                                      boolean skipFirst, boolean skipLast) {
        if (path == null || path.size() < 2) {
            return true;
        }
        for (int i = 0; i < path.size() - 1; i++) {
            if (skipFirst && i == 0) {
                continue;
            }
            if (skipLast && i == path.size() - 2) {
                continue;
            }
            if (obstacles.segmentHitsAvoid(path.get(i), path.get(i + 1), 0, true)) {
                return true;
            }
        }
        return false;
    }

    private static double travel(ObstacleIndex obstacles, List<Coordinate> pts, int from, int to) {
        if (pts == null || from >= to) {
            return 0;
        }
        double s = 0;
        for (int i = from + 1; i <= to; i++) {
            double w = obstacles.travelCost(pts.get(i - 1), pts.get(i));
            if (!Double.isFinite(w)) {
                return Double.POSITIVE_INFINITY;
            }
            s += w;
        }
        return s;
    }

    private static double spanLength(List<Coordinate> pts, int from, int to) {
        double s = 0;
        for (int i = from + 1; i <= to; i++) {
            s += pts.get(i - 1).distance(pts.get(i));
        }
        return s;
    }

    private static List<Coordinate> two(Coordinate a, Coordinate b) {
        List<Coordinate> out = new ArrayList<>(2);
        out.add(new Coordinate(a));
        out.add(new Coordinate(b));
        return out;
    }

    private static List<Coordinate> dedupe(List<Coordinate> raw, double minM) {
        List<Coordinate> out = new ArrayList<>();
        Coordinate prev = null;
        for (Coordinate c : raw) {
            if (c == null) {
                continue;
            }
            if (prev != null && prev.distance(c) < minM) {
                continue;
            }
            out.add(new Coordinate(c));
            prev = c;
        }
        if (out.size() == 1 && raw.size() >= 2) {
            out.add(new Coordinate(raw.get(raw.size() - 1)));
        }
        return out;
    }

    private static List<Coordinate> collapseHeading(List<Coordinate> pts, double keepDeg) {
        if (pts.size() <= 2) {
            return pts;
        }
        List<Coordinate> out = new ArrayList<>();
        out.add(pts.get(0));
        for (int i = 1; i < pts.size() - 1; i++) {
            Coordinate a = out.get(out.size() - 1);
            Coordinate b = pts.get(i);
            Coordinate c = pts.get(i + 1);
            if (b.distance(a) < 0.5) {
                continue;
            }
            if (OrthoPaths.turnDeg(a, b, c) >= keepDeg && deviation(a, c, b) >= 0.9) {
                out.add(b);
            }
        }
        out.add(pts.get(pts.size() - 1));
        return out;
    }

    private static List<Coordinate> dropColinear(List<Coordinate> pts) {
        if (pts.size() <= 2) {
            return pts;
        }
        List<Coordinate> out = new ArrayList<>();
        out.add(pts.get(0));
        for (int i = 1; i < pts.size() - 1; i++) {
            Coordinate a = out.get(out.size() - 1);
            Coordinate b = pts.get(i);
            Coordinate c = pts.get(i + 1);
            if (OrthoPaths.turnDeg(a, b, c) < 8 && deviation(a, c, b) < 0.8) {
                continue;
            }
            out.add(b);
        }
        out.add(pts.get(pts.size() - 1));
        return out;
    }

    private static double deviation(Coordinate a, Coordinate c, Coordinate b) {
        double vx = c.x - a.x;
        double vy = c.y - a.y;
        double len = Math.hypot(vx, vy);
        if (len < 1e-6) {
            return a.distance(b);
        }
        double t = ((b.x - a.x) * vx + (b.y - a.y) * vy) / (len * len);
        t = Math.max(0, Math.min(1, t));
        double px = a.x + t * vx;
        double py = a.y + t * vy;
        return Math.hypot(b.x - px, b.y - py);
    }
}
