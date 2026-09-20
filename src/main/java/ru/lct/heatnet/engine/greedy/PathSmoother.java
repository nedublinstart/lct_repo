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
     */
    public static List<Coordinate> refine(List<Coordinate> raw, ObstacleIndex obstacles) {
        if (raw == null || raw.size() < 2) {
            return copy(raw);
        }
        List<Coordinate> pts = hugHits(dedupe(raw, 0.4), obstacles);
        if (obstacles != null) {
            pts = OrthoPaths.collapse(pts, obstacles);
        }
        pts = collapseColinear(pts, obstacles);
        return dropIfShorter(pts, obstacles);
    }

    /**
     * Если хорда без промежуточной вершины короче и ∥/⊥ улице или вдоль фасада — вершину выкидываем.
     */
    private static List<Coordinate> dropIfShorter(List<Coordinate> pts, ObstacleIndex obstacles) {
        if (pts == null || pts.size() <= 2 || obstacles == null) {
            return pts;
        }
        pts = new ArrayList<>(pts);
        boolean changed = true;
        int guard = 0;
        while (changed && guard++ < 16) {
            changed = false;
            outer:
            for (int span = pts.size() - 1; span >= 2; span--) {
                for (int i = 0; i + span < pts.size(); i++) {
                    int j = i + span;
                    double sub = 0;
                    for (int k = i; k < j; k++) {
                        sub += pts.get(k).distance(pts.get(k + 1));
                    }
                    double neu = pts.get(i).distance(pts.get(j));
                    if (neu + 1.2 < sub && chordOk(obstacles, pts.get(i), pts.get(j))) {
                        List<Coordinate> nxt = new ArrayList<>(i + 1 + pts.size() - j);
                        for (int k = 0; k <= i; k++) {
                            nxt.add(pts.get(k));
                        }
                        for (int k = j; k < pts.size(); k++) {
                            nxt.add(pts.get(k));
                        }
                        pts = nxt;
                        changed = true;
                        break outer;
                    }
                }
            }
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

    private static List<Coordinate> hugHits(List<Coordinate> raw, ObstacleIndex obstacles) {
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
            if (a.distance(b) > 2.5 && obstacles.segmentHitsAvoid(a, b, 0, true)) {
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
