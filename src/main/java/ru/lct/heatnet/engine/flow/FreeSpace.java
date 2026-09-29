package ru.lct.heatnet.engine.flow;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Location;
import org.locationtech.jts.geom.MultiLineString;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.geom.Polygonal;
import org.locationtech.jts.geom.Puntal;
import org.locationtech.jts.index.strtree.STRtree;
import org.locationtech.jts.operation.buffer.BufferOp;
import org.locationtech.jts.operation.buffer.BufferParameters;
import org.locationtech.jts.operation.union.UnaryUnionOp;
import org.locationtech.jts.simplify.TopologyPreservingSimplifier;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

/**
 * Свободное пространство для оси новой трубы.
 * <p>
 * Запрещённые объекты раздуваются на минимальное расстояние таблицы 2 плюс половину ширины
 * расчётного габарита пары труб (расстояние меряется между внешними границами габаритов). Объекты
 * со специальным проходом (дороги, трамвай, газ, кабель, существующая теплосеть) не блокируют ось,
 * но отрезок обязан их именно пересекать: вход в полосу минимального расстояния без пересечения
 * самого объекта — это прокладка «вдоль», она запрещена. За каждое пересечение к длине добавляется
 * (Kспец − 1) · длина специального участка — стоимость пропорциональна cнов(ДУ), поэтому надбавку
 * удобно хранить в метрах.
 * <p>
 * Контуры запретов упрощаются на {@link #SIMPLIFY_M} и раздуваются на столько же больше, поэтому
 * проверка по упрощённой границе строже точной. Там, где ось обязана подойти вплотную к заданной
 * точке (камера или участок врезки, фасад ИТП), используется точная проверка {@link #violation}.
 */
final class FreeSpace {

    static final double EPS = 0.01;
    private static final double SIMPLIFY_M = 0.3;
    private static final double MITRE_LIMIT = 1.1;
    private static final double TAP_EXIT_MIN_SIN = 0.5;
    /** Короче этого кусок у конца ломаной не выделяется в отдельный участок. */
    private static final double MIN_PIECE_M = 0.05;
    /** С этого числа рёбер объект получает собственную сетку рёбер. */
    private static final int GRID_MIN_EDGES = 32;
    /** Полугабарит линейных объектов п. 4.3: газопровод 0,4 м, кабель 0,2 м. */
    private static final double GAS_HALF = 0.2;
    private static final double CABLE_HALF = 0.1;

    final Envelope roi;
    final int designDn;
    final double margin;
    /** Граница запретов (с зазором), по её выпуклым углам строится граф видимости. */
    final Geometry hard;
    private final EdgeGrid hardEdges;
    final List<Zone> zones = new ArrayList<>();
    private final STRtree zoneBoxes;
    final List<Avoid> avoids = new ArrayList<>();
    private final STRtree avoidBoxes;
    private final double maxClearance;
    private final HitVisitor hit = new HitVisitor();

    /** Запрещённый объект в исходной геометрии и требуемое расстояние до оси. */
    static final class Avoid {
        int index;
        final String id;
        final Geometry raw;
        final double clearance;
        private final boolean polygonal;
        /** Рёбра исходной геометрии подряд (x1, y1, x2, y2); точка — ребро нулевой длины. */
        final double[] edges;
        final Envelope env;
        /** Сетка рёбер для крупных объектов (железная дорога, водоём): запросы по соседним клеткам. */
        private final EdgeGrid grid;
        private final Nearest nearest;

        Avoid(String id, Geometry raw, double clearance) {
            this.id = id;
            this.raw = raw;
            this.clearance = clearance;
            this.polygonal = raw instanceof Polygonal;
            this.edges = edgesOf(raw);
            this.env = raw.getEnvelopeInternal();
            int count = edges.length / 4;
            if (count >= GRID_MIN_EDGES) {
                grid = new EdgeGrid(edges, null, count, 4.0, clearance);
                if (polygonal) {
                    grid.indexInside();
                }
                nearest = new Nearest();
            } else {
                grid = null;
                nearest = null;
            }
        }

        /** Точка внутри многоугольника: чётность пересечений луча с контурами. */
        boolean contains(double x, double y) {
            if (!polygonal || !env.contains(x, y)) {
                return false;
            }
            if (grid != null) {
                return grid.covers(x, y);
            }
            boolean in = false;
            for (int i = 0; i < edges.length; i += 4) {
                double y1 = edges[i + 1];
                double y2 = edges[i + 3];
                if ((y1 > y) != (y2 > y)
                        && x < edges[i] + (y - y1) * (edges[i + 2] - edges[i]) / (y2 - y1)) {
                    in = !in;
                }
            }
            return in;
        }

        /**
         * Расстояние от отрезка до объекта, если оно меньше limit, иначе +∞; 0 — отрезок касается
         * объекта или заходит внутрь.
         */
        double distance(double ax, double ay, double bx, double by, double limit) {
            if (contains(ax, ay) || contains(bx, by)) {
                return 0;
            }
            double best;
            if (grid == null) {
                best = Double.POSITIVE_INFINITY;
                for (int i = 0; i < edges.length && best > 0; i += 4) {
                    best = Math.min(best, Geo.segSegDist(ax, ay, bx, by, edges[i], edges[i + 1], edges[i + 2], edges[i + 3]));
                }
            } else {
                nearest.set(ax, ay, bx, by);
                if (Double.isInfinite(limit)) {
                    grid.box(Math.min(ax, bx) - limit, Math.min(ay, by) - limit, Math.max(ax, bx) + limit,
                            Math.max(ay, by) + limit, nearest);
                } else {
                    grid.corridor(ax, ay, bx, by, limit, nearest);
                }
                best = nearest.best;
            }
            return best < limit ? best : Double.POSITIVE_INFINITY;
        }

        private final class Nearest implements EdgeGrid.Visitor {
            double ax;
            double ay;
            double bx;
            double by;
            double best;

            void set(double ax, double ay, double bx, double by) {
                this.ax = ax;
                this.ay = ay;
                this.bx = bx;
                this.by = by;
                best = Double.POSITIVE_INFINITY;
            }

            @Override
            public boolean visit(int e) {
                int o = e * 4;
                best = Math.min(best, Geo.segSegDist(ax, ay, bx, by, edges[o], edges[o + 1], edges[o + 2], edges[o + 3]));
                return best <= 0;
            }
        }

        private static double[] edgesOf(Geometry g) {
            List<Coordinate[]> seqs = new ArrayList<>();
            sequences(g, seqs);
            int n = 0;
            for (Coordinate[] s : seqs) {
                n += s.length == 1 ? 1 : s.length - 1;
            }
            double[] out = new double[n * 4];
            int k = 0;
            for (Coordinate[] s : seqs) {
                if (s.length == 1) {
                    out[k++] = s[0].x;
                    out[k++] = s[0].y;
                    out[k++] = s[0].x;
                    out[k++] = s[0].y;
                    continue;
                }
                for (int i = 0; i + 1 < s.length; i++) {
                    out[k++] = s[i].x;
                    out[k++] = s[i].y;
                    out[k++] = s[i + 1].x;
                    out[k++] = s[i + 1].y;
                }
            }
            return out;
        }

        private static void sequences(Geometry g, List<Coordinate[]> out) {
            for (int i = 0; i < g.getNumGeometries(); i++) {
                Geometry part = g.getGeometryN(i);
                if (part instanceof Polygon) {
                    Polygon p = (Polygon) part;
                    out.add(p.getExteriorRing().getCoordinates());
                    for (int h = 0; h < p.getNumInteriorRing(); h++) {
                        out.add(p.getInteriorRingN(h).getCoordinates());
                    }
                } else if (!part.isEmpty()) {
                    out.add(part.getCoordinates());
                }
            }
        }
    }

    static final class Zone {
        int index;
        final String id;
        final String type;
        final boolean area;
        final boolean existingNetwork;
        final double kSpec;
        final double extendM;
        final double minSin;
        /** Для площадного объекта — кольца полигона, для линейного — вершины ломаных. */
        final List<double[]> core = new ArrayList<>();
        final List<double[]> band = new ArrayList<>();
        /** Полоса чуть шире проверочной: её углы — вершины графа видимости. */
        Geometry cornerBand;
        final Envelope env = new Envelope();

        Zone(String id, String type, boolean area, boolean existingNetwork, double kSpec, double extendM, double minSin) {
            this.id = id;
            this.type = type;
            this.area = area;
            this.existingNetwork = existingNetwork;
            this.kSpec = kSpec;
            this.extendM = extendM;
            this.minSin = minSin;
        }
    }

    FreeSpace(Scene scene, AppendixModel appendix, Prices prices, Envelope roi, int designDn) {
        this.roi = roi;
        this.designDn = designDn;
        this.margin = prices.width(designDn) / 2;
        List<Geometry> nodeParts = new ArrayList<>();
        List<Geometry> testParts = new ArrayList<>();
        double maxClear = 0;
        for (SpatialConstraint c : scene.constraints) {
            if (c.geometry == null || c.geometry.isEmpty() || !c.geometry.getEnvelopeInternal().intersects(roi)) {
                continue;
            }
            AppendixModel.ConstraintSpec rule = c.rule != null ? c.rule : appendix.constraintRule(c.type);
            double dist = rule.minDistance(designDn) + margin + ownHalfWidth(c.type, c.geometry);
            if (rule.special() || rule.cross()) {
                addZone(c.id, c.type, c.geometry, rule, dist, false);
                continue;
            }
            Avoid a = new Avoid(c.id, c.geometry, dist);
            a.index = avoids.size();
            avoids.add(a);
            maxClear = Math.max(maxClear, dist);
            Geometry raw = c.geometry instanceof Puntal ? c.geometry : simplify(c.geometry);
            nodeParts.add(buffer(raw, dist + SIMPLIFY_M));
            testParts.add(buffer(raw, dist + SIMPLIFY_M - EPS));
        }
        AppendixModel.ConstraintSpec netRule = appendix.constraintRule("heat_network");
        if (netRule.special() || netRule.cross()) {
            for (ExistingSegment s : scene.segments) {
                if (s.line != null && s.line.getEnvelopeInternal().intersects(roi)) {
                    double dist = netRule.minDistance(designDn) + margin + prices.width(s.dn) / 2;
                    addZone(s.id, "heat_network", s.line, netRule, dist, true);
                }
            }
        }
        Geometry node = nodeParts.isEmpty() ? GeoJsonGeometries.GF.createPolygon() : UnaryUnionOp.union(nodeParts);
        Geometry test = testParts.isEmpty() ? GeoJsonGeometries.GF.createPolygon() : UnaryUnionOp.union(testParts);
        this.hard = node;
        this.hardEdges = edges(test, 6.0);
        hardEdges.indexInside();
        this.zoneBoxes = new STRtree();
        for (Zone z : zones) {
            zoneBoxes.insert(z.env, z.index);
        }
        zoneBoxes.build();
        this.avoidBoxes = new STRtree();
        for (Avoid a : avoids) {
            avoidBoxes.insert(a.raw.getEnvelopeInternal(), a.index);
        }
        avoidBoxes.build();
        this.maxClearance = maxClear;
    }

    private static double ownHalfWidth(String type, Geometry g) {
        if (g instanceof Polygonal || type == null) {
            return 0;
        }
        String t = type.toLowerCase(java.util.Locale.ROOT);
        if (t.contains("gas") || t.contains("газ")) {
            return GAS_HALF;
        }
        if (t.contains("cable") || t.contains("power") || t.contains("кабел")) {
            return CABLE_HALF;
        }
        return 0;
    }

    private static Geometry simplify(Geometry g) {
        try {
            Geometry s = TopologyPreservingSimplifier.simplify(g, SIMPLIFY_M);
            return s == null || s.isEmpty() ? g : s;
        } catch (RuntimeException e) {
            return g;
        }
    }

    static Geometry buffer(Geometry g, double d) {
        BufferParameters p = new BufferParameters();
        p.setJoinStyle(BufferParameters.JOIN_MITRE);
        p.setMitreLimit(MITRE_LIMIT);
        p.setQuadrantSegments(2);
        p.setEndCapStyle(BufferParameters.CAP_FLAT);
        if (g instanceof Puntal || g.getDimension() < 2) {
            p.setJoinStyle(BufferParameters.JOIN_ROUND);
            p.setEndCapStyle(BufferParameters.CAP_ROUND);
            p.setQuadrantSegments(2);
            return BufferOp.bufferOp(g, d / Math.cos(Math.PI / 8), p);
        }
        return BufferOp.bufferOp(g, d, p);
    }

    private void addZone(String id, String type, Geometry g, AppendixModel.ConstraintSpec rule, double dist,
                         boolean existingNetwork) {
        boolean area = g instanceof Polygonal;
        double minSin = rule.minAngleDeg == null ? 0 : Math.sin(Math.toRadians(rule.minAngleDeg));
        double k = rule.kSpec > 0 ? rule.kSpec : 1.0;
        Zone z = new Zone(id, type, area, existingNetwork, k, Math.max(0, rule.extendM), minSin);
        if (area) {
            rings(g, z.core);
        } else if (g instanceof LineString || g instanceof MultiLineString) {
            for (int i = 0; i < g.getNumGeometries(); i++) {
                z.core.add(coords(((LineString) g.getGeometryN(i)).getCoordinates()));
            }
        } else {
            return;
        }
        BufferParameters p = new BufferParameters();
        p.setJoinStyle(BufferParameters.JOIN_MITRE);
        p.setMitreLimit(MITRE_LIMIT);
        p.setEndCapStyle(area ? BufferParameters.CAP_FLAT : BufferParameters.CAP_SQUARE);
        rings(BufferOp.bufferOp(g, dist, p), z.band);
        z.cornerBand = BufferOp.bufferOp(g, dist + 2 * EPS, p);
        for (double[] r : z.band) {
            for (int i = 0; i < r.length; i += 2) {
                z.env.expandToInclude(r[i], r[i + 1]);
            }
        }
        z.index = zones.size();
        zones.add(z);
    }

    static void rings(Geometry g, List<double[]> out) {
        for (int i = 0; i < g.getNumGeometries(); i++) {
            Geometry part = g.getGeometryN(i);
            if (part instanceof Polygon) {
                Polygon p = (Polygon) part;
                out.add(coords(p.getExteriorRing().getCoordinates()));
                for (int h = 0; h < p.getNumInteriorRing(); h++) {
                    out.add(coords(p.getInteriorRingN(h).getCoordinates()));
                }
            }
        }
    }

    static double[] coords(Coordinate[] cs) {
        double[] out = new double[cs.length * 2];
        for (int i = 0; i < cs.length; i++) {
            out[i * 2] = cs[i].x;
            out[i * 2 + 1] = cs[i].y;
        }
        return out;
    }

    private static EdgeGrid edges(Geometry g, double cell) {
        List<double[]> rings = new ArrayList<>();
        rings(g, rings);
        int n = 0;
        for (double[] r : rings) {
            int edges = r.length / 2 - 1;
            if (edges > 0) {
                n += edges;
            }
        }
        double[] xy = new double[n * 4];
        int[] owner = new int[n];
        int k = 0;
        for (int ri = 0; ri < rings.size(); ri++) {
            double[] r = rings.get(ri);
            for (int i = 0; i + 3 < r.length; i += 2) {
                xy[k * 4] = r[i];
                xy[k * 4 + 1] = r[i + 1];
                xy[k * 4 + 2] = r[i + 2];
                xy[k * 4 + 3] = r[i + 3];
                owner[k] = ri;
                k++;
            }
        }
        return new EdgeGrid(xy, owner, k, cell);
    }

    boolean pointFree(double x, double y) {
        return !hardEdges.covers(x, y);
    }

    /** Точка не в запрете и не внутри полосы спецобъекта. */
    boolean nodeFree(double x, double y) {
        if (!pointFree(x, y)) {
            return false;
        }
        @SuppressWarnings("unchecked")
        List<Integer> near = zoneBoxes.query(new Envelope(x, x, y, y));
        for (Integer zi : near) {
            if (insideRings(zones.get(zi).band, x, y)) {
                return false;
            }
        }
        return true;
    }

    /** Отрезок не пересекает запрещённые объекты (с зазором). */
    boolean segmentFree(double ax, double ay, double bx, double by) {
        hit.set(ax, ay, bx, by);
        if (hardEdges.walk(ax, ay, bx, by, hit)) {
            return false;
        }
        double len = Geo.dist(ax, ay, bx, by);
        if (len > 4 * EPS) {
            for (int k = 1; k <= 3; k++) {
                double t = k * 0.25;
                if (!pointFree(ax + t * (bx - ax), ay + t * (by - ay))) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Точная проверка по исходным контурам: наибольшее нарушение расстояния (м), 0 — нарушений нет.
     * {@code skip} — индекс объекта, которому ось принадлежит по смыслу (корпус своего ОКС), или -1.
     */
    double violation(double ax, double ay, double bx, double by, int skip) {
        Envelope env = new Envelope(ax, bx, ay, by);
        env.expandBy(maxClearance + 1);
        @SuppressWarnings("unchecked")
        List<Integer> near = avoidBoxes.query(env);
        if (near.isEmpty()) {
            return 0;
        }
        Envelope segEnv = new Envelope(ax, bx, ay, by);
        double worst = 0;
        for (Integer i : near) {
            if (i == skip) {
                continue;
            }
            Avoid a = avoids.get(i);
            if (a.env.distance(segEnv) >= a.clearance) {
                continue;
            }
            worst = Math.max(worst, a.clearance - a.distance(ax, ay, bx, by, a.clearance));
        }
        return worst;
    }

    /** Запрещённый объект, внутри которого лежит точка, или ближайший не дальше maxM; -1 — нет. */
    int avoidAt(double x, double y, double maxM) {
        Envelope env = new Envelope(x - maxM, x + maxM, y - maxM, y + maxM);
        @SuppressWarnings("unchecked")
        List<Integer> near = avoidBoxes.query(env);
        int best = -1;
        double bestD = maxM;
        for (Integer i : near) {
            Avoid a = avoids.get(i);
            if (a.contains(x, y)) {
                return i;
            }
            double d = a.distance(x, y, x, y, maxM + 1e-9);
            if (d <= bestD) {
                bestD = d;
                best = i;
            }
        }
        return best;
    }

    private final class HitVisitor implements EdgeGrid.Visitor {
        double ax;
        double ay;
        double bx;
        double by;

        void set(double ax, double ay, double bx, double by) {
            this.ax = ax;
            this.ay = ay;
            this.bx = bx;
            this.by = by;
        }

        @Override
        public boolean visit(int edge) {
            double[] e = hardEdges.xy;
            int o = edge * 4;
            return Geo.properCross(ax, ay, bx, by, e[o], e[o + 1], e[o + 2], e[o + 3]);
        }
    }

    /**
     * Надбавка к длине за специальные проходы на отрезке ab или NaN, если отрезок идёт вдоль
     * спецобъекта в пределах минимального расстояния либо пересекает дорогу под углом меньше
     * допустимого. {@code tapA/tapB} — конец лежит на существующей теплосети (точка врезки).
     */
    double specialExtra(double ax, double ay, double bx, double by, boolean tapA, boolean tapB) {
        if (zones.isEmpty()) {
            return 0;
        }
        double len = Geo.dist(ax, ay, bx, by);
        if (len < 1e-9) {
            return 0;
        }
        @SuppressWarnings("unchecked")
        List<Integer> near = zoneBoxes.query(new Envelope(ax, bx, ay, by));
        double extra = 0;
        for (Integer zi : near) {
            double add = zoneExtra(zones.get(zi), ax, ay, bx, by, len, tapA, tapB, null);
            if (Double.isNaN(add)) {
                return Double.NaN;
            }
            extra += add;
        }
        return extra;
    }

    /**
     * Интервалы специальных участков вдоль отрезка в долях его длины: {t0, t1, kSpec, zone}; доли могут
     * выходить за [0,1] — участок продолжается на соседние отрезки ломаной. null — отрезок недопустим.
     */
    List<double[]> specialSpans(double ax, double ay, double bx, double by, boolean tapA, boolean tapB) {
        List<double[]> spans = new ArrayList<>();
        double len = Geo.dist(ax, ay, bx, by);
        if (zones.isEmpty() || len < 1e-9) {
            return spans;
        }
        @SuppressWarnings("unchecked")
        List<Integer> near = zoneBoxes.query(new Envelope(ax, bx, ay, by));
        for (Integer zi : near) {
            if (Double.isNaN(zoneExtra(zones.get(zi), ax, ay, bx, by, len, tapA, tapB, spans))) {
                return null;
            }
        }
        return spans;
    }

    /**
     * Специальные участки ломаной в метрах от её начала: {s0, s1, kSpec} по возрастанию без наложений,
     * в пределах ломаной. Где зоны перекрываются — наибольший коэффициент, коэффициенты не складываются.
     * Смена набора ограничений начинает новый участок, даже если коэффициент тот же.
     * null — ломаная недопустима. Стоимость трубы считается по этим же интервалам.
     */
    List<double[]> pathSpans(double[] path, boolean tapStart, boolean tapEnd) {
        List<double[]> raw = new ArrayList<>();
        double total = Geo.length(path);
        int legs = path.length / 2 - 1;
        double acc = 0;
        for (int i = 0; i < legs; i++) {
            double ax = path[i * 2];
            double ay = path[i * 2 + 1];
            double bx = path[i * 2 + 2];
            double by = path[i * 2 + 3];
            double len = Geo.dist(ax, ay, bx, by);
            if (len < 1e-9) {
                continue;
            }
            List<double[]> spans = specialSpans(ax, ay, bx, by, tapStart && i == 0, tapEnd && i == legs - 1);
            if (spans == null) {
                return null;
            }
            for (double[] s : spans) {
                double s0 = acc + s[0] * len;
                double s1 = acc + s[1] * len;
                s0 = s0 < MIN_PIECE_M ? 0 : s0;
                s1 = s1 > total - MIN_PIECE_M ? total : s1;
                if (s1 > s0 + 1e-9) {
                    raw.add(new double[]{s0, s1, s[2], s[3]});
                }
            }
            acc += len;
        }
        return mergeSpans(raw);
    }

    /** Надбавка спецпроходов ломаной в метрах: Σ (Kспец − 1) · длина участка; NaN — ломаная недопустима. */
    double pathExtra(double[] path, boolean tapStart, boolean tapEnd) {
        List<double[]> spans = pathSpans(path, tapStart, tapEnd);
        if (spans == null) {
            return Double.NaN;
        }
        double extra = 0;
        for (double[] s : spans) {
            extra += (s[2] - 1) * (s[1] - s[0]);
        }
        return extra;
    }

    static List<double[]> mergeSpans(List<double[]> raw) {
        List<double[]> out = new ArrayList<>();
        if (raw.isEmpty()) {
            return out;
        }
        double[] cuts = new double[raw.size() * 2];
        for (int i = 0; i < raw.size(); i++) {
            cuts[i * 2] = raw.get(i)[0];
            cuts[i * 2 + 1] = raw.get(i)[1];
        }
        java.util.Arrays.sort(cuts);
        int[] zones = new int[raw.size()];
        int[] prevZones = new int[raw.size()];
        int prevCount = -1;
        for (int i = 0; i + 1 < cuts.length; i++) {
            double a = cuts[i];
            double b = cuts[i + 1];
            if (b - a < 1e-9) {
                continue;
            }
            double m = (a + b) / 2;
            double k = 0;
            int count = 0;
            for (double[] s : raw) {
                if (s[0] <= m && m <= s[1]) {
                    k = Math.max(k, s[2]);
                    int zone = s.length > 3 ? (int) Math.round(s[3]) : -1;
                    boolean seen = false;
                    for (int t = 0; t < count; t++) {
                        if (zones[t] == zone) {
                            seen = true;
                            break;
                        }
                    }
                    if (!seen) {
                        zones[count++] = zone;
                    }
                }
            }
            if (k <= 0) {
                prevCount = -1;
                continue;
            }
            java.util.Arrays.sort(zones, 0, count);
            double[] last = out.isEmpty() ? null : out.get(out.size() - 1);
            if (last != null && prevCount == count && Math.abs(last[1] - a) < 1e-9 && Math.abs(last[2] - k) < 1e-12
                    && sameZones(prevZones, zones, count)) {
                last[1] = b;
            } else {
                out.add(new double[]{a, b, k});
                prevCount = count;
                System.arraycopy(zones, 0, prevZones, 0, count);
            }
        }
        return out;
    }

    private static boolean sameZones(int[] left, int[] right, int count) {
        for (int i = 0; i < count; i++) {
            if (left[i] != right[i]) {
                return false;
            }
        }
        return true;
    }

    /**
     * Для площадного объекта специальный участок — внутренность плюс extendM с каждой стороны.
     * Если конец отрезка — камера уже внутри полигона, extendM с её стороны не добавляется:
     * 3 м остаются только за границей со стороны выхода. Для линейного объекта — ±extendM от точки
     * пересечения вдоль оси, без половины габарита новой сети.
     */
    private double zoneExtra(Zone z, double ax, double ay, double bx, double by, double len,
                             boolean tapA, boolean tapB, List<double[]> spans) {
        List<double[]> bandHits = new ArrayList<>();
        crossings(z.band, ax, ay, bx, by, bandHits);
        boolean startIn = insideRings(z.band, ax, ay);
        if (bandHits.isEmpty() && !startIn) {
            return 0;
        }
        List<Double> cuts = new ArrayList<>();
        cuts.add(0.0);
        for (double[] h : bandHits) {
            cuts.add(h[0]);
        }
        cuts.add(1.0);
        cuts.sort(Double::compare);
        List<double[]> coreHits = new ArrayList<>();
        crossings(z.core, ax, ay, bx, by, coreHits);
        double extra = 0;
        int zi = z.index;
        for (int i = 0; i + 1 < cuts.size(); i++) {
            double t0 = cuts.get(i);
            double t1 = cuts.get(i + 1);
            if (t1 - t0 < 1e-12) {
                continue;
            }
            double tm = (t0 + t1) / 2;
            if (!insideRings(z.band, ax + tm * (bx - ax), ay + tm * (by - ay))) {
                continue;
            }
            int crossingsIn = 0;
            boolean steep = true;
            for (double[] h : coreHits) {
                if (h[0] >= t0 - 1e-12 && h[0] <= t1 + 1e-12) {
                    crossingsIn++;
                    if (h[1] < z.minSin - 1e-9) {
                        steep = false;
                    }
                }
            }
            boolean atTapEnd = z.existingNetwork && ((tapA && t0 <= 1e-9) || (tapB && t1 >= 1 - 1e-9));
            if (atTapEnd) {
                if (!tapExitOk(z, ax, ay, bx, by, tapA && t0 <= 1e-9)) {
                    return Double.NaN;
                }
                if (crossingsIn <= 1) {
                    continue;
                }
            }
            if (crossingsIn == 0 || !steep) {
                return Double.NaN;
            }
            if (z.area) {
                double inside = 0;
                List<Double> cc = new ArrayList<>();
                cc.add(t0);
                for (double[] h : coreHits) {
                    if (h[0] > t0 && h[0] < t1) {
                        cc.add(h[0]);
                    }
                }
                cc.add(t1);
                double first = Double.NaN;
                double last = Double.NaN;
                for (int k = 0; k + 1 < cc.size(); k++) {
                    double m = (cc.get(k) + cc.get(k + 1)) / 2;
                    if (insideRings(z.core, ax + m * (bx - ax), ay + m * (by - ay))) {
                        inside += (cc.get(k + 1) - cc.get(k)) * len;
                        if (Double.isNaN(first)) {
                            first = cc.get(k);
                        }
                        last = cc.get(k + 1);
                    }
                }
                boolean chamberStart = tapA && insideRings(z.core, ax, ay);
                boolean chamberEnd = tapB && insideRings(z.core, bx, by);
                double ext0 = chamberStart ? 0 : z.extendM;
                double ext1 = chamberEnd ? 0 : z.extendM;
                double special = inside + ext0 + ext1;
                extra += (z.kSpec - 1) * special;
                if (spans != null && !Double.isNaN(first)) {
                    spans.add(new double[]{first - ext0 / len, last + ext1 / len, z.kSpec, zi});
                }
            } else {
                for (double[] h : coreHits) {
                    if (h[0] >= t0 - 1e-12 && h[0] <= t1 + 1e-12) {
                        boolean tapPoint = (tapA && h[0] < 1e-6) || (tapB && h[0] > 1 - 1e-6);
                        if (tapPoint) {
                            continue;
                        }
                        extra += (z.kSpec - 1) * 2 * z.extendM;
                        if (spans != null) {
                            spans.add(new double[]{h[0] - z.extendM / len, h[0] + z.extendM / len, z.kSpec, zi});
                        }
                    }
                }
            }
        }
        return extra;
    }

    /** Выход от точки врезки не вдоль трубы: угол к ближайшему куску не меньше 30°. */
    private boolean tapExitOk(Zone z, double ax, double ay, double bx, double by, boolean fromA) {
        double px = fromA ? ax : bx;
        double py = fromA ? ay : by;
        double best = Double.POSITIVE_INFINITY;
        double sin = 1;
        for (double[] line : z.core) {
            for (int i = 0; i + 3 < line.length; i += 2) {
                double d = Geo.segDist(line[i], line[i + 1], line[i + 2], line[i + 3], px, py);
                if (d < best) {
                    best = d;
                    sin = Geo.sinAngle(ax, ay, bx, by, line[i], line[i + 1], line[i + 2], line[i + 3]);
                }
            }
        }
        return sin >= TAP_EXIT_MIN_SIN;
    }

    /** Пересечения ab с кольцами/ломаными: {t, sinУгла}. */
    private static void crossings(List<double[]> lines, double ax, double ay, double bx, double by,
                                  List<double[]> out) {
        for (double[] r : lines) {
            for (int i = 0; i + 3 < r.length; i += 2) {
                double t = Geo.crossParam(ax, ay, bx, by, r[i], r[i + 1], r[i + 2], r[i + 3]);
                if (!Double.isNaN(t)) {
                    out.add(new double[]{t, Geo.sinAngle(ax, ay, bx, by, r[i], r[i + 1], r[i + 2], r[i + 3])});
                }
            }
        }
        out.sort((p, q) -> Double.compare(p[0], q[0]));
    }

    static boolean insideRings(List<double[]> rings, double x, double y) {
        boolean in = false;
        for (double[] r : rings) {
            for (int i = 0, j = r.length - 2; i < r.length; j = i, i += 2) {
                double xi = r[i];
                double yi = r[i + 1];
                double xj = r[j];
                double yj = r[j + 1];
                if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
                    in = !in;
                }
            }
        }
        return in;
    }
}
