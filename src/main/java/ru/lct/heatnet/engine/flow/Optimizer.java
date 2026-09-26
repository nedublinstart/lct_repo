package ru.lct.heatnet.engine.flow;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * Поиск леса минимальной стоимости.
 * <p>
 * Основной ход — переподвешивание поддерева: поддерево отсоединяется, из его верхней точки идёт
 * Дейкстра по графу видимости, и для каждой окончательной вершины перебираются способы подключения:
 * основание перпендикуляра на ребро дерева (новая камера), существующая камера разветвления со
 * свободным местом, место врезки на существующей сети. Для каждого способа приращение стоимости
 * оценивается снизу (труба по DN расхода поддерева, камера, подъём DN выше по дереву, реконструкция);
 * поиск останавливается, когда уже пройденная длина стоит больше лучших кандидатов. Несколько лучших
 * кандидатов считаются точно ({@link Model}).
 * <p>
 * Начальный лес — последовательная вставка ОКС в нескольких порядках; затем улучшение ходами до
 * локального минимума и итерированный локальный поиск (перевставка нескольких ОКС).
 */
final class Optimizer {

    private static final org.slf4j.Logger log = org.slf4j.LoggerFactory.getLogger(Optimizer.class);
    private static final int TOP_K = 6;
    private static final int EDGE = 0;
    private static final int NODE = 1;
    private static final int TAPC = 2;
    private static final double IMPROVE_EPS = 1.0;

    final Model model;
    final VisGraph g;
    final Search search;
    final Taps taps;
    final Ports ports;
    final FreeSpace space;
    final Prices prices;
    final Forest.Legs legs;
    private final SubsetDp dp;
    private final Random rnd;
    private long deadline;

    Optimizer(Model model, VisGraph g, Taps taps, Ports ports, FreeSpace space, long seed) {
        this.model = model;
        this.g = g;
        this.search = new Search(g);
        this.taps = taps;
        this.ports = ports;
        this.space = space;
        this.prices = model.prices;
        this.rnd = new Random(seed);
        this.legs = (path, tapEnd) -> space.pathExtra(path, false, tapEnd);
        this.dp = new SubsetDp(g, search, ports, prices);
    }

    // ------------------------------------------------------------------ верхний уровень

    Forest solve(List<Forest> seeds, long budgetNanos) {
        deadline = System.nanoTime() + budgetNanos;
        Forest best = null;
        double bestObj = Double.POSITIVE_INFINITY;
        List<Forest> starts = new ArrayList<>();
        for (Forest s : seeds) {
            starts.add(s.copy());
        }
        List<List<Integer>> orders = orders();
        for (List<Integer> order : orders) {
            if (timeUp() && !starts.isEmpty()) {
                break;
            }
            starts.add(construct(order));
        }
        for (Forest f : starts) {
            if (timeUp() && best != null) {
                break;
            }
            f = improve(f);
            double obj = model.evaluate(f).objective;
            if (obj < bestObj) {
                bestObj = obj;
                best = f;
            }
        }
        if (best == null) {
            best = new Forest(legs);
        }
        double startObj = bestObj;
        Forest cur = best.copy();
        double curObj = bestObj;
        int stale = 0;
        int iter = 0;
        int rebuilds = 0;
        int rebuildWins = 0;
        long lastWin = System.nanoTime();
        while (!timeUp()) {
            boolean dpMove = rnd.nextBoolean();
            Forest trial = dpMove ? rebuild(cur, cluster(cur)) : perturb(cur);
            iter++;
            if (trial == null) {
                continue;
            }
            if (dpMove) {
                rebuilds++;
            }
            trial = improve(trial);
            double obj = model.evaluate(trial).objective;
            if (obj < curObj - IMPROVE_EPS) {
                cur = trial;
                curObj = obj;
                stale = 0;
                if (obj < bestObj - IMPROVE_EPS) {
                    best = trial.copy();
                    bestObj = obj;
                    lastWin = System.nanoTime();
                    if (dpMove) {
                        rebuildWins++;
                    }
                }
            } else if (++stale > 12) {
                cur = best.copy();
                curObj = bestObj;
                stale = 0;
            }
        }
        log.debug("Лес: старт {} → {} ₽, итераций {}, перестроек группы {} (улучшили рекорд {}), рекорд за {} мс до конца",
                Math.round(startObj), Math.round(bestObj), iter, rebuilds, rebuildWins,
                Math.max(0, (deadline - lastWin) / 1_000_000));
        model.evaluate(best);
        return best;
    }

    private boolean timeUp() {
        return System.nanoTime() > deadline;
    }

    private List<List<Integer>> orders() {
        int n = ports.terms.size();
        double[] near = new double[n];
        for (int i = 0; i < n; i++) {
            Ports.Terminal t = ports.terms.get(i);
            double best = Double.POSITIVE_INFINITY;
            double[] p = model.net.pieces;
            for (int k = 0; k < model.net.pieceCount(); k++) {
                best = Math.min(best, Geo.segDist(p[k * 4], p[k * 4 + 1], p[k * 4 + 2], p[k * 4 + 3], t.x, t.y));
            }
            near[i] = best;
        }
        List<Integer> base = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            base.add(i);
        }
        List<List<Integer>> out = new ArrayList<>();
        List<Integer> byNear = new ArrayList<>(base);
        byNear.sort((a, b) -> Double.compare(near[a], near[b]));
        out.add(byNear);
        List<Integer> byFlow = new ArrayList<>(base);
        byFlow.sort((a, b) -> Double.compare(ports.terms.get(b).flow, ports.terms.get(a).flow));
        out.add(byFlow);
        List<Integer> byFar = new ArrayList<>(byNear);
        Collections.reverse(byFar);
        out.add(byFar);
        for (int r = 0; r < Integer.getInteger("heatnet.flow.randomOrders", 3); r++) {
            List<Integer> rnd = new ArrayList<>(base);
            Collections.shuffle(rnd, this.rnd);
            out.add(rnd);
        }
        return out;
    }

    Forest construct(List<Integer> order) {
        Forest f = new Forest(legs);
        for (int t : order) {
            Ports.Terminal term = ports.terms.get(t);
            Forest.Node x = f.add(Forest.LEAF, term.x, term.y);
            x.term = t;
            Forest next = attachBest(f, x, Double.POSITIVE_INFINITY, 0);
            if (next != null) {
                f = next;
            }
        }
        return f;
    }

    /** Переподвешивание каждого узла до локального минимума. */
    Forest improve(Forest f) {
        double cur = model.evaluate(f).objective;
        boolean changed = true;
        int rounds = 0;
        while (changed && !timeUp() && rounds++ < 30) {
            changed = false;
            List<Integer> ids = new ArrayList<>();
            for (Forest.Node n : f.nodes) {
                if (n.type != Forest.TAP && n.parent != null) {
                    ids.add(n.id);
                }
            }
            Collections.shuffle(ids, rnd);
            for (int id : ids) {
                if (timeUp()) {
                    break;
                }
                Forest.Node u = f.byId(id);
                if (u == null || u.parent == null) {
                    continue;
                }
                Forest trial = f.copy();
                Forest.Node x = trial.byId(id);
                trial.detach(x);
                Model.Eval base = model.evaluate(trial);
                double baseObj = base.objective - model.penaltyOf(x);
                Forest next = attachBest(trial, x, cur - baseObj - IMPROVE_EPS, baseObj);
                if (next == null) {
                    continue;
                }
                double obj = model.evaluate(next).objective;
                if (obj < cur - IMPROVE_EPS) {
                    f = next;
                    cur = obj;
                    changed = true;
                }
            }
            Forest polished = f.copy();
            if (polish(polished)) {
                double obj = model.evaluate(polished).objective;
                if (obj < cur - IMPROVE_EPS) {
                    f = polished;
                    cur = obj;
                    changed = true;
                }
            }
        }
        return f;
    }

    // ------------------------------------------------------------------ геометрия при той же топологии

    /**
     * Камеры разветвления сдвигаются в взвешенную точку Ферма соседних вершин, затем каждое ребро
     * перекладывается кратчайшей планарной ломаной между теми же концами. Лес меняется на месте
     * (новые массивы ломаных, общие с копиями массивы не трогаются).
     */
    private boolean polish(Forest f) {
        boolean any = false;
        for (int pass = 0; pass < 4 && !timeUp(); pass++) {
            boolean changed = false;
            for (Forest.Node n : new ArrayList<>(f.nodes)) {
                if (n.type == Forest.JUNC && relocate(f, n)) {
                    changed = true;
                }
            }
            for (Forest.Node n : new ArrayList<>(f.nodes)) {
                if (n.type != Forest.TAP && n.parent != null && tighten(f, n)) {
                    changed = true;
                }
            }
            if (!changed) {
                break;
            }
            any = true;
        }
        return any;
    }

    /** Кратчайшая планарная ломаная ребра n → родитель при тех же концах; true — ребро стало дешевле. */
    private boolean tighten(Forest f, Forest.Node n) {
        Forest.Node p = n.parent;
        double[] cur = n.path;
        if (p == null || cur == null || cur.length < 4) {
            return false;
        }
        double tx = cur[cur.length - 2];
        double ty = cur[cur.length - 1];
        boolean tapEnd = p.type == Forest.TAP;
        obstaclesExcept(f, n);
        block(f, n);
        for (int i = 0; i < cur.length; i += 2) {
            int v = g.nodeAt(cur[i], cur[i + 1]);
            if (v >= 0) {
                search.blocked[v] = false;
            }
        }
        search.filter(this::legAllowed);
        startSearch(n);
        double limit = n.len + n.extra - 1e-3;
        int bestV = -1;
        while (true) {
            int v = search.next();
            if (v < 0) {
                break;
            }
            double d = search.dist[v];
            if (d >= limit) {
                break;
            }
            double vx = search.x(v);
            double vy = search.y(v);
            double leg;
            if (tapEnd) {
                leg = legAllowed(vx, vy, tx, ty) ? taps.legTo(vx, vy, tx, ty) : Double.NaN;
            } else {
                leg = legCost(vx, vy, tx, ty, false);
            }
            if (!Double.isNaN(leg) && d + leg < limit) {
                limit = d + leg;
                bestV = v;
            }
        }
        double[] path = null;
        if (bestV >= 0) {
            double[] core = search.path(bestV);
            double[] out = new double[core.length + 4];
            int k = 0;
            if (n.type == Forest.LEAF) {
                out[k++] = n.x;
                out[k++] = n.y;
            }
            for (int i = 0; i < core.length; i += 2) {
                k = push(out, k, core[i], core[i + 1]);
            }
            k = push(out, k, tx, ty);
            path = k >= 4 ? java.util.Arrays.copyOf(out, k) : null;
        }
        unblock();
        search.filter(null);
        if (path == null) {
            return false;
        }
        double extra = legs.extra(path, tapEnd);
        double len = Geo.length(path);
        if (Double.isNaN(extra) || len + extra >= n.len + n.extra - 1e-3) {
            return false;
        }
        n.path = path;
        n.len = len;
        n.extra = extra;
        return true;
    }

    /**
     * Камера j переезжает в точку минимума Σ цена(DN) · расстояние до соседних вершин своих ломаных
     * (итерации Вайсфельда); если отрезки до соседей не проходят, шаг уменьшается вдвое.
     */
    private boolean relocate(Forest f, Forest.Node j) {
        if (j.parent == null || j.path == null || j.kids.size() < 2 || j.path.length < 4) {
            return false;
        }
        int m = j.kids.size() + 1;
        double[] ax = new double[m];
        double[] ay = new double[m];
        double[] w = new double[m];
        ax[0] = j.path[2];
        ay[0] = j.path[3];
        w[0] = prices.perM(j.dn);
        for (int i = 0; i < j.kids.size(); i++) {
            double[] kp = j.kids.get(i).path;
            if (kp == null || kp.length < 4) {
                return false;
            }
            ax[i + 1] = kp[kp.length - 4];
            ay[i + 1] = kp[kp.length - 3];
            w[i + 1] = prices.perM(j.kids.get(i).dn);
        }
        double x = j.x;
        double y = j.y;
        for (int it = 0; it < 60; it++) {
            double sx = 0;
            double sy = 0;
            double sw = 0;
            for (int i = 0; i < m; i++) {
                double d = Math.max(1e-6, Geo.dist(x, y, ax[i], ay[i]));
                sx += w[i] * ax[i] / d;
                sy += w[i] * ay[i] / d;
                sw += w[i] / d;
            }
            double nx = sx / sw;
            double ny = sy / sw;
            double step = Geo.dist(x, y, nx, ny);
            x = nx;
            y = ny;
            if (step < 1e-3) {
                break;
            }
        }
        boolean tapEnd = j.parent.type == Forest.TAP && j.path.length == 4;
        double base = weighted(j.x, j.y, ax, ay, w, m);
        for (double lam = 1.0; lam >= 0.06; lam /= 2) {
            double qx = j.x + lam * (x - j.x);
            double qy = j.y + lam * (y - j.y);
            if (weighted(qx, qy, ax, ay, w, m) >= base - 100.0 || !space.nodeFree(qx, qy)) {
                continue;
            }
            if (!legsFeasible(f, j, qx, qy, ax, ay, m, tapEnd)) {
                continue;
            }
            double[] up = j.path.clone();
            up[0] = qx;
            up[1] = qy;
            double upExtra = legs.extra(up, j.parent.type == Forest.TAP);
            if (Double.isNaN(upExtra)) {
                continue;
            }
            double[][] kidPaths = new double[j.kids.size()][];
            double[] kidExtra = new double[j.kids.size()];
            boolean ok = true;
            for (int i = 0; i < j.kids.size() && ok; i++) {
                double[] kp = j.kids.get(i).path.clone();
                kp[kp.length - 2] = qx;
                kp[kp.length - 1] = qy;
                kidPaths[i] = kp;
                kidExtra[i] = legs.extra(kp, false);
                ok = !Double.isNaN(kidExtra[i]);
            }
            if (!ok) {
                continue;
            }
            j.x = qx;
            j.y = qy;
            j.path = up;
            j.len = Geo.length(up);
            j.extra = upExtra;
            for (int i = 0; i < j.kids.size(); i++) {
                Forest.Node k = j.kids.get(i);
                k.path = kidPaths[i];
                k.len = Geo.length(kidPaths[i]);
                k.extra = kidExtra[i];
            }
            return true;
        }
        return false;
    }

    private static double weighted(double x, double y, double[] ax, double[] ay, double[] w, int m) {
        double s = 0;
        for (int i = 0; i < m; i++) {
            s += w[i] * Geo.dist(x, y, ax[i], ay[i]);
        }
        return s;
    }

    /** Новые отрезки от камеры до соседей: свободны и не пересекают остальные трубы леса. */
    private boolean legsFeasible(Forest f, Forest.Node j, double qx, double qy, double[] ax, double[] ay, int m,
                                 boolean tapEnd) {
        for (int i = 0; i < m; i++) {
            if (i == 0 && tapEnd) {
                if (Double.isNaN(taps.legTo(qx, qy, ax[0], ay[0]))) {
                    return false;
                }
            } else if (!space.segmentFree(qx, qy, ax[i], ay[i])) {
                return false;
            }
        }
        for (Forest.Node n : f.nodes) {
            double[] p = n.path;
            if (p == null) {
                continue;
            }
            int from = 0;
            int to = p.length / 2 - 1;
            if (n == j) {
                from = 1;
            } else if (n.parent == j) {
                to -= 1;
            }
            for (int s = from; s < to; s++) {
                double sx = p[s * 2];
                double sy = p[s * 2 + 1];
                double ex = p[s * 2 + 2];
                double ey = p[s * 2 + 3];
                for (int i = 0; i < m; i++) {
                    if (Geo.properCross(qx, qy, ax[i], ay[i], sx, sy, ex, ey)) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    /** Сетка отрезков леса без ломаной узла skip (для перекладки ребра). */
    private void obstaclesExcept(Forest f, Forest.Node skip) {
        int count = 0;
        for (Forest.Node n : f.nodes) {
            if (n.path != null && n != skip) {
                count += n.path.length / 2 - 1;
            }
        }
        double[] xy = new double[Math.max(1, count) * 4];
        int[] owner = new int[Math.max(1, count)];
        int k = 0;
        for (Forest.Node n : f.nodes) {
            if (n.path == null || n == skip) {
                continue;
            }
            for (int i = 0; i + 3 < n.path.length; i += 2) {
                xy[k * 4] = n.path[i];
                xy[k * 4 + 1] = n.path[i + 1];
                xy[k * 4 + 2] = n.path[i + 2];
                xy[k * 4 + 3] = n.path[i + 3];
                owner[k] = n.id;
                k++;
            }
        }
        treeGrid = new EdgeGrid(xy, owner, k, 15.0);
    }

    private Forest perturb(Forest cur) {
        Forest f = cur.copy();
        List<Forest.Node> leaves = new ArrayList<>();
        for (Forest.Node n : f.nodes) {
            if (n.type == Forest.LEAF) {
                leaves.add(n);
            }
        }
        if (leaves.isEmpty()) {
            return f;
        }
        Collections.shuffle(leaves, rnd);
        int k = Math.min(leaves.size(), 2 + rnd.nextInt(3));
        List<Forest.Node> moved = new ArrayList<>(leaves.subList(0, k));
        for (Forest.Node x : moved) {
            f.detach(x);
        }
        for (Forest.Node x : moved) {
            Forest next = attachBest(f, f.byId(x.id), Double.POSITIVE_INFINITY, 0);
            if (next != null) {
                f = next;
            }
        }
        return f;
    }

    // ------------------------------------------------------------------ перестройка группы ОКС

    /** Соседние ОКС: случайный ОКС и ближайшие к нему, от 3 до {@link SubsetDp#MAX_UNITS}. */
    List<Integer> cluster(Forest f) {
        List<Forest.Node> leaves = new ArrayList<>();
        for (Forest.Node n : f.nodes) {
            if (n.type == Forest.LEAF) {
                leaves.add(n);
            }
        }
        int max = Math.min(SubsetDp.MAX_UNITS, leaves.size());
        List<Integer> out = new ArrayList<>();
        if (max < 2) {
            return out;
        }
        int k = Math.min(max, 3 + rnd.nextInt(Math.max(1, max - 2)));
        Forest.Node seed = leaves.get(rnd.nextInt(leaves.size()));
        leaves.sort((p, q) -> Double.compare(Geo.dist(p.x, p.y, seed.x, seed.y), Geo.dist(q.x, q.y, seed.x, seed.y)));
        for (int i = 0; i < k; i++) {
            out.add(leaves.get(i).id);
        }
        return out;
    }

    /**
     * Большая окрестность: ОКС группы отсоединяются и подключаются заново. {@link SubsetDp} даёт
     * оптимальное дерево каждого подмножества группы, поиск подключения — цену его врезки в
     * оставшийся лес; перебор разбиений группы за O(3^k) выбирает, какие ОКС идут общим деревом и
     * где каждое дерево подключается. Деревья строятся по порядку убывания расхода, подключение
     * каждого уточняется точной оценкой. null — группа не перестраивается.
     */
    Forest rebuild(Forest cur, List<Integer> leafIds) {
        int k = leafIds.size();
        if (k < 2 || k > SubsetDp.MAX_UNITS) {
            return null;
        }
        Forest f = cur.copy();
        int[] terms = new int[k];
        int[] ids = new int[k];
        for (int i = 0; i < k; i++) {
            Forest.Node x = f.byId(leafIds.get(i));
            if (x == null || x.type != Forest.LEAF) {
                return null;
            }
            f.detach(x);
            terms[i] = x.term;
            ids[i] = x.id;
        }
        int subsets = 1 << k;
        double[] cost = new double[subsets];
        int[] top = new int[subsets];
        boolean ok;
        context(f, null);
        block(f, null);
        search.filter(this::legAllowed);
        search.memoize(true);
        try {
            ok = dp.run(terms);
            topK = 1;
            for (int s = 1; ok && s < subsets; s++) {
                cost[s] = groupCost(s, top);
            }
        } finally {
            topK = TOP_K;
            search.memoize(false);
            search.filter(null);
            unblock();
        }
        try {
            if (!ok) {
                return null;
            }
            double[] part = new double[subsets];
            int[] choice = new int[subsets];
            for (int s = 1; s < subsets; s++) {
                part[s] = cost[s];
                choice[s] = s;
                int low = s & -s;
                for (int grp = (s - 1) & s; grp > 0; grp = (grp - 1) & s) {
                    if ((grp & low) != 0 && cost[grp] + part[s ^ grp] < part[s]) {
                        part[s] = cost[grp] + part[s ^ grp];
                        choice[s] = grp;
                    }
                }
            }
            if (!(part[subsets - 1] < Double.POSITIVE_INFINITY)) {
                return null;
            }
            List<Integer> groups = new ArrayList<>();
            for (int s = subsets - 1; s != 0; s ^= choice[s]) {
                groups.add(choice[s]);
            }
            groups.sort((p, q) -> Double.compare(dp.flow[q], dp.flow[p]));
            for (int grp : groups) {
                f = place(f, grp, top[grp], ids);
            }
            return f;
        } finally {
            dp.release();
        }
    }

    /**
     * Цена подключения дерева подмножества s: дерево из динамики плюс лучшая врезка в лес (оценка
     * снизу, как у переподвешивания). В top[s] — вершина окна верхней камеры дерева.
     */
    private double groupCost(int s, int[] top) {
        setFlow(dp.flow[s]);
        int base = s * dp.w;
        if (starts.length < dp.w) {
            starts = new int[dp.w];
            inits = new double[dp.w];
        }
        int count = 0;
        for (int j = 0; j < dp.w; j++) {
            double a = dp.a[base + j];
            if (a < Double.POSITIVE_INFINITY) {
                starts[count] = dp.win[j];
                inits[count] = a / cPrice;
                count++;
            }
        }
        if (count == 0) {
            return Double.POSITIVE_INFINITY;
        }
        search.startNodes(starts, inits, count);
        while (true) {
            int v = search.next();
            if (v < 0) {
                break;
            }
            double d = search.dist[v];
            double stop = topCount > 0 ? topVals[0] : Double.POSITIVE_INFINITY;
            if (d * cPrice >= stop) {
                break;
            }
            expand(v, d, stop);
        }
        Cand bestC = null;
        for (Cand c : best.values()) {
            if (bestC == null || c.approx < bestC.approx) {
                bestC = c;
            }
        }
        if (bestC == null) {
            return Double.POSITIVE_INFINITY;
        }
        int j = dp.index(search.origin(bestC.vg));
        if (j < 0) {
            return Double.POSITIVE_INFINITY;
        }
        top[s] = dp.chain(s, j)[0];
        return bestC.approx;
    }

    private int[] starts = new int[64];
    private double[] inits = new double[64];

    /** Узел плана дерева группы: камера в вершине графа или ИТП; путь — от узла к родителю. */
    private static final class Plan {
        int vertex = -1;
        int unit = -1;
        double[] path;
        int[] verts;
        double extra;
        final List<Plan> kids = new ArrayList<>(3);
    }

    /** Дерево подмножества s по плану динамики; если не выходит — ОКС подключаются по одному. */
    private Forest place(Forest f, int s, int topJ, int[] ids) {
        if (Integer.bitCount(s) == 1) {
            Forest next = attachBest(f, f.byId(ids[Integer.numberOfTrailingZeros(s)]), Double.POSITIVE_INFINITY, 0);
            return next != null ? next : f;
        }
        Plan p = planChamber(s, topJ);
        if (p != null && planValid(f, p)) {
            Forest t = f.copy();
            Forest.Node root = materialize(t, p, ids);
            Forest next = attachBest(t, root, Double.POSITIVE_INFINITY, 0);
            if (next != null) {
                return next;
            }
        }
        List<Integer> units = new ArrayList<>();
        for (int i = 0; i < dp.k; i++) {
            if ((s >> i & 1) != 0) {
                units.add(i);
            }
        }
        units.sort((a, b) -> Double.compare(dp.flow[1 << b], dp.flow[1 << a]));
        for (int i : units) {
            Forest next = attachBest(f, f.byId(ids[i]), Double.POSITIVE_INFINITY, 0);
            if (next != null) {
                f = next;
            }
        }
        return f;
    }

    /** Камера подмножества s в вершине окна j с двумя или тремя нижними поддеревьями. */
    private Plan planChamber(int s, int j) {
        Plan p = new Plan();
        p.vertex = dp.win[j];
        int at = s * dp.w + j;
        int three = dp.s3[at];
        boolean ok;
        if (three < 0) {
            int s1 = dp.s2[at];
            ok = addKid(p, s1, j) && addKid(p, s ^ s1, j);
        } else {
            int s1 = dp.s2[three * dp.w + j];
            ok = addKid(p, s1, j) && addKid(p, three ^ s1, j) && addKid(p, s ^ three, j);
        }
        return ok && p.kids.size() <= 3 ? p : null;
    }

    private boolean addKid(Plan parent, int s, int j) {
        int[] chain = dp.chain(s, j);
        Plan kid;
        int off;
        if (Integer.bitCount(s) == 1) {
            kid = new Plan();
            kid.unit = Integer.numberOfTrailingZeros(s);
            Ports.Terminal t = ports.terms.get(dp.terms[kid.unit]);
            kid.path = new double[chain.length * 2 + 2];
            kid.path[0] = t.x;
            kid.path[1] = t.y;
            off = 2;
        } else {
            kid = planChamber(s, chain[0]);
            if (kid == null) {
                return false;
            }
            if (chain.length == 1) {
                parent.kids.addAll(kid.kids);
                return true;
            }
            kid.path = new double[chain.length * 2];
            off = 0;
        }
        kid.verts = new int[chain.length];
        for (int i = 0; i < chain.length; i++) {
            int v = dp.win[chain[i]];
            kid.verts[i] = v;
            kid.path[off + i * 2] = g.x[v];
            kid.path[off + i * 2 + 1] = g.y[v];
        }
        kid.path = Forest.dedupe(kid.path);
        parent.kids.add(kid);
        return true;
    }

    /** Вершины плана не заняты и не повторяются, трубы не пересекают лес и друг друга. */
    private boolean planValid(Forest f, Plan root) {
        List<double[]> segs = new ArrayList<>();
        java.util.Set<Integer> used = new java.util.HashSet<>();
        used.add(root.vertex);
        if (!collectPlan(root, segs, used)) {
            return false;
        }
        for (Forest.Node n : f.nodes) {
            if (n.path == null) {
                continue;
            }
            for (int i = 0; i < n.path.length; i += 2) {
                int v = g.nodeAt(n.path[i], n.path[i + 1]);
                if (v >= 0 && used.contains(v)) {
                    return false;
                }
            }
        }
        obstaclesExcept(f, null);
        for (double[] s : segs) {
            if (!legAllowed(s[0], s[1], s[2], s[3])) {
                return false;
            }
        }
        for (int i = 0; i < segs.size(); i++) {
            double[] a = segs.get(i);
            for (int j = i + 1; j < segs.size(); j++) {
                double[] b = segs.get(j);
                if (Geo.properCross(a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3])) {
                    return false;
                }
            }
        }
        return true;
    }

    private boolean collectPlan(Plan p, List<double[]> segs, java.util.Set<Integer> used) {
        for (Plan k : p.kids) {
            k.extra = legs.extra(k.path, false);
            if (Double.isNaN(k.extra)) {
                return false;
            }
            for (int i = 0; i + 1 < k.verts.length; i++) {
                if (!used.add(k.verts[i])) {
                    return false;
                }
            }
            for (int i = 0; i + 3 < k.path.length; i += 2) {
                segs.add(new double[]{k.path[i], k.path[i + 1], k.path[i + 2], k.path[i + 3]});
            }
            if (!collectPlan(k, segs, used)) {
                return false;
            }
        }
        return true;
    }

    private Forest.Node materialize(Forest f, Plan p, int[] ids) {
        Forest.Node n = p.unit >= 0 ? f.byId(ids[p.unit]) : f.add(Forest.JUNC, g.x[p.vertex], g.y[p.vertex]);
        for (Plan k : p.kids) {
            Forest.Node kn = materialize(f, k, ids);
            f.attachToNode(kn, n, k.path, Geo.length(k.path), k.extra);
        }
        return n;
    }

    // ------------------------------------------------------------------ подключение поддерева

    private static final class Cand {
        int kind;
        int node;
        int seg;
        double qx;
        double qy;
        int vg;
        double dist;
        double leg;
        Taps.Option opt;
        double approx;
    }

    private static final class TreeSeg {
        double ax;
        double ay;
        double bx;
        double by;
        int node;
        int seg;
        boolean attachable;
    }

    private Forest cf;
    private Forest.Node cx;
    private double cF;
    private double cPrice;
    private int cDn;
    private final List<TreeSeg> treeSegs = new ArrayList<>();
    private final List<Forest.Node> openNodes = new ArrayList<>();
    private EdgeGrid treeGrid;
    private final List<Integer> blockedList = new ArrayList<>();
    private boolean[] inX = new boolean[64];
    private final Map<Integer, Double> upCache = new HashMap<>();
    private final Map<Long, Double> siteCache = new HashMap<>();
    private List<ExistingNet.Tap> baseTaps;
    private List<Forest.Node> baseTapNodes;
    private double baseRecon;
    /** Цена ещё одной врезки в единицах цели. */
    private double tapFee;
    private final Map<Long, Cand> best = new HashMap<>();
    private final double[] topVals = new double[TOP_K];
    private int topCount;
    /** Сколько лучших кандидатов держит поиск: TOP_K для точной проверки, 1 для оценки группы. */
    private int topK = TOP_K;

    /**
     * Лучшее подключение отсоединённого поддерева x к лесу f или null. bound — приращение цели, выше
     * которого кандидаты не нужны; baseObj — цель леса без поддерева.
     */
    private Forest attachBest(Forest f, Forest.Node x, double bound, double baseObj) {
        if (!(bound > 0)) {
            return null;
        }
        List<Cand> cands = candidates(f, x, bound);
        Forest bestF = null;
        double bestObj = Double.POSITIVE_INFINITY;
        for (Cand c : cands) {
            Forest t = apply(f, x, c);
            if (t == null) {
                continue;
            }
            double obj = model.evaluate(t).objective;
            if (obj < bestObj) {
                bestObj = obj;
                bestF = t;
            }
        }
        return bestF;
    }

    private List<Cand> candidates(Forest f, Forest.Node x, double bound) {
        context(f, x);
        setFlow(model.flowOf(x));
        block(f, x);
        search.filter(this::legAllowed);
        startSearch(x);
        double limit = bound;
        while (true) {
            int v = search.next();
            if (v < 0) {
                break;
            }
            double d = search.dist[v];
            double stop = Math.min(limit, topCount == topK ? topVals[topK - 1] : Double.POSITIVE_INFINITY);
            if (d * cPrice >= stop) {
                break;
            }
            expand(v, d, stop);
        }
        unblock();
        search.filter(null);
        List<Cand> out = new ArrayList<>(best.values());
        out.removeIf(c -> !(c.approx < bound));
        out.sort((a, b) -> Double.compare(a.approx, b.approx));
        return out.size() > topK ? new ArrayList<>(out.subList(0, topK)) : out;
    }

    /** Лес, к которому подключается поддерево x (x == null — отсоединённых поддеревьев нет). */
    private void context(Forest f, Forest.Node x) {
        cf = f;
        cx = x;
        Model.Eval base = model.evaluate(f);
        baseTaps = model.currentTaps();
        baseTapNodes = model.currentTapNodes();
        baseRecon = base.recon + base.reconChambers;
        tapFee = prices.tap + (base.tieIns >= 1 ? model.extraTapWeight : 0);
        collectTree(f, x);
    }

    /** Расход подключаемого поддерева: DN новой трубы и кэши приращений зависят от него. */
    private void setFlow(double flow) {
        cF = flow;
        cDn = prices.dnFor(cF);
        cPrice = prices.perM(cDn);
        upCache.clear();
        siteCache.clear();
        best.clear();
        topCount = 0;
    }

    private void collectTree(Forest f, Forest.Node x) {
        treeSegs.clear();
        openNodes.clear();
        List<Forest.Node> sub = new ArrayList<>();
        if (x != null) {
            Forest.subtree(x, sub);
        }
        int maxId = 0;
        for (Forest.Node n : f.nodes) {
            maxId = Math.max(maxId, n.id);
        }
        if (inX.length <= maxId) {
            inX = new boolean[maxId * 2 + 1];
        } else {
            java.util.Arrays.fill(inX, false);
        }
        for (Forest.Node n : sub) {
            inX[n.id] = true;
        }
        int count = 0;
        for (Forest.Node n : f.nodes) {
            if (n.path == null) {
                continue;
            }
            count += n.path.length / 2 - 1;
        }
        double[] xy = new double[Math.max(1, count) * 4];
        int[] owner = new int[Math.max(1, count)];
        int k = 0;
        for (Forest.Node n : f.nodes) {
            if (!inX[n.id] && attached(n) && (n.type == Forest.JUNC || n.type == Forest.TAP)) {
                openNodes.add(n);
            }
            if (n.path == null) {
                continue;
            }
            boolean attachable = !inX[n.id] && attached(n);
            for (int i = 0; i + 3 < n.path.length; i += 2) {
                TreeSeg s = new TreeSeg();
                s.ax = n.path[i];
                s.ay = n.path[i + 1];
                s.bx = n.path[i + 2];
                s.by = n.path[i + 3];
                s.node = n.id;
                s.seg = i / 2;
                s.attachable = attachable && !(n.type == Forest.LEAF && i == 0);
                treeSegs.add(s);
                xy[k * 4] = s.ax;
                xy[k * 4 + 1] = s.ay;
                xy[k * 4 + 2] = s.bx;
                xy[k * 4 + 3] = s.by;
                owner[k] = n.id;
                k++;
            }
        }
        treeGrid = new EdgeGrid(xy, owner, k, 15.0);
    }

    private static boolean attached(Forest.Node n) {
        Forest.Node u = n;
        while (u.parent != null) {
            u = u.parent;
        }
        return u.type == Forest.TAP;
    }

    private void block(Forest f, Forest.Node x) {
        blockedList.clear();
        for (Forest.Node n : f.nodes) {
            if (n.path == null) {
                continue;
            }
            for (int i = 0; i < n.path.length; i += 2) {
                int v = g.nodeAt(n.path[i], n.path[i + 1]);
                if (v >= 0 && !search.blocked[v]) {
                    search.blocked[v] = true;
                    blockedList.add(v);
                }
            }
        }
        for (int i = 0; i < ports.count; i++) {
            int v = g.portBase + i;
            boolean own = x != null && x.type == Forest.LEAF && ports.owner[i] == x.term;
            if (!own && !search.blocked[v]) {
                search.blocked[v] = true;
                blockedList.add(v);
            }
        }
        if (x == null) {
            return;
        }
        if (x.type == Forest.LEAF) {
            Ports.Terminal t = ports.terms.get(x.term);
            for (int i = 0; i < t.count; i++) {
                int v = g.portBase + t.first + i;
                if (search.blocked[v]) {
                    search.blocked[v] = false;
                }
            }
        } else {
            int v = g.nodeAt(x.x, x.y);
            if (v >= 0) {
                search.blocked[v] = false;
            }
        }
    }

    private void unblock() {
        for (int v : blockedList) {
            search.blocked[v] = false;
        }
        blockedList.clear();
    }

    private void startSearch(Forest.Node x) {
        if (x.type == Forest.LEAF) {
            Ports.Terminal t = ports.terms.get(x.term);
            int[] nodes = new int[t.count];
            double[] init = new double[t.count];
            for (int i = 0; i < t.count; i++) {
                nodes[i] = g.portBase + t.first + i;
                init[i] = ports.stub[t.first + i];
            }
            search.startNodes(nodes, init, t.count);
            return;
        }
        int v = g.nodeAt(x.x, x.y);
        if (v >= 0) {
            search.startNodes(new int[]{v}, new double[]{0}, 1);
        } else {
            search.start(x.x, x.y, false, 0);
        }
    }

    /** Отрезок не пересекает ни одного ребра дерева (касание в конце допустимо). */
    private boolean legAllowed(double ax, double ay, double bx, double by) {
        crossHit.set(ax, ay, bx, by);
        return !treeGrid.walk(ax, ay, bx, by, crossHit);
    }

    private final CrossVisitor crossHit = new CrossVisitor();

    private final class CrossVisitor implements EdgeGrid.Visitor {
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
            double[] e = treeGrid.xy;
            int o = edge * 4;
            return Geo.properCross(ax, ay, bx, by, e[o], e[o + 1], e[o + 2], e[o + 3]);
        }
    }

    private void expand(int v, double d, double stop) {
        double vx = search.x(v);
        double vy = search.y(v);
        for (TreeSeg s : treeSegs) {
            if (!s.attachable) {
                continue;
            }
            double t = Geo.footParam(s.ax, s.ay, s.bx, s.by, vx, vy);
            double qx = s.ax + t * (s.bx - s.ax);
            double qy = s.ay + t * (s.by - s.ay);
            double direct = Geo.dist(vx, vy, qx, qy);
            if ((d + direct) * cPrice >= stop) {
                continue;
            }
            Forest.Node c = cf.byId(s.node);
            int kind = EDGE;
            int target = c.id;
            if (t <= 1e-9 && s.seg == 0) {
                continue;
            }
            if (t >= 1 - 1e-9 && s.seg == c.path.length / 2 - 2) {
                continue;
            }
            double approx = edgeDelta(c, s, qx, qy);
            if ((d + direct) * cPrice + approx >= stop) {
                continue;
            }
            if (!space.nodeFree(qx, qy)) {
                continue;
            }
            double leg = legCost(vx, vy, qx, qy, false);
            if (Double.isNaN(leg)) {
                continue;
            }
            offer(kind, target, s.seg, qx, qy, v, d, leg, null, (d + leg) * cPrice + approx);
        }
        for (Forest.Node n : openNodes) {
            int cap = n.type == Forest.JUNC ? 3 : taps.capacity(n.chamber);
            if (n.kids.size() >= cap) {
                continue;
            }
            double direct = Geo.dist(vx, vy, n.x, n.y);
            if ((d + direct) * cPrice >= stop) {
                continue;
            }
            double approx = n.type == Forest.JUNC ? nodeDelta(n) : tapFee + rootDelta(n);
            if ((d + direct) * cPrice + approx >= stop) {
                continue;
            }
            double leg = n.type == Forest.TAP ? taps.legTo(vx, vy, n.x, n.y) : legCost(vx, vy, n.x, n.y, false);
            if (Double.isNaN(leg) || !legAllowed(vx, vy, n.x, n.y)) {
                continue;
            }
            offer(NODE, n.id, -1, n.x, n.y, v, d, leg, null, (d + leg) * cPrice + approx);
        }
        Taps.Option[] opts = v == search.source ? taps.fromPoint(vx, vy) : taps.from(v);
        for (Taps.Option o : opts) {
            if ((d + o.leg) * cPrice >= stop) {
                continue;
            }
            Forest.Node used = siteNode(o);
            if (used != null) {
                continue;
            }
            double approx = tapFee + newSiteDelta(o);
            double total = (d + o.leg) * cPrice + approx;
            if (total >= stop || !legAllowed(vx, vy, o.x, o.y)) {
                continue;
            }
            offer(TAPC, -1, -1, o.x, o.y, v, d, o.leg, o, total);
        }
    }

    private Forest.Node siteNode(Taps.Option o) {
        for (Forest.Node n : openNodes) {
            if (n.tapSite(o.chamber, o.seg, o.at)) {
                return n;
            }
        }
        return null;
    }

    private double legCost(double ax, double ay, double bx, double by, boolean tapEnd) {
        if (!space.segmentFree(ax, ay, bx, by) || !legAllowed(ax, ay, bx, by)) {
            return Double.NaN;
        }
        double extra = space.specialExtra(ax, ay, bx, by, false, tapEnd);
        if (Double.isNaN(extra)) {
            return Double.NaN;
        }
        return Geo.dist(ax, ay, bx, by) + extra;
    }

    private void offer(int kind, int node, int seg, double qx, double qy, int vg, double dist, double leg,
                       Taps.Option opt, double approx) {
        long key = kind == TAPC ? siteKey(opt) : ((long) kind << 40) ^ node;
        Cand c = best.get(key);
        if (c != null && c.approx <= approx) {
            return;
        }
        if (c == null) {
            c = new Cand();
            best.put(key, c);
        }
        c.kind = kind;
        c.node = node;
        c.seg = seg;
        c.qx = qx;
        c.qy = qy;
        c.vg = vg;
        c.dist = dist;
        c.leg = leg;
        c.opt = opt;
        c.approx = approx;
        insertTop(approx);
    }

    private void insertTop(double v) {
        if (topCount < topK) {
            topVals[topCount++] = v;
        } else if (v < topVals[topK - 1]) {
            topVals[topK - 1] = v;
        } else {
            return;
        }
        for (int i = topCount - 1; i > 0 && topVals[i] < topVals[i - 1]; i--) {
            double t = topVals[i];
            topVals[i] = topVals[i - 1];
            topVals[i - 1] = t;
        }
    }

    private static long siteKey(Taps.Option o) {
        if (o.chamber >= 0) {
            return (3L << 40) ^ o.chamber;
        }
        return (4L << 40) ^ ((long) o.seg << 24) ^ Math.round(o.at * 2);
    }

    // ------------------------------------------------------------------ оценки приращения снизу

    /** Врезка поддерева в ребро c в точке q: новая камера, рост расхода выше q. */
    private double edgeDelta(Forest.Node c, TreeSeg s, double qx, double qy) {
        int nd = prices.dnFor(c.flow + cF);
        double upper = Geo.dist(qx, qy, s.bx, s.by);
        for (int i = (s.seg + 1) * 2; i + 3 < c.path.length; i += 2) {
            upper += Geo.dist(c.path[i], c.path[i + 1], c.path[i + 2], c.path[i + 3]);
        }
        double d = 0;
        if (nd > c.dn) {
            d += upper * (prices.perM(nd) - prices.perM(c.dn));
        }
        d += prices.chamber(Math.max(nd, c.dn));
        return d + up(c.parent);
    }

    /** Подключение к существующей камере разветвления n. */
    private double nodeDelta(Forest.Node n) {
        return up(n);
    }

    /** Рост расхода на cF от узла u до корня: DN рёбер, камеры, реконструкция. */
    private double up(Forest.Node u) {
        Double cached = upCache.get(u.id);
        if (cached != null) {
            return cached;
        }
        double d = 0;
        Forest.Node w = u;
        while (w != null && w.type != Forest.TAP) {
            int nd = prices.dnFor(w.flow + cF);
            if (nd > w.dn) {
                d += (w.len + w.extra) * (prices.perM(nd) - prices.perM(w.dn));
            }
            if (w.type == Forest.JUNC && w.kids.size() >= 2) {
                int max = w.dn;
                for (Forest.Node k : w.kids) {
                    max = Math.max(max, k.dn);
                }
                d += prices.chamber(Math.max(max, nd)) - prices.chamber(max);
            }
            w = w.parent;
        }
        if (w != null) {
            d += rootDelta(w);
        }
        upCache.put(u.id, d);
        return d;
    }

    /** Дополнительный расход cF через существующий корень t: реконструкция и камера врезки. */
    private double rootDelta(Forest.Node t) {
        long key = (5L << 40) ^ t.id;
        Double cached = siteCache.get(key);
        if (cached != null) {
            return cached;
        }
        int idx = baseTapNodes.indexOf(t);
        double d = 0;
        if (idx >= 0) {
            ExistingNet.Tap tap = baseTaps.get(idx);
            double old = tap.flow;
            int oldDn = tap.newDn;
            tap.flow = old + cF;
            tap.newDn = Math.max(oldDn, prices.dnFor(cF));
            ExistingNet.Recon r = model.net.recon(baseTaps, prices, false);
            d = (r.cost + r.chamberCost - baseRecon) * (1 + model.reconWeight);
            tap.flow = old;
            tap.newDn = oldDn;
            if (t.chamber < 0) {
                int segDn = model.net.segs.get(t.seg).dn;
                d += Math.max(0, prices.chamber(Math.max(segDn, prices.dnFor(t.flow + cF)))
                        - prices.chamber(Math.max(segDn, t.dn)));
            }
        }
        siteCache.put(key, d);
        return d;
    }

    /** Новое место врезки: реконструкция и новая камера на участке. */
    private double newSiteDelta(Taps.Option o) {
        long key = siteKey(o);
        Double cached = siteCache.get(key);
        if (cached != null) {
            return cached;
        }
        ExistingNet.Tap tap = new ExistingNet.Tap(o.seg, o.at, o.chamber, cF, cDn);
        baseTaps.add(tap);
        ExistingNet.Recon r = model.net.recon(baseTaps, prices, false);
        baseTaps.remove(baseTaps.size() - 1);
        double d = (r.cost + r.chamberCost - baseRecon) * (1 + model.reconWeight);
        if (o.chamber < 0) {
            d += prices.chamber(Math.max(model.net.segs.get(o.seg).dn, cDn));
        }
        siteCache.put(key, d);
        return d;
    }

    // ------------------------------------------------------------------ применение кандидата

    private Forest apply(Forest f, Forest.Node x, Cand c) {
        double[] path = pathOf(x, c);
        if (path == null) {
            return null;
        }
        boolean tapEnd = c.kind == TAPC || (c.kind == NODE && f.byId(c.node).type == Forest.TAP);
        double len = Geo.length(path);
        double extra = legs.extra(path, tapEnd);
        if (Double.isNaN(extra)) {
            return null;
        }
        Forest t = f.copy();
        Forest.Node tx = t.byId(x.id);
        if (c.kind == EDGE) {
            Forest.Node tc = t.byId(c.node);
            Forest.Node j = t.attachToEdge(tx, tc, c.seg, c.qx, c.qy, path, len, extra);
            if (Double.isNaN(j.extra) || Double.isNaN(tc.extra)) {
                return null;
            }
        } else if (c.kind == NODE) {
            t.attachToNode(tx, t.byId(c.node), path, len, extra);
        } else {
            Taps.Option o = c.opt;
            Forest.Node root = t.tapNode(o.chamber, o.seg, o.at, o.x, o.y);
            t.attachToNode(tx, root, path, len, extra);
        }
        return t;
    }

    private double[] pathOf(Forest.Node x, Cand c) {
        double[] core = search.path(c.vg);
        double[] out = new double[core.length + 4];
        int n = 0;
        if (x.type == Forest.LEAF) {
            out[n++] = x.x;
            out[n++] = x.y;
        }
        for (int i = 0; i < core.length; i += 2) {
            n = push(out, n, core[i], core[i + 1]);
        }
        n = push(out, n, c.qx, c.qy);
        if (n < 4) {
            return null;
        }
        return java.util.Arrays.copyOf(out, n);
    }

    private static int push(double[] out, int n, double x, double y) {
        if (n >= 2 && Math.abs(out[n - 2] - x) < 1e-9 && Math.abs(out[n - 1] - y) < 1e-9) {
            return n;
        }
        out[n++] = x;
        out[n++] = y;
        return n;
    }
}
