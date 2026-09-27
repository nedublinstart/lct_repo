package ru.lct.heatnet.engine.flow;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.locationtech.jts.geom.Envelope;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.context.annotation.Primary;
import org.springframework.stereotype.Component;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.ProgressListener;
import ru.lct.heatnet.engine.RoutingEngine;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.engine.steiner.Strategy;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;

/**
 * Трассировка леса минимальной стоимости по приложению.
 * <ol>
 * <li>Свободное пространство: запреты раздуты на минимальные расстояния таблицы 5.1 плюс полугабарит
 * расчётной трубы; спецобъекты пересекаются с надбавкой (Kспец − 1) · длина спецучастка.</li>
 * <li>Выходы ИТП — перпендикуляр к одной из ближайших стен своего корпуса.</li>
 * <li>Сокращённый граф видимости (битангенты углов) в CSR.</li>
 * <li>Лес: последовательная вставка ОКС, переподвешивание поддеревьев с отсечением по оценке снизу,
 * перестройка группы до 8 ОКС динамикой по подмножествам и итерированный локальный поиск.</li>
 * <li>Оценка каждого кандидата точная: DN по расходу и предельной длине, камеры, врезки в существующие
 * камеры и штрафы. Реконструкция существующей сети не входит в смету.</li>
 * </ol>
 * Режимы отличаются только целью. «Минимум врезок» добавляет вес каждой врезке сверх первой.
 * «Минимальная длина» ищет наименьшую новую сеть: метр в цели стоит 100 млн ₽.
 * В смете у всех режимов полная стоимость по приложению.
 */
@Component
@Primary
public class FlowRoutingEngine implements RoutingEngine {

    private static final Logger log = LoggerFactory.getLogger(FlowRoutingEngine.class);
    /** Вес каждой врезки сверх первой в режиме «минимум врезок», ₽. */
    static final double EXTRA_TAP_WEIGHT = 50_000_000;
    /** Рубли за метр в цели «минимальная длина»: метр важнее разницы камер, смета остаётся полной. */
    static final double MIN_LENGTH_PER_M = 100_000_000;
    private static final double ROI_MARGIN_M = 100;

    private final long budgetMs;

    public FlowRoutingEngine() {
        this(Long.getLong("heatnet.flow.budget-ms", 12_000L));
    }

    /** budgetMs — время поиска на все выбранные режимы вместе. */
    public FlowRoutingEngine(long budgetMs) {
        this.budgetMs = Math.max(300, budgetMs);
    }

    @Override
    public List<Variant> route(Scene scene, AppendixModel appendix, CalculationMode mode, ProgressListener progress) {
        return route(scene, appendix, mode, progress, null);
    }

    @Override
    public List<Variant> route(Scene scene, AppendixModel appendix, CalculationMode mode, ProgressListener progress,
                               List<String> strategyCodes) {
        List<Strategy> selected = Strategy.select(strategyCodes);
        long t0 = System.nanoTime();
        progress.progress(15, "Таблицы диаметров и цен");
        Prices prices = new Prices(appendix);
        double total = 0;
        for (ProspectiveOks o : scene.oks) {
            if (o.connection != null) {
                total += Math.max(0, o.flowTph);
            }
        }
        int designDn = prices.dnFor(total);
        Envelope roi = new Envelope(scene.envelope());
        roi.expandBy(ROI_MARGIN_M);

        progress.progress(20, "Зоны минимальных расстояний");
        FreeSpace space = new FreeSpace(scene, appendix, prices, roi, designDn);
        Ports ports = new Ports(scene, space);
        long t1 = System.nanoTime();
        progress.progress(28, "Граф видимости");
        VisGraph g = VisGraph.build(space, ports.px, ports.py, ports.owner);
        ExistingNet net = new ExistingNet(scene);
        Taps taps = new Taps(net, space, g);
        Model model = new Model(prices, net, ports);
        long t2 = System.nanoTime();
        log.info("Граф видимости: вершин {}, рёбер {}, выходов ИТП {}, зоны {} мс, граф {} мс",
                g.n, g.edgeCount(), ports.count, (t1 - t0) / 1_000_000, (t2 - t1) / 1_000_000);

        double weights = 0;
        for (Strategy s : selected) {
            weights += weight(s);
        }
        // Четверть бюджета — ещё один проход по S от уже найденных лесов.
        long totalNs = budgetMs * 1_000_000L;
        long finaleNs = totalNs / 4;
        long bodyNs = totalNs - finaleNs;
        List<Variant> out = new ArrayList<>();
        List<Forest> found = new ArrayList<>();
        List<Model.Eval> evals = new ArrayList<>();
        for (int i = 0; i < selected.size(); i++) {
            Strategy s = selected.get(i);
            progress.progress(35 + (45 * i) / Math.max(1, selected.size()), s.title);
            model.extraTapWeight = s == Strategy.MIN_TAPS ? EXTRA_TAP_WEIGHT : 0;
            model.lengthPrice = Model.SCORE_LENGTH_RUB_PER_M;
            model.lengthFirst = s == Strategy.MIN_RECON;
            model.lengthBoost = s == Strategy.MIN_RECON ? MIN_LENGTH_PER_M : 0;
            long budget = (long) (bodyNs * weight(s) / weights);
            Optimizer opt = new Optimizer(model, g, taps, ports, space, 7919L * (i + 1));
            Forest best = opt.solve(found, budget);
            Model.Eval e = model.evaluate(best);
            int same = sameAs(evals, e);
            found.add(best);
            if (same >= 0) {
                out.get(same).notes.add("Режим «" + s.title + "» дал тот же лес");
                continue;
            }
            Variant v = new Emitter(model, space).emit(best);
            v.code = s.code;
            v.title = s.title;
            v.description = s.description;
            v.notes.add(0, String.format(Locale.ROOT, "Врезок: %d, камер разветвления: %d, новая сеть %.1f м",
                    e.tieIns, junctions(best), e.length));
            out.add(v);
            evals.add(e);
            log.info("{}: {} ₽ (трубы {}, камеры {}, врезки в камеры {}, штраф {}), L={} м",
                    s.code, Math.round(e.total), Math.round(e.pipes), Math.round(e.chambers), Math.round(e.taps),
                    Math.round(e.penalty), Math.round(e.length));
        }
        if (!found.isEmpty()) {
            progress.progress(88, "Уточнение минимальной стоимости");
            model.extraTapWeight = 0;
            model.lengthPrice = Model.SCORE_LENGTH_RUB_PER_M;
            model.lengthFirst = false;
            model.lengthBoost = 0;
            Optimizer fin = new Optimizer(model, g, taps, ports, space, 7919L * 17);
            Forest refined = fin.solve(found, finaleNs);
            Model.Eval e = model.evaluate(refined);
            if (!closeTo(evals, e)) {
                Variant v = new Emitter(model, space).emit(refined);
                v.code = Strategy.MIN_COST.code;
                v.title = Strategy.MIN_COST.title;
                v.description = Strategy.MIN_COST.description;
                v.notes.add(0, String.format(Locale.ROOT, "Врезок: %d, камер разветвления: %d, новая сеть %.1f м",
                        e.tieIns, junctions(refined), e.length));
                out.add(v);
                log.info("уточнение: {} ₽ (трубы {}, камеры {}, врезки в камеры {}, штраф {}), L={} м",
                        Math.round(e.total), Math.round(e.pipes), Math.round(e.chambers), Math.round(e.taps),
                        Math.round(e.penalty), Math.round(e.length));
            }
        }
        log.info("Трассировка: {} мс, вариантов {}", (System.nanoTime() - t0) / 1_000_000, out.size());
        return out;
    }

    private static double weight(Strategy s) {
        return s == Strategy.MIN_COST ? 2 : 1;
    }

    /** Уточнение не добавляет второй контур, если смета и длина уже есть у другого режима. */
    private static boolean closeTo(List<Model.Eval> evals, Model.Eval e) {
        for (Model.Eval o : evals) {
            if (Math.abs(o.total - e.total) < 1000.0 && o.tieIns == e.tieIns
                    && Math.abs(o.length - e.length) < 1.0 && o.unconnected == e.unconnected) {
                return true;
            }
        }
        return false;
    }

    private static int sameAs(List<Model.Eval> evals, Model.Eval e) {
        for (int i = 0; i < evals.size(); i++) {
            Model.Eval o = evals.get(i);
            if (Math.abs(o.total - e.total) < 1.0 && o.tieIns == e.tieIns && Math.abs(o.length - e.length) < 0.01) {
                return i;
            }
        }
        return -1;
    }

    private static int junctions(Forest f) {
        int n = 0;
        for (Forest.Node u : f.nodes) {
            if (u.type == Forest.JUNC && u.kids.size() >= 2) {
                n++;
            }
        }
        return n;
    }
}
