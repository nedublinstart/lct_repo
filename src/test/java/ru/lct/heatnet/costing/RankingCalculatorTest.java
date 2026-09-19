package ru.lct.heatnet.costing;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.engine.Variant;

class RankingCalculatorTest {

    @Test
    void cheaperAndShorterRanksFirst() {
        AppendixModel appendix = new AppendixModel();
        appendix.getRanking().costWeight = 0.7;
        appendix.getRanking().lengthWeight = 0.3;
        appendix.getRanking().lengthToCost = 1000;

        Variant expensive = new Variant();
        expensive.code = "b";
        expensive.totalCost = 1_000_000;
        expensive.newLengthM = 1000;

        Variant cheap = new Variant();
        cheap.code = "a";
        cheap.totalCost = 100_000;
        cheap.newLengthM = 100;

        List<Variant> list = new ArrayList<>();
        list.add(expensive);
        list.add(cheap);
        new RankingCalculator().rank(list, appendix);
        assertThat(list.get(0).code).isEqualTo("a");
        assertThat(list.get(0).score).isLessThan(list.get(1).score);
    }
}
