package ru.lct.heatnet.costing;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.engine.NewSegment;

class DiameterSelectorTest {

    private final DiameterSelector selector = new DiameterSelector();

    @Test
    void picksMinimalDiameterThatFitsFlow() {
        AppendixModel appendix = load();
        assertThat(selector.select(10, appendix)).isEqualTo(80);
        assertThat(selector.select(12, appendix)).isEqualTo(80);
        assertThat(selector.select(13, appendix)).isEqualTo(100);
        assertThat(selector.select(500, appendix)).isEqualTo(500);
    }

    @Test
    void upgradesDiameterWhenRunIsTooLong() {
        AppendixModel appendix = load();
        NewSegment seg = new NewSegment();
        seg.flowTph = 10;
        seg.lengthM = 10_000;
        selector.apply(List.of(seg), appendix);
        assertThat(seg.dn).isGreaterThan(80);
    }

    @Test
    void applyDoesNotMutateAppendixOrderPermanentlyAcrossCalls() {
        AppendixModel appendix = load();
        List<NewSegment> segs = new ArrayList<>();
        NewSegment a = new NewSegment();
        a.flowTph = 50;
        a.lengthM = 10;
        segs.add(a);
        selector.apply(segs, appendix);
        assertThat(a.dn).isEqualTo(150);
    }

    private static AppendixModel load() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
