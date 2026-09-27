package ru.lct.heatnet.engine.flow;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class FreeSpaceSpansTest {

    @Test
    void sameCoefficientDoesNotMergeDifferentRestrictions() {
        List<double[]> raw = new ArrayList<>();
        raw.add(new double[]{0, 10, 1.60, 1});
        raw.add(new double[]{10, 20, 1.60, 2});

        List<double[]> spans = FreeSpace.mergeSpans(raw);

        assertThat(spans).hasSize(2);
        assertThat(spans.get(0)).containsExactly(0, 10, 1.60);
        assertThat(spans.get(1)).containsExactly(10, 20, 1.60);
    }

    @Test
    void sameRestrictionStaysOneSegment() {
        List<double[]> raw = new ArrayList<>();
        raw.add(new double[]{0, 10, 1.05, 4});
        raw.add(new double[]{10, 18, 1.05, 4});

        List<double[]> spans = FreeSpace.mergeSpans(raw);

        assertThat(spans).hasSize(1);
        assertThat(spans.get(0)).containsExactly(0, 18, 1.05);
    }

    @Test
    void overlapTakesTheLargestCoefficientAndSplitsOnTheSet() {
        List<double[]> raw = new ArrayList<>();
        raw.add(new double[]{0, 10, 1.60, 1});
        raw.add(new double[]{6, 14, 1.75, 2});

        List<double[]> spans = FreeSpace.mergeSpans(raw);

        assertThat(spans).hasSize(3);
        assertThat(spans.get(0)).containsExactly(0, 6, 1.60);
        assertThat(spans.get(1)).containsExactly(6, 10, 1.75);
        assertThat(spans.get(2)).containsExactly(10, 14, 1.75);
    }
}
