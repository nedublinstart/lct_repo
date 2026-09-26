package ru.lct.heatnet.engine.depth;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import org.junit.jupiter.api.Test;
import ru.lct.heatnet.engine.depth.DepthProfile.Band;
import ru.lct.heatnet.engine.depth.DepthProfile.Choice;
import ru.lct.heatnet.engine.depth.DepthProfile.Cover;
import ru.lct.heatnet.engine.depth.DepthProfile.Edge;
import ru.lct.heatnet.engine.depth.DepthProfile.Knot;
import ru.lct.heatnet.engine.depth.DepthProfile.Pin;

class DepthProfileTest {

    @Test
    void shallowerCrossingIsPreferred() {
        Choice choice = DepthProfile.choose(3.0, 0.7, List.of(new Band(2.42, 3.4)), 0.10);
        assertThat(choice.depth).isEqualTo(2.42);
        assertThat(DepthProfile.kGl(choice.depth, 3.0, 0.10)).isEqualTo(1.0);
    }

    @Test
    void tooTallPipeGoesBelow() {
        Choice choice = DepthProfile.choose(3.0, 0.7, List.of(new Band(0.6, 3.4)), 0.10);
        assertThat(choice.depth).isCloseTo(3.4, within(1e-9));
        assertThat(DepthProfile.kGl(3.4, 3.0, 0.10)).isCloseTo(1.04, within(1e-9));
        assertThat(DepthProfile.kGlMean(3.0, 3.4, 3.0, 0.10)).isCloseTo(1.02, within(1e-9));
        assertThat(DepthProfile.kGl(2.5, 3.0, 0.10)).isEqualTo(1.0);
    }

    @Test
    void rampLeavesOrdinaryAtTenMetersPerMeter() {
        Pin pin = pin(0, 50, 50, 2.0, 2.0);
        List<Knot> knots = DepthProfile.solve(List.of(new Edge("A", "B", 100)), List.of(pin), List.of(),
                3.0, 0.7, 0.10, 0.10).get(0);
        assertThat(at(knots, 0)).isCloseTo(3.0, within(1e-6));
        assertThat(at(knots, 40)).isCloseTo(3.0, within(1e-4));
        assertThat(at(knots, 45)).isCloseTo(2.5, within(1e-4));
        assertThat(at(knots, 50)).isCloseTo(2.0, within(1e-6));
        assertThat(at(knots, 60)).isCloseTo(3.0, within(1e-4));
        assertThat(at(knots, 100)).isCloseTo(3.0, within(1e-6));
        assertSlope(knots, 0.1001);
    }

    @Test
    void closeObstaclesDoNotReturnToOrdinary() {
        Pin left = pin(0, 40, 40, 4.0, 4.0);
        Pin right = pin(0, 46, 46, 4.0, 4.0);
        List<Knot> knots = DepthProfile.solve(List.of(new Edge("A", "B", 100)), List.of(left, right), List.of(),
                3.0, 0.7, 0.10, 0.10).get(0);
        assertThat(at(knots, 40)).isCloseTo(4.0, within(1e-4));
        assertThat(at(knots, 43)).isGreaterThan(3.6);
        assertThat(at(knots, 43)).isLessThan(3.85);
        assertThat(at(knots, 0)).isCloseTo(3.0, within(1e-4));
        assertThat(at(knots, 100)).isCloseTo(3.0, within(1e-4));
        assertSlope(knots, 0.1001);
    }

    @Test
    void rampContinuesOntoTheNextPipe() {
        Pin pin = pin(1, 10, 10, 1.0, 1.0);
        List<Edge> edges = List.of(new Edge("U", "V", 10), new Edge("V", "W", 10));
        List<List<Knot>> profiles = DepthProfile.solve(edges, List.of(pin), List.of(), 3.0, 0.7, 0.10, 0.10);
        assertThat(at(profiles.get(0), 0)).isCloseTo(3.0, within(1e-4));
        assertThat(at(profiles.get(0), 10)).isCloseTo(2.0, within(1e-4));
        assertThat(at(profiles.get(1), 0)).isCloseTo(2.0, within(1e-4));
        assertThat(at(profiles.get(1), 10)).isCloseTo(1.0, within(1e-4));
        assertSlope(profiles.get(0), 0.1001);
        assertSlope(profiles.get(1), 0.1001);
    }

    @Test
    void incompatibleDepthsShareTheLowerPassage() {
        Pin left = pin(0, 0, 0, 1.0, 1.0);
        left.bands.add(new Band(1.0, 5.0));
        left.low = 0.7;
        left.high = 1.0;
        Pin right = pin(0, 1, 1, 0.5, 0.5);
        right.bands.add(new Band(0.5, 4.0));
        right.low = 0.7;
        right.high = 0.5;
        List<Knot> knots = DepthProfile.solve(List.of(new Edge("A", "B", 10)), List.of(left, right), List.of(),
                3.0, 0.7, 0.10, 0.10).get(0);
        assertThat(at(knots, 0)).isCloseTo(5.0, within(1e-4));
        assertThat(at(knots, 1)).isCloseTo(5.0, within(1e-4));
        assertSlope(knots, 0.1001);
    }

    @Test
    void roadCoverPushesAShallowPassageDown() {
        Pin pin = pin(0, 50, 50, 1.0, 1.2);
        pin.bands.add(new Band(1.2, 3.5));
        Cover cover = new Cover(0, 52, 52, 2.0);
        List<Knot> knots = DepthProfile.solve(List.of(new Edge("A", "B", 100)), List.of(pin), List.of(cover),
                3.0, 0.7, 0.10, 0.10).get(0);
        assertThat(at(knots, 50)).isCloseTo(3.5, within(1e-3));
        assertThat(at(knots, 52)).isCloseTo(3.3, within(0.05));
        assertSlope(knots, 0.1001);
    }

    private static Pin pin(int edge, double s0, double s1, double depth, double high) {
        Pin pin = new Pin(edge, s0, s1);
        pin.depth = depth;
        pin.low = 0.7;
        pin.high = high;
        pin.floor = 0.7;
        return pin;
    }

    private static double at(List<Knot> knots, double station) {
        if (station <= knots.get(0).station) {
            return knots.get(0).depth;
        }
        for (int i = 1; i < knots.size(); i++) {
            Knot a = knots.get(i - 1);
            Knot b = knots.get(i);
            if (station <= b.station + 1e-9) {
                double span = b.station - a.station;
                if (span < 1e-12) {
                    return b.depth;
                }
                double t = (station - a.station) / span;
                return a.depth + t * (b.depth - a.depth);
            }
        }
        return knots.get(knots.size() - 1).depth;
    }

    private static void assertSlope(List<Knot> knots, double limit) {
        for (int i = 1; i < knots.size(); i++) {
            Knot a = knots.get(i - 1);
            Knot b = knots.get(i);
            double span = b.station - a.station;
            if (span < 1e-9) {
                continue;
            }
            assertThat(Math.abs(b.depth - a.depth) / span).isLessThanOrEqualTo(limit);
        }
    }
}
