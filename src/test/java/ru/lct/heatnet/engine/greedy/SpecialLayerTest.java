package ru.lct.heatnet.engine.greedy;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.scene.SpatialConstraint;

class SpecialLayerTest {

    private final GeometryFactory gf = new GeometryFactory();

    @Test
    void perpendicularRoadCrossingIsAllowedAndCostsKSpec() {
        SpecialLayer layer = roadLayer();
        Coordinate a = new Coordinate(-6, 50);
        Coordinate b = new Coordinate(26, 50);
        SpecialLayer.Travel t = layer.inspect(a, b);
        assertThat(t.allowed).isTrue();
        assertThat(t.special).isTrue();
        assertThat(t.crossingAngleDeg).isGreaterThan(80);
        assertThat(t.cost).isGreaterThan(a.distance(b));
        assertThat(t.kSpec).isCloseTo(1.6, within(0.05));
    }

    @Test
    void longitudinalRunInsideCarriagewayIsForbidden() {
        SpecialLayer layer = roadLayer();
        SpecialLayer.Travel t = layer.inspect(new Coordinate(10, 8), new Coordinate(10, 92));
        assertThat(t.allowed).isFalse();
    }

    @Test
    void shallowDiagonalAcrossStreetIsForbidden() {
        SpecialLayer layer = roadLayer();
        SpecialLayer.Travel t = layer.inspect(new Coordinate(-5, 24), new Coordinate(25, 76));
        assertThat(t.crossingAngleDeg).isLessThan(45);
        assertThat(t.allowed).isFalse();
    }

    @Test
    void crossingSteeperThan45DegreesIsAllowed() {
        SpecialLayer layer = roadLayer();
        SpecialLayer.Travel t = layer.inspect(new Coordinate(-5, 41), new Coordinate(25, 58.3));
        assertThat(t.crossingAngleDeg).isGreaterThan(55);
        assertThat(t.allowed).isTrue();
        assertThat(t.special).isTrue();
    }

    @Test
    void shortGrazeDoesNotForceSpecialMethod() {
        SpecialLayer layer = roadLayer();
        SpecialLayer.Travel t = layer.inspect(new Coordinate(-2, 1.2), new Coordinate(2.5, 1.2));
        assertThat(t.allowed).isTrue();
        assertThat(t.special).isFalse();
    }

    @Test
    void splitKeepsSpecialOnlyOnTheCrossing() {
        SpecialLayer layer = roadLayer();
        List<Coordinate> path = List.of(new Coordinate(-12, 50), new Coordinate(32, 50));
        List<SpecialLayer.Piece> pieces = layer.splitByTransport(path);
        assertThat(pieces.size()).isGreaterThanOrEqualTo(2);
        double special = 0;
        double base = 0;
        boolean sawSpecial = false;
        boolean sawBase = false;
        for (SpecialLayer.Piece p : pieces) {
            if (p.special) {
                sawSpecial = true;
                special += p.lengthM;
                assertThat(p.kSpec).isGreaterThan(1.01);
            } else {
                sawBase = true;
                base += p.lengthM;
            }
        }
        assertThat(sawSpecial).isTrue();
        assertThat(sawBase).isTrue();
        assertThat(special).isLessThan(32);
        assertThat(base).isGreaterThan(4);
        for (SpecialLayer.Piece p : pieces) {
            assertThat(p.coords.size()).as("без 2-метрового измельчения").isLessThanOrEqualTo(4);
        }
    }

    @Test
    void inferredCarriagewayBetweenFacingBlocksForbidsAlongStreetChord() {
        AppendixModel appendix = appendix();
        SpecialLayer layer = new SpecialLayer(appendix);
        layer.inferFromBlocks(List.of(rect(0, 0, 20, 90), rect(44, 0, 64, 90)));
        layer.finish();
        assertThat(layer.isEmpty()).isFalse();
        SpecialLayer.Travel along = layer.inspect(new Coordinate(32, 15), new Coordinate(32, 75));
        assertThat(along.allowed).isFalse();
        SpecialLayer.Travel across = layer.inspect(new Coordinate(18, 45), new Coordinate(46, 45));
        assertThat(across.allowed).isTrue();
        assertThat(across.special).isTrue();
        assertThat(across.crossingAngleDeg).isGreaterThan(45);
    }

    private SpecialLayer roadLayer() {
        AppendixModel appendix = appendix();
        SpecialLayer layer = new SpecialLayer(appendix);
        SpatialConstraint c = new SpatialConstraint();
        c.id = "R";
        c.type = "road";
        c.rule = appendix.constraintRule("road");
        Polygon road = rect(0, 0, 20, 100);
        c.geometry = road;
        layer.addExplicit(road, c);
        layer.finish();
        return layer;
    }

    private Polygon rect(double x0, double y0, double x1, double y1) {
        return gf.createPolygon(new Coordinate[]{
                new Coordinate(x0, y0), new Coordinate(x1, y0),
                new Coordinate(x1, y1), new Coordinate(x0, y1),
                new Coordinate(x0, y0)
        });
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
