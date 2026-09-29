package ru.lct.heatnet.engine.flow;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

/**
 * Разъяснения от 29.09.2026: камера внутри дороги и прямой вход в ОКС у угла.
 */
class ClarificationGeometryTest {

    private static final GeometryFactory GF = GeoJsonGeometries.GF;

    @Test
    void chamberInsideRoadExtendsThreeMetersOnlyOnTheExitSide() {
        AppendixModel appendix = appendix();
        Polygon road = GF.createPolygon(new Coordinate[]{
                new Coordinate(10, 0), new Coordinate(30, 0), new Coordinate(30, 20),
                new Coordinate(10, 20), new Coordinate(10, 0)
        });
        Scene scene = new Scene();
        SpatialConstraint constraint = new SpatialConstraint();
        constraint.id = "road-1";
        constraint.type = "road";
        constraint.geometry = road;
        constraint.rule = appendix.constraintRule("road");
        scene.constraints.add(constraint);

        Prices prices = new Prices(appendix);
        Envelope roi = new Envelope(-5, 50, -5, 25);
        FreeSpace space = new FreeSpace(scene, appendix, prices, roi, 200);

        double[] intoChamber = {0, 10, 20, 10};
        double into = specialLength(space, intoChamber, false, true);
        assertThat(into).isCloseTo(13.0, within(0.05));

        double[] through = {0, 10, 40, 10};
        double crossed = specialLength(space, through, false, false);
        assertThat(crossed).isCloseTo(26.0, within(0.05));
    }

    @Test
    void connectionNearCornerUsesTheNearestBoundaryPoint() {
        AppendixModel appendix = appendix();
        Polygon building = GF.createPolygon(new Coordinate[]{
                new Coordinate(0, 0), new Coordinate(10, 0), new Coordinate(10, 10),
                new Coordinate(0, 10), new Coordinate(0, 0)
        });
        Scene scene = new Scene();
        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "A";
        oks.flowTph = 10;
        oks.footprint = building;
        oks.connection = GF.createPoint(new Coordinate(0.2, 0.2));
        scene.oks.add(oks);
        SpatialConstraint constraint = new SpatialConstraint();
        constraint.id = "A-footprint";
        constraint.type = "oks";
        constraint.geometry = building;
        constraint.rule = appendix.constraintRule("oks");
        scene.constraints.add(constraint);

        Prices prices = new Prices(appendix);
        Envelope roi = scene.envelope();
        roi.expandBy(100);
        FreeSpace space = new FreeSpace(scene, appendix, prices, roi, prices.dnFor(10));
        Ports ports = new Ports(scene, space);

        assertThat(ports.terms).hasSize(1);
        Ports.Terminal term = ports.terms.get(0);
        assertThat(term.count).isGreaterThan(0);
        boolean nearBoundary = false;
        for (int i = 0; i < term.count; i++) {
            int p = term.first + i;
            boolean left = ports.px[p] < -1 && Math.abs(ports.py[p] - 0.2) < 0.6;
            boolean bottom = ports.py[p] < -1 && Math.abs(ports.px[p] - 0.2) < 0.6;
            if (left || bottom) {
                nearBoundary = true;
            }
        }
        assertThat(nearBoundary).isTrue();
    }

    private static double specialLength(FreeSpace space, double[] path, boolean tapStart, boolean tapEnd) {
        java.util.List<double[]> spans = space.pathSpans(path, tapStart, tapEnd);
        assertThat(spans).isNotNull();
        double length = 0;
        for (double[] span : spans) {
            length += span[1] - span[0];
        }
        return length;
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
