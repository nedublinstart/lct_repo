package ru.lct.heatnet.engine.greedy;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SpatialConstraint;

class PathSmootherTest {

    private final GeometryFactory gf = new GeometryFactory();

    @Test
    void octantStaircaseCollapsesToAChord() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        List<Coordinate> stairs = new ArrayList<>();
        Coordinate c = new Coordinate(0, 0);
        stairs.add(new Coordinate(c));
        for (int i = 0; i < 24; i++) {
            if (i % 2 == 0) {
                c = new Coordinate(c.x + 4, c.y);
            } else {
                c = new Coordinate(c.x + 4, c.y + 4);
            }
            stairs.add(c);
        }
        List<Coordinate> slim = PathSmoother.straighten(stairs, obstacles);
        assertThat(slim.size()).isLessThanOrEqualTo(3);
        assertThat(slim.get(0).distance(stairs.get(0))).isLessThan(0.2);
        assertThat(slim.get(slim.size() - 1).distance(stairs.get(stairs.size() - 1))).isLessThan(0.2);
    }

    @Test
    void stairsAroundBuildingKeepTheCornerAndStayOutside() {
        GeometryFactory local = gf;
        Scene scene = new Scene();
        Polygon wall = local.createPolygon(new Coordinate[]{
                new Coordinate(30, 10), new Coordinate(80, 10), new Coordinate(80, 60),
                new Coordinate(30, 60), new Coordinate(30, 10)
        });
        SpatialConstraint c = new SpatialConstraint();
        c.id = "BLD";
        c.type = "oks";
        AppendixModel appendix = appendix();
        c.rule = appendix.constraintRule("oks");
        c.geometry = wall;
        scene.constraints.add(c);
        scene.envelope();
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);

        List<Coordinate> stairs = new ArrayList<>();
        for (int x = 0; x <= 96; x += 4) {
            stairs.add(new Coordinate(x, 4));
        }
        for (int y = 8; y <= 80; y += 4) {
            stairs.add(new Coordinate(96, y));
        }
        List<Coordinate> slim = PathSmoother.straighten(stairs, obstacles);
        assertThat(slim.size()).isLessThan(stairs.size() / 3);
        assertThat(slim.size()).isLessThanOrEqualTo(6);
        LineString ls = local.createLineString(slim.toArray(new Coordinate[0]));
        Polygon core = (Polygon) wall.buffer(-1.0);
        assertThat(core.intersects(ls) && !core.touches(ls)).isFalse();
    }

    @Test
    void stubFromInsideBuildingIsKept() {
        Scene scene = new Scene();
        Polygon wall = gf.createPolygon(new Coordinate[]{
                new Coordinate(0, 0), new Coordinate(40, 0), new Coordinate(40, 40),
                new Coordinate(0, 40), new Coordinate(0, 0)
        });
        SpatialConstraint c = new SpatialConstraint();
        c.id = "BLD";
        c.type = "oks";
        AppendixModel appendix = appendix();
        c.rule = appendix.constraintRule("oks");
        c.geometry = wall;
        scene.constraints.add(c);
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);

        List<Coordinate> raw = new ArrayList<>();
        raw.add(new Coordinate(20, 20));
        raw.add(new Coordinate(20, 43));
        for (int i = 1; i <= 12; i++) {
            raw.add(new Coordinate(20 + i * 4.0, 43 + (i % 2) * 4.0));
        }
        List<Coordinate> slim = PathSmoother.straightenKeepStub(raw, obstacles);
        assertThat(slim.get(0).distance(new Coordinate(20, 20))).isLessThan(0.2);
        assertThat(slim.size()).isGreaterThanOrEqualTo(3);
        assertThat(slim.size()).isLessThan(raw.size());
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
