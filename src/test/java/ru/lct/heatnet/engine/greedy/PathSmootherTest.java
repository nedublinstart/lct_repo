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
    void twoPointPathIsCopiedNotAliased() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        List<Coordinate> raw = new ArrayList<>();
        raw.add(new Coordinate(0, 0));
        raw.add(new Coordinate(10, 0));
        List<Coordinate> slim = PathSmoother.straighten(raw, obstacles);
        slim.clear();
        assertThat(raw).hasSize(2);
    }

    @Test
    void octantStaircaseBecomesRightAngleL() {
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
        assertThat(slim.size()).isEqualTo(3);
        assertThat(OrthoPaths.rightAngle(slim.get(0), slim.get(1), slim.get(2))).isTrue();
        assertThat(OrthoPaths.nearlyAxis(slim.get(0), slim.get(1))).isTrue();
        assertThat(OrthoPaths.nearlyAxis(slim.get(1), slim.get(2))).isTrue();
        assertThat(OrthoPaths.longOpenDiagonal(slim.get(0), slim.get(slim.size() - 1))
                && slim.size() == 2).isFalse();
    }

    @Test
    void openDiagonalBecomesLNotChord() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        List<Coordinate> raw = List.of(new Coordinate(0, 0), new Coordinate(80, 50));
        List<Coordinate> slim = PathSmoother.straighten(raw, obstacles);
        assertThat(slim.size()).isEqualTo(3);
        assertThat(OrthoPaths.rightAngle(slim.get(0), slim.get(1), slim.get(2))).isTrue();
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
        boolean right = false;
        for (int i = 1; i < slim.size() - 1; i++) {
            if (OrthoPaths.rightAngle(slim.get(i - 1), slim.get(i), slim.get(i + 1))) {
                right = true;
            }
        }
        assertThat(right).isTrue();
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

    @Test
    void refineDropsDetourVertexWhenChordIsShorter() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        List<Coordinate> raw = List.of(
                new Coordinate(0, 0),
                new Coordinate(40, 0),
                new Coordinate(80, 0));
        List<Coordinate> slim = PathSmoother.refine(raw, obstacles);
        assertThat(slim.size()).isEqualTo(2);
        assertThat(slim.get(0).distance(new Coordinate(0, 0))).isLessThan(0.2);
        assertThat(slim.get(1).distance(new Coordinate(80, 0))).isLessThan(0.2);
    }

    @Test
    void refineDropsTwoBumpsWhenAxisChordIsShorter() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        List<Coordinate> raw = List.of(
                new Coordinate(0, 0),
                new Coordinate(30, 20),
                new Coordinate(60, 20),
                new Coordinate(90, 0));
        List<Coordinate> slim = PathSmoother.refine(raw, obstacles);
        assertThat(slim.size()).isEqualTo(2);
        assertThat(slim.get(0).distance(new Coordinate(0, 0))).isLessThan(0.2);
        assertThat(slim.get(1).distance(new Coordinate(90, 0))).isLessThan(0.2);
    }

    @Test
    void refineDropsBumpWhenAxisChordIsShorter() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        List<Coordinate> raw = List.of(
                new Coordinate(0, 4),
                new Coordinate(50, -10),
                new Coordinate(100, 4));
        List<Coordinate> slim = PathSmoother.refine(raw, obstacles);
        assertThat(slim.size()).isEqualTo(2);
        assertThat(slim.get(0).distance(new Coordinate(0, 4))).isLessThan(0.2);
        assertThat(slim.get(1).distance(new Coordinate(100, 4))).isLessThan(0.2);
    }

    @Test
    void refineHugsBuildingInsteadOfCuttingThrough() {
        Scene scene = new Scene();
        Polygon wall = gf.createPolygon(new Coordinate[]{
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
        List<Coordinate> raw = List.of(new Coordinate(40, 4), new Coordinate(40, 80));
        List<Coordinate> slim = PathSmoother.refine(raw, obstacles);
        assertThat(slim.size()).isGreaterThanOrEqualTo(3);
        LineString ls = gf.createLineString(slim.toArray(new Coordinate[0]));
        Polygon core = (Polygon) wall.buffer(-1.0);
        assertThat(core.intersects(ls) && !core.touches(ls)).isFalse();
    }

    @Test
    void dropRedundantRemovesIntermediateWhenChordIsShorter() {
        ObstacleIndex obstacles = ObstacleIndex.build(new Scene());
        List<Coordinate> raw = List.of(
                new Coordinate(0, 0),
                new Coordinate(40, 0),
                new Coordinate(80, 0),
                new Coordinate(120, 0));
        List<Coordinate> slim = PathSmoother.dropRedundant(raw, obstacles);
        assertThat(slim.size()).isEqualTo(2);
        assertThat(slim.get(0).distance(new Coordinate(0, 0))).isLessThan(0.2);
        assertThat(slim.get(1).distance(new Coordinate(120, 0))).isLessThan(0.2);
    }

    @Test
    void emitPolishHugsBuildingWhenPipeTouchesFacade() {
        Scene scene = new Scene();
        Polygon wall = gf.createPolygon(new Coordinate[]{
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
        List<Coordinate> raw = List.of(new Coordinate(20, 12), new Coordinate(90, 12));
        List<Coordinate> slim = PathSmoother.emitPolish(raw, obstacles);
        LineString ls = gf.createLineString(slim.toArray(new Coordinate[0]));
        Polygon core = (Polygon) wall.buffer(-1.0);
        assertThat(core.intersects(ls) && !core.touches(ls)).isFalse();
        assertThat(slim.size()).isGreaterThanOrEqualTo(2);
    }

    @Test
    void skipAheadReplacesThreeSidedDetourWithFacadeRun() {
        Scene scene = new Scene();
        Polygon wall = gf.createPolygon(new Coordinate[]{
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
        List<Coordinate> raw = List.of(
                new Coordinate(20, 5),
                new Coordinate(90, 5),
                new Coordinate(90, 70),
                new Coordinate(20, 70));
        double old = OrthoPaths.length(raw);
        List<Coordinate> slim = PathSmoother.emitPolish(raw, obstacles);
        assertThat(OrthoPaths.length(slim)).isLessThan(old - 40);
        assertThat(slim.get(0).distance(new Coordinate(20, 5))).isLessThan(0.2);
        assertThat(slim.get(slim.size() - 1).distance(new Coordinate(20, 70))).isLessThan(0.2);
        LineString ls = gf.createLineString(slim.toArray(new Coordinate[0]));
        Polygon core = (Polygon) wall.buffer(-1.0);
        assertThat(core.intersects(ls) && !core.touches(ls)).isFalse();
    }

    @Test
    void refineStillDoesNotCutThroughBuildingWhenOnlyConsecutiveDrop() {
        Scene scene = new Scene();
        Polygon wall = gf.createPolygon(new Coordinate[]{
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
        List<Coordinate> raw = List.of(
                new Coordinate(20, 5),
                new Coordinate(90, 5),
                new Coordinate(90, 70),
                new Coordinate(20, 70));
        List<Coordinate> slim = PathSmoother.refine(raw, obstacles);
        LineString ls = gf.createLineString(slim.toArray(new Coordinate[0]));
        Polygon core = (Polygon) wall.buffer(-1.0);
        assertThat(core.intersects(ls) && !core.touches(ls)).isFalse();
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
