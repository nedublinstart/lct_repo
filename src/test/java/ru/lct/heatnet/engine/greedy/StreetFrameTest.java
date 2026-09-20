package ru.lct.heatnet.engine.greedy;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.costing.CostCalculator;
import ru.lct.heatnet.costing.DiameterSelector;
import ru.lct.heatnet.costing.RankingCalculator;
import ru.lct.heatnet.costing.ReconstructionCalculator;
import ru.lct.heatnet.engine.NewSegment;
import ru.lct.heatnet.engine.SmartRoutingEngine;
import ru.lct.heatnet.engine.Variant;
import ru.lct.heatnet.ingest.GeoJsonStreamingIngestor;
import ru.lct.heatnet.persist.CalculationMode;
import ru.lct.heatnet.persist.IngestedFeature;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SceneAssembler;
import ru.lct.heatnet.scene.SpatialConstraint;
import ru.lct.heatnet.service.DatasetService;

class StreetFrameTest {

    private final GeometryFactory gf = new GeometryFactory();

    @Test
    void snapPointsSitNearRoadsAndBuildings() {
        AppendixModel appendix = appendix();
        Scene scene = axisStreet(appendix, false);
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);
        StreetFrame frame = StreetFrame.build(obstacles, scene);
        assertThat(frame.nodeCount()).isGreaterThan(20);
        assertThat(frame.edgeCount()).isGreaterThan(20);
        assertThat(frame.axes()).isNotEmpty();

        Polygon road = roadPoly();
        Polygon left = buildingPoly(6, 15, 26, 105);
        int nearRoad = 0;
        int nearHouse = 0;
        for (Coordinate c : frame.nodes()) {
            if (road.distance(gf.createPoint(c)) <= 8) {
                nearRoad++;
            }
            if (left.distance(gf.createPoint(c)) <= 8) {
                nearHouse++;
            }
        }
        assertThat(nearRoad).as("точки привязки у дороги").isGreaterThan(8);
        assertThat(nearHouse).as("точки привязки у корпуса").isGreaterThan(4);
    }

    @Test
    void pathFollowsRoadAxisNotMapNorthOnRotatedStreet() {
        AppendixModel appendix = appendix();
        Scene scene = rotatedStreet(appendix);
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);
        StreetFrame frame = StreetFrame.build(obstacles, scene);
        assertThat(obstacles.special().corridors()).as("повёрнутая проезжая в каркасе").isNotEmpty();
        assertThat(frame.axes()).isNotEmpty();
        Coordinate axis = frame.axes().get(0);
        assertThat(Math.abs(axis.x)).isGreaterThan(0.3);
        assertThat(Math.abs(axis.y)).isGreaterThan(0.3);

        double ang = Math.toRadians(32);
        Coordinate u = new Coordinate(Math.cos(ang), Math.sin(ang));
        Coordinate v = new Coordinate(-u.y, u.x);
        Coordinate start = at(u, v, 24, 40);
        Coordinate goal = at(u, v, -24, 40);
        List<Coordinate> path = frame.find(start, goal);
        assertThat(path).isNotNull();
        assertThat(path.size()).isGreaterThanOrEqualTo(2);
        int aligned = 0;
        int total = 0;
        for (int i = 1; i < path.size(); i++) {
            Coordinate a = path.get(i - 1);
            Coordinate b = path.get(i);
            if (a.distance(b) < 4) {
                continue;
            }
            total++;
            if (frame.headingOk(a, b)) {
                aligned++;
            }
        }
        assertThat(total).isGreaterThan(0);
        assertThat(aligned).isEqualTo(total);
    }

    @Test
    void steinerUsesStreetFrameAndCrossesPerpendicular() {
        AppendixModel appendix = appendix();
        Scene scene = axisStreet(appendix, true);
        Variant first = new SmartRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        }).get(0);
        assertThat(first.unconnectedOks).isEmpty();
        assertThat(first.segments).isNotEmpty();

        Polygon road = roadPoly();
        Coordinate axis = new Coordinate(0, 1);
        double alongBad = 0;
        boolean sawSpecial = false;
        for (NewSegment seg : first.segments) {
            LineString ls = seg.geometryMeters;
            Coordinate[] pts = ls.getCoordinates();
            Coordinate origin = new Coordinate(88, 60);
            for (int i = 1; i < pts.length; i++) {
                double len = pts[i - 1].distance(pts[i]);
                if (len < 12 || pts[i - 1].distance(origin) < 8 || pts[i].distance(origin) < 8) {
                    continue;
                }
                double ang = SpecialLayer.crossingAngleDeg(pts[i - 1], pts[i], axis);
                assertThat(ang <= 14 || ang >= 76)
                        .as("ребро не ∥/⊥ улице %s %s→%s ang=%.1f", seg.id, pts[i - 1], pts[i], ang)
                        .isTrue();
            }
            double hit = SpecialLayer.hitLength(ls, road);
            if ("special".equals(seg.layingMethod)) {
                sawSpecial = true;
                double ang = SpecialLayer.crossingAngleDeg(pts[0], pts[pts.length - 1], axis);
                assertThat(ang).isGreaterThanOrEqualTo(70);
            }
            if (hit >= 8) {
                double ang = SpecialLayer.crossingAngleDeg(pts[0], pts[pts.length - 1], axis);
                if (ang < 70) {
                    alongBad += hit;
                }
            }
        }
        assertThat(sawSpecial).isTrue();
        assertThat(alongBad).isLessThan(8);
    }

    private Scene axisStreet(AppendixModel appendix, boolean explicitRoad) {
        Scene scene = new Scene();
        ExistingSegment seg = new ExistingSegment();
        seg.id = "S-1";
        seg.dn = 400;
        seg.existingFlowTph = 10;
        seg.nextId = "SRC";
        seg.line = gf.createLineString(new Coordinate[]{new Coordinate(0, 0), new Coordinate(0, 120)});
        scene.segments.add(seg);
        scene.constraints.add(building("L", 6, 15, 26, 105, appendix));
        scene.constraints.add(building("R", 78, 15, 98, 105, appendix));
        if (explicitRoad) {
            SpatialConstraint road = new SpatialConstraint();
            road.id = "ROAD";
            road.type = "tdtp";
            road.geometry = roadPoly();
            road.rule = appendix.constraintRule("tdtp");
            scene.constraints.add(road);
        }
        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "OKS-A";
        oks.flowTph = 12;
        oks.connection = gf.createPoint(new Coordinate(88, 60));
        scene.oks.add(oks);
        scene.envelope();
        return scene;
    }

    private Scene rotatedStreet(AppendixModel appendix) {
        double ang = Math.toRadians(32);
        Coordinate u = new Coordinate(Math.cos(ang), Math.sin(ang));
        Coordinate v = new Coordinate(-u.y, u.x);
        Scene scene = new Scene();
        ExistingSegment seg = new ExistingSegment();
        seg.id = "S-1";
        seg.dn = 400;
        seg.existingFlowTph = 10;
        seg.nextId = "SRC";
        Coordinate p0 = at(u, v, -90, 0);
        Coordinate p1 = at(u, v, -90, 80);
        seg.line = gf.createLineString(new Coordinate[]{p0, p1});
        scene.segments.add(seg);
        scene.constraints.add(orientedBuilding("L", u, v, -52, 60, 18, 50, appendix));
        scene.constraints.add(orientedBuilding("R", u, v, 52, 60, 18, 50, appendix));
        SpatialConstraint road = new SpatialConstraint();
        road.id = "ROAD";
        road.type = "tdtp";
        road.geometry = orientedBox(u, v, 0, 40, 18, 140);
        road.rule = appendix.constraintRule("tdtp");
        scene.constraints.add(road);
        ProspectiveOks oks = new ProspectiveOks();
        oks.id = "OKS-A";
        oks.flowTph = 12;
        oks.connection = gf.createPoint(at(u, v, 52, 60));
        scene.oks.add(oks);
        scene.envelope();
        return scene;
    }

    private SpatialConstraint orientedBuilding(String id, Coordinate u, Coordinate v,
                                               double along, double mid, double halfW, double halfL,
                                               AppendixModel appendix) {
        SpatialConstraint c = new SpatialConstraint();
        c.id = id;
        c.type = "oks";
        c.geometry = orientedBox(u, v, along, mid, halfW, halfL);
        c.rule = appendix.constraintRule("oks");
        return c;
    }

    private Polygon orientedBox(Coordinate u, Coordinate v, double along, double mid,
                                double halfW, double halfL) {
        Coordinate c = at(u, v, along, mid);
        Coordinate a = new Coordinate(c.x + halfL * u.x + halfW * v.x, c.y + halfL * u.y + halfW * v.y);
        Coordinate b = new Coordinate(c.x + halfL * u.x - halfW * v.x, c.y + halfL * u.y - halfW * v.y);
        Coordinate d = new Coordinate(c.x - halfL * u.x - halfW * v.x, c.y - halfL * u.y - halfW * v.y);
        Coordinate e = new Coordinate(c.x - halfL * u.x + halfW * v.x, c.y - halfL * u.y + halfW * v.y);
        return gf.createPolygon(new Coordinate[]{a, b, d, e, new Coordinate(a)});
    }

    private static Coordinate at(Coordinate u, Coordinate v, double along, double mid) {
        return new Coordinate(along * v.x + mid * u.x, along * v.y + mid * u.y);
    }

    private SpatialConstraint building(String id, double x0, double y0, double x1, double y1, AppendixModel appendix) {
        SpatialConstraint c = new SpatialConstraint();
        c.id = id;
        c.type = "oks";
        c.geometry = buildingPoly(x0, y0, x1, y1);
        c.rule = appendix.constraintRule("oks");
        return c;
    }

    private Polygon buildingPoly(double x0, double y0, double x1, double y1) {
        return gf.createPolygon(new Coordinate[]{
                new Coordinate(x0, y0), new Coordinate(x1, y0),
                new Coordinate(x1, y1), new Coordinate(x0, y1),
                new Coordinate(x0, y0)
        });
    }

    private Polygon roadPoly() {
        return gf.createPolygon(new Coordinate[]{
                new Coordinate(32, -220), new Coordinate(72, -220),
                new Coordinate(72, 340), new Coordinate(32, 340),
                new Coordinate(32, -220)
        });
    }

    @Test
    void contestFrameIsMostlyOneStreetGraph() throws Exception {
        Path file = DatasetService.findContestGeoJson();
        assumeTrue(file != null && Files.isRegularFile(file), "конкурсный GeoJSON лежит в корне ветки");
        AppendixModel appendix = appendix();
        List<IngestedFeature> all = new ArrayList<>();
        new GeoJsonStreamingIngestor().parse(file, UUID.randomUUID(), appendix, all::addAll);
        Scene scene = new SceneAssembler().assemble(all, appendix);
        ObstacleIndex obstacles = ObstacleIndex.build(scene, appendix);
        StreetFrame frame = StreetFrame.build(obstacles, scene);
        assertThat(obstacles.special().corridors()).isNotEmpty();
        assertThat(frame.nodeCount()).isGreaterThan(200);
        assertThat(frame.edgeCount() * 1.0 / Math.max(1, frame.nodeCount()))
                .as("скелет улиц, не решётка: E/N узлы=%s рёбра=%s", frame.nodeCount(), frame.edgeCount())
                .isLessThan(2.8);
        int reached = 0;
        for (ProspectiveOks o : scene.oks) {
            if (o.connection == null) {
                continue;
            }
            Coordinate origin = o.connection.getCoordinate();
            Coordinate tap = nearestSeg(scene, origin);
            Coordinate at = obstacles.exitToStreet(origin, tap, 2.2);
            if (frame.find(at, tap) != null) {
                reached++;
            }
        }
        assertThat(reached).as("ОКС с путём по каркасу до сети, компонент=%s крупнейшая=%s/%s",
                frame.componentCount(), frame.largestComponentSize(), frame.nodeCount())
                .isGreaterThanOrEqualTo(14);
        List<Variant> variants = new SmartRoutingEngine().route(scene, appendix, CalculationMode.PLAN_2D, (p, m) -> {
        });
        assertThat(variants).isNotEmpty();
        assertThat(variants.get(0).unconnectedOks.size())
                .as("Steiner должен подключить большинство ОКС, unconnected=%s", variants.get(0).unconnectedOks)
                .isLessThanOrEqualTo(4);
        StreetFrame routed = frame;
        java.util.Set<String> oksIds = new java.util.HashSet<>();
        for (ProspectiveOks o : scene.oks) {
            if (o.id != null) {
                oksIds.add(o.id);
            }
        }
        int longEdges = 0;
        int aligned = 0;
        for (NewSegment seg : variants.get(0).segments) {
            Coordinate[] pts = seg.geometryMeters.getCoordinates();
            boolean stub = oksIds.contains(seg.fromId);
            for (int i = 1; i < pts.length; i++) {
                if (pts[i - 1].distance(pts[i]) < 10) {
                    continue;
                }
                if (stub && i == 1) {
                    continue;
                }
                longEdges++;
                if (routed.headingOk(pts[i - 1], pts[i])) {
                    aligned++;
                }
            }
        }
        assertThat(longEdges).isGreaterThan(0);
        assertThat(aligned * 1.0 / longEdges)
                .as("длинные рёбра ∥/⊥ осям улиц: %s/%s", aligned, longEdges)
                .isGreaterThanOrEqualTo(0.75);
        DiameterSelector diameters = new DiameterSelector();
        ReconstructionCalculator reconstruction = new ReconstructionCalculator();
        CostCalculator cost = new CostCalculator();
        for (Variant v : variants) {
            diameters.applyTree(v, appendix);
            reconstruction.apply(v, scene, appendix);
            cost.apply(v, scene, appendix);
        }
        new RankingCalculator().rank(variants, appendix);
        for (Variant v : variants) {
            assertThat(v.unconnectedOks)
                    .as("%s должен подключить все ОКС", v.title)
                    .isEmpty();
        }
        assertThat(variants.get(0).totalCost)
                .as("%s C=%.0f ₽", variants.get(0).title, variants.get(0).totalCost)
                .isLessThan(300_000_000);
        assertThat(variants.get(0).taps.size())
                .as("%s врезок=%s", variants.get(0).title, variants.get(0).taps.size())
                .isLessThanOrEqualTo(3);
    }

    private static Coordinate nearestSeg(Scene scene, Coordinate from) {
        Coordinate best = scene.segments.get(0).line.getCoordinateN(0);
        double bestD = from.distance(best);
        for (ExistingSegment seg : scene.segments) {
            Coordinate[] c = seg.line.getCoordinates();
            for (Coordinate p : c) {
                double d = from.distance(p);
                if (d < bestD) {
                    bestD = d;
                    best = p;
                }
            }
        }
        return best;
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
