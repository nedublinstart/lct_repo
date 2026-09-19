package ru.lct.heatnet.engine.steiner;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.engine.greedy.ObstacleIndex;
import ru.lct.heatnet.scene.ExistingSegment;
import ru.lct.heatnet.scene.HeatSource;
import ru.lct.heatnet.scene.Scene;

class TapCatalogTest {

    private final GeometryFactory gf = new GeometryFactory();

    @Test
    void residualFlowOnThinSpineMakesFatTapCheaperThanReconstruction() {
        AppendixModel appendix = appendix();
        Scene scene = bottleneckScene();
        TapCatalog catalog = TapCatalog.build(scene, appendix, ObstacleIndex.build(scene, appendix));
        TapCandidate south = find(catalog, "SOUTH");
        TapCandidate fat = find(catalog, "FAT");
        assertThat(south).isNotNull();
        assertThat(fat).isNotNull();

        double first = 400;
        assertThat(catalog.reconRubles(south, first, appendix)).isEqualTo(0.0);

        Map<String, Double> extra = new HashMap<>();
        catalog.commit(south, first, extra);
        assertThat(extra.get("THIN")).isEqualTo(400.0);

        double overflow = 90;
        double southMoney = catalog.money(south, overflow, 40, extra, Strategy.MIN_COST, appendix);
        double fatMoney = catalog.money(fat, overflow, 80, extra, Strategy.MIN_COST, appendix);
        assertThat(catalog.reconRubles(south, overflow, extra, appendix)).isGreaterThan(1_000_000);
        assertThat(catalog.reconRubles(fat, overflow, extra, appendix)).isEqualTo(0.0);
        assertThat(fatMoney).isLessThan(southMoney);
        assertThat(catalog.spareOnWalk(south, appendix)).isEqualTo(437.4);
        assertThat(catalog.spareOnWalk(fat, extra, appendix)).isGreaterThan(400);
    }

    private Scene bottleneckScene() {
        Scene scene = new Scene();
        scene.segments.add(seg("SOUTH", 400, new Coordinate(0, -80), new Coordinate(0, 0), "THIN"));
        scene.segments.add(seg("THIN", 300, new Coordinate(0, 0), new Coordinate(0, 160), "FAT"));
        scene.segments.add(seg("FAT", 500, new Coordinate(0, 160), new Coordinate(0, 240), "SRC"));
        HeatSource src = new HeatSource();
        src.id = "SRC";
        src.point = gf.createPoint(new Coordinate(0, 240));
        scene.sources.add(src);
        scene.envelope();
        return scene;
    }

    private ExistingSegment seg(String id, int dn, Coordinate a, Coordinate b, String next) {
        ExistingSegment s = new ExistingSegment();
        s.id = id;
        s.dn = dn;
        s.existingFlowTph = 0;
        s.nextId = next;
        s.line = gf.createLineString(new Coordinate[]{a, b});
        return s;
    }

    private static TapCandidate find(TapCatalog catalog, String existingId) {
        for (TapCandidate t : catalog.all()) {
            if (existingId.equals(t.existingId)) {
                return t;
            }
        }
        return null;
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
