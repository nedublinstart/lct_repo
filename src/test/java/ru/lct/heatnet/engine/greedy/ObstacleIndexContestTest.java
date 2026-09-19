package ru.lct.heatnet.engine.greedy;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import org.junit.jupiter.api.Test;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.ingest.GeoJsonStreamingIngestor;
import ru.lct.heatnet.persist.IngestedFeature;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SceneAssembler;
import ru.lct.heatnet.service.DatasetService;

class ObstacleIndexContestTest {

    @Test
    void contestObstacleIndexBuildsInSeconds() throws Exception {
        Path file = DatasetService.findContestGeoJson();
        assumeTrue(file != null && Files.isRegularFile(file), "конкурсный GeoJSON лежит в корне ветки");
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        AppendixModel appendix = new AppendixLoader(props).load();
        List<IngestedFeature> all = new ArrayList<>();
        new GeoJsonStreamingIngestor().parse(file, UUID.randomUUID(), appendix, all::addAll);
        Scene scene = new SceneAssembler().assemble(all, appendix);

        long t0 = System.nanoTime();
        ObstacleIndex index = ObstacleIndex.build(scene, appendix);
        long ms = (System.nanoTime() - t0) / 1_000_000L;
        assertThat(index).isNotNull();
        assertThat(index.avoidPolygons()).isNotEmpty();
        assertThat(ms).as("индекс препятствий не должен зависать на union кварталов: %d мс", ms)
                .isLessThan(12_000L);
    }
}
