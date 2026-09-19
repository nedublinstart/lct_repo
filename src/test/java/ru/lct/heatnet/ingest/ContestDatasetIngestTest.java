package ru.lct.heatnet.ingest;

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
import ru.lct.heatnet.persist.IngestedFeature;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SceneAssembler;
import ru.lct.heatnet.service.DatasetService;

class ContestDatasetIngestTest {

    @Test
    void contestFileMapsOfficialObjectTypesAndInfersTopology() throws Exception {
        Path file = DatasetService.findContestGeoJson();
        assumeTrue(file != null && Files.isRegularFile(file), "конкурсный GeoJSON лежит в корне ветки");
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        AppendixModel appendix = new AppendixLoader(props).load();
        List<IngestedFeature> all = new ArrayList<>();
        GeoJsonStreamingIngestor.ParseStats stats = new GeoJsonStreamingIngestor()
                .parse(file, UUID.randomUUID(), appendix, all::addAll);
        assertThat(stats.total).isEqualTo(144);
        assertThat(count(all, FeatureKind.CONNECTION_POINT)).isEqualTo(17);
        assertThat(count(all, FeatureKind.EXISTING_SEGMENT)).isEqualTo(29);
        assertThat(count(all, FeatureKind.CHAMBER)).isEqualTo(9);
        assertThat(count(all, FeatureKind.SOURCE)).isEqualTo(1);
        assertThat(count(all, FeatureKind.CONSTRAINT)).isEqualTo(88);

        Scene scene = new SceneAssembler().assemble(all, appendix);
        assertThat(scene.oks).hasSize(17);
        assertThat(scene.segments).hasSize(29);
        assertThat(scene.projector.isGeographic()).isTrue();
        assertThat(scene.segments.stream().allMatch(s -> s.nextId != null && !s.nextId.isBlank())).isTrue();
        double flow = scene.oks.stream().mapToDouble(o -> o.flowTph).sum();
        assertThat(flow).isEqualTo(488.72);
    }

    private static long count(List<IngestedFeature> all, FeatureKind kind) {
        return all.stream().filter(f -> f.getKind() == kind).count();
    }
}
