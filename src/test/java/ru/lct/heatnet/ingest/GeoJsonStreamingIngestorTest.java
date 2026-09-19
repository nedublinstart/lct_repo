package ru.lct.heatnet.ingest;

import static org.assertj.core.api.Assertions.assertThat;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import org.junit.jupiter.api.Test;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.persist.IngestedFeature;

class GeoJsonStreamingIngestorTest {

    @Test
    void parsesMiniSampleWithoutLoadingWholeTreeTwice() throws Exception {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        AppendixModel appendix = new AppendixLoader(props).load();
        List<IngestedFeature> all = new ArrayList<>();
        GeoJsonStreamingIngestor.ParseStats stats = new GeoJsonStreamingIngestor()
                .parse(Path.of("samples/mini-input.geojson"), UUID.randomUUID(), appendix, all::addAll);
        assertThat(stats.total).isEqualTo(13);
        assertThat(all.stream().filter(f -> f.getKind() == FeatureKind.OKS_PROSPECTIVE)).hasSize(2);
        assertThat(all.stream().filter(f -> f.getKind() == FeatureKind.EXISTING_SEGMENT)).hasSize(3);
        assertThat(all.stream().filter(f -> f.getKind() == FeatureKind.CONNECTION_POINT)).hasSize(2);
    }

    @Test
    void propertyReaderSeesNumericAliases() {
        ObjectNode n = new ObjectMapper().createObjectNode();
        n.put("raschetnyy_rashod", "12,5");
        PropertyReader reader = new PropertyReader(n);
        assertThat(reader.num("flow", "raschetnyy_rashod")).isEqualTo(12.5);
    }
}
