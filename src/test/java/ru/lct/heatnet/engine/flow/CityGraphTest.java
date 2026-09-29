package ru.lct.heatnet.engine.flow;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Envelope;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.scene.Scene;

/**
 * На городе больше порога полного графа пустырь в несколько километров остаётся связным,
 * а список рёбер не раздувается до всех пар вершин.
 */
class CityGraphTest {

    @Test
    void emptyFieldStaysConnectedAcrossThreeKilometres() {
        int saved = VisGraph.exactVertexLimit;
        VisGraph.exactVertexLimit = 0;
        try {
            AppendixModel appendix = appendix();
            Scene scene = new Scene();
            Envelope roi = new Envelope(-200, 3600, -200, 600);
            FreeSpace space = new FreeSpace(scene, appendix, new Prices(appendix), roi, 150);
            VisGraph graph = VisGraph.build(space, new double[]{0, 3000}, new double[]{0, 0}, new int[]{-1, -1});
            assertThat(graph.n).isGreaterThan(2);
            assertThat(graph.edgeCount()).isGreaterThan(0);
            assertThat(reaches(graph, graph.portBase, graph.portBase + 1)).isTrue();
        } finally {
            VisGraph.exactVertexLimit = saved;
        }
    }

    private static boolean reaches(VisGraph graph, int from, int to) {
        boolean[] seen = new boolean[graph.n];
        int[] queue = new int[graph.n];
        int head = 0;
        int tail = 0;
        queue[tail++] = from;
        seen[from] = true;
        while (head < tail) {
            int v = queue[head++];
            for (int e = graph.off[v]; e < graph.off[v + 1]; e++) {
                int u = graph.adj[e];
                if (!seen[u]) {
                    if (u == to) {
                        return true;
                    }
                    seen[u] = true;
                    queue[tail++] = u;
                }
            }
        }
        return false;
    }

    private static AppendixModel appendix() {
        HeatnetProperties props = new HeatnetProperties();
        props.setAppendixPath("config/appendix.yml");
        return new AppendixLoader(props).load();
    }
}
