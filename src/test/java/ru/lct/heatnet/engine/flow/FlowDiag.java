package ru.lct.heatnet.engine.flow;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import org.locationtech.jts.geom.Envelope;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;
import ru.lct.heatnet.ingest.GeoJsonStreamingIngestor;
import ru.lct.heatnet.persist.IngestedFeature;
import ru.lct.heatnet.scene.ProspectiveOks;
import ru.lct.heatnet.scene.Scene;
import ru.lct.heatnet.scene.SceneAssembler;
import ru.lct.heatnet.service.DatasetService;

/** Диагностика: длины путей по графу видимости, MST по метрике графа. */
public final class FlowDiag {

    public static void main(String[] args) throws Exception {
        AppendixModel appendix = new AppendixLoader(new HeatnetProperties()).load();
        List<IngestedFeature> features = new ArrayList<>();
        Path input = DatasetService.findContestGeoJson();
        new GeoJsonStreamingIngestor().parse(input, UUID.randomUUID(), appendix, features::addAll);
        Scene scene = new SceneAssembler().assemble(features, appendix);
        String mode = System.getProperty("heatnet.diag.mode", "");
        if (mode.contains("freeChambers")) {
            for (AppendixModel.ChamberTier tier : appendix.getCosts().newChamberTiers) {
                tier.cost = 0;
            }
        }
        if (mode.contains("noBuildings")) {
            scene.constraints.removeIf(c -> "oks".equals(c.type));
        } else if (mode.contains("noClear")) {
            for (ru.lct.heatnet.scene.SpatialConstraint c : scene.constraints) {
                if ("oks".equals(c.type)) {
                    AppendixModel.ConstraintSpec r = new AppendixModel.ConstraintSpec();
                    r.action = "AVOID";
                    r.bufferM = 0;
                    c.rule = r;
                }
            }
        }
        Prices prices = new Prices(appendix);
        double total = 0;
        for (ProspectiveOks o : scene.oks) {
            total += o.flowTph;
        }
        Envelope roi = new Envelope(scene.envelope());
        roi.expandBy(100);
        int designDn = mode.contains("smallClear") ? prices.dn[0] : prices.dnFor(total);
        FreeSpace space = new FreeSpace(scene, appendix, prices, roi, designDn);
        if (mode.contains("checkViolation")) {
            java.util.Random r = new java.util.Random(5);
            double worst = 0;
            int n = 0;
            for (FreeSpace.Avoid a : space.avoids) {
                Envelope e = a.raw.getEnvelopeInternal();
                for (int i = 0; i < 400; i++) {
                    double ax = e.getMinX() - 15 + r.nextDouble() * (e.getWidth() + 30);
                    double ay = e.getMinY() - 15 + r.nextDouble() * (e.getHeight() + 30);
                    double bx = i % 5 == 0 ? ax : ax + (r.nextDouble() - 0.5) * 60;
                    double by = i % 5 == 0 ? ay : ay + (r.nextDouble() - 0.5) * 60;
                    org.locationtech.jts.geom.Geometry seg = i % 5 == 0
                            ? ru.lct.heatnet.geo.GeoJsonGeometries.GF.createPoint(new org.locationtech.jts.geom.Coordinate(ax, ay))
                            : ru.lct.heatnet.geo.GeoJsonGeometries.GF.createLineString(new org.locationtech.jts.geom.Coordinate[]{
                                    new org.locationtech.jts.geom.Coordinate(ax, ay), new org.locationtech.jts.geom.Coordinate(bx, by)});
                    double own = a.distance(ax, ay, bx, by, Double.POSITIVE_INFINITY);
                    double near = a.distance(ax, ay, bx, by, 6.0);
                    double jts = a.raw.distance(seg);
                    worst = Math.max(worst, Math.abs(jts - own));
                    if (jts < 6.0 - 1e-9 ? Math.abs(near - jts) > 1e-9 : near != Double.POSITIVE_INFINITY) {
                        worst = Double.POSITIVE_INFINITY;
                    }
                    n++;
                }
            }
            System.out.printf("violation check: %d segments, max |JTS - own| = %.3e m%n", n, worst);
            int containsMismatch = 0;
            int freeMismatch = 0;
            int m = 0;
            org.locationtech.jts.algorithm.locate.IndexedPointInAreaLocator hardLoc =
                    new org.locationtech.jts.algorithm.locate.IndexedPointInAreaLocator(space.hard);
            Envelope all = space.hard.getEnvelopeInternal();
            for (int i = 0; i < 200_000; i++) {
                double x = all.getMinX() - 20 + r.nextDouble() * (all.getWidth() + 40);
                double y = all.getMinY() - 20 + r.nextDouble() * (all.getHeight() + 40);
                org.locationtech.jts.geom.Coordinate c = new org.locationtech.jts.geom.Coordinate(x, y);
                org.locationtech.jts.geom.Point p = ru.lct.heatnet.geo.GeoJsonGeometries.GF.createPoint(c);
                boolean jtsFree = hardLoc.locate(c) == org.locationtech.jts.geom.Location.EXTERIOR;
                double edge = space.hard.getBoundary().distance(p);
                if (jtsFree != space.pointFree(x, y) && edge > 3 * FreeSpace.EPS) {
                    freeMismatch++;
                }
                FreeSpace.Avoid a = space.avoids.get(i % space.avoids.size());
                Envelope e = a.raw.getEnvelopeInternal();
                double ax = e.getMinX() - 5 + r.nextDouble() * (e.getWidth() + 10);
                double ay = e.getMinY() - 5 + r.nextDouble() * (e.getHeight() + 10);
                org.locationtech.jts.geom.Point q = ru.lct.heatnet.geo.GeoJsonGeometries.GF.createPoint(
                        new org.locationtech.jts.geom.Coordinate(ax, ay));
                if (a.raw.covers(q) != a.contains(ax, ay) && a.raw.getBoundary().distance(q) > 1e-9) {
                    containsMismatch++;
                }
                m++;
            }
            System.out.printf("point checks: %d, pointFree mismatches %d, contains mismatches %d%n", m, freeMismatch,
                    containsMismatch);
            return;
        }
        Ports ports = new Ports(scene, space);
        VisGraph g = VisGraph.build(space, ports.px, ports.py, ports.owner);
        ExistingNet net = new ExistingNet(scene);
        Taps taps = new Taps(net, space, g);
        Search s = new Search(g);
        int k = ports.terms.size();
        double[][] d = new double[k + 1][k + 1];
        double star = 0;
        for (int i = 0; i < k; i++) {
            Ports.Terminal t = ports.terms.get(i);
            int[] nodes = new int[t.count];
            double[] init = new double[t.count];
            for (int j = 0; j < t.count; j++) {
                nodes[j] = g.portBase + t.first + j;
                init[j] = ports.stub[t.first + j];
            }
            s.startNodes(nodes, init, t.count);
            double[] best = new double[k + 1];
            java.util.Arrays.fill(best, Double.POSITIVE_INFINITY);
            int v;
            while ((v = s.next()) >= 0) {
                double dv = s.dist[v];
                for (Taps.Option o : taps.from(v)) {
                    best[k] = Math.min(best[k], dv + o.leg);
                }
                if (g.kind[v] == VisGraph.PORT) {
                    int owner = g.owner[v];
                    int port = v - g.portBase;
                    best[owner] = Math.min(best[owner], dv + ports.stub[port]);
                }
            }
            for (int j = 0; j <= k; j++) {
                d[i][j] = best[j];
                d[j][i] = best[j];
            }
            star += best[k];
            System.out.printf("term %s flow %.2f toNet %.1f%n", t.id, t.flow, best[k]);
        }
        boolean[] in = new boolean[k + 1];
        double[] key = new double[k + 1];
        int[] from = new int[k + 1];
        java.util.Arrays.fill(key, Double.POSITIVE_INFINITY);
        key[k] = 0;
        from[k] = -1;
        double mst = 0;
        for (int it = 0; it <= k; it++) {
            int u = -1;
            for (int j = 0; j <= k; j++) {
                if (!in[j] && (u < 0 || key[j] < key[u])) {
                    u = j;
                }
            }
            in[u] = true;
            mst += key[u];
            if (from[u] >= 0) {
                System.out.printf("mst %s - %s %.1f%n", u == k ? "net" : ports.terms.get(u).id,
                        from[u] == k ? "net" : ports.terms.get(from[u]).id, key[u]);
            }
            for (int j = 0; j <= k; j++) {
                if (!in[j] && d[u][j] < key[j]) {
                    key[j] = d[u][j];
                    from[j] = u;
                }
            }
        }
        System.out.printf("star %.1f m, MST(metric closure) %.1f m, VG %d/%d%n", star, mst, g.n, g.edgeCount());
        if (args.length > 0) {
            Model model = new Model(prices, net, ports);
            Optimizer opt = new Optimizer(model, g, taps, ports, space, Long.parseLong(args.length > 1 ? args[1] : "1"));
            long t0 = System.nanoTime();
            Forest best = opt.solve(new ArrayList<>(), Long.parseLong(args[0]) * 1_000_000L);
            Model.Eval e = model.evaluate(best);
            System.out.printf("best %.0f pipes %.0f chambers %.0f taps %.0f recon %.0f+%.0f L %.1f tieIns %d in %d ms%n",
                    e.total, e.pipes, e.chambers, e.taps, e.recon, e.reconChambers, e.length, e.tieIns,
                    (System.nanoTime() - t0) / 1_000_000);
            dump(best, ports, args.length > 2 ? args[2] : "/tmp/opt/forest.json");
        }
    }

    static void dump(Forest f, Ports ports, String file) throws Exception {
        StringBuilder sb = new StringBuilder("[");
        boolean first = true;
        for (Forest.Node n : f.nodes) {
            if (!first) {
                sb.append(',');
            }
            first = false;
            sb.append("{\"id\":").append(n.id).append(",\"type\":").append(n.type)
                    .append(",\"x\":").append(n.x).append(",\"y\":").append(n.y)
                    .append(",\"parent\":").append(n.parent == null ? -1 : n.parent.id)
                    .append(",\"dn\":").append(n.dn).append(",\"flow\":").append(n.flow)
                    .append(",\"kids\":").append(n.kids.size())
                    .append(",\"term\":\"").append(n.term >= 0 ? ports.terms.get(n.term).id : "").append("\"")
                    .append(",\"path\":[");
            if (n.path != null) {
                for (int i = 0; i < n.path.length; i++) {
                    if (i > 0) {
                        sb.append(',');
                    }
                    sb.append(n.path[i]);
                }
            }
            sb.append("]}");
        }
        sb.append(']');
        java.nio.file.Files.writeString(java.nio.file.Path.of(file), sb.toString());
    }
}
