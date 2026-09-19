package ru.lct.heatnet.scene;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.MultiLineString;
import org.locationtech.jts.geom.Point;
import org.springframework.stereotype.Component;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.geo.CrsProjector;
import ru.lct.heatnet.geo.GeoJsonGeometries;
import ru.lct.heatnet.ingest.FeatureKind;
import ru.lct.heatnet.ingest.PropertyReader;
import ru.lct.heatnet.persist.IngestedFeature;

@Component
public class SceneAssembler {

    private final ObjectMapper mapper = GeoJsonGeometries.mapper();

    public Scene assemble(List<IngestedFeature> features, AppendixModel appendix) {
        Scene scene = new Scene();
        List<Geometry> raw = new ArrayList<>();
        List<Parsed> parsed = new ArrayList<>();
        for (IngestedFeature f : features) {
            Parsed p = parse(f, appendix);
            if (p.geometry != null) {
                raw.add(p.geometry);
            }
            parsed.add(p);
        }
        CrsProjector projector = CrsProjector.detect(raw);
        scene.projector = projector;

        Map<String, Point> connections = new HashMap<>();
        Map<String, ProspectiveOks> oks = new HashMap<>();

        for (Parsed p : parsed) {
            Geometry meters = projector.toMeters(p.geometry);
            switch (p.kind) {
                case EXISTING_SEGMENT:
                    Geometry lineGeom = meters;
                    if (lineGeom instanceof MultiLineString && lineGeom.getNumGeometries() > 0) {
                        lineGeom = lineGeom.getGeometryN(0);
                    }
                    if (lineGeom instanceof LineString) {
                        ExistingSegment s = new ExistingSegment();
                        s.id = p.id;
                        s.line = (LineString) lineGeom;
                        s.dn = p.dn == null ? 150 : p.dn.intValue();
                        s.existingFlowTph = p.flow == null ? 0 : p.flow;
                        s.nextId = p.nextId;
                        scene.segments.add(s);
                    }
                    break;
                case CHAMBER:
                    if (asPoint(meters) != null) {
                        Chamber c = new Chamber();
                        c.id = p.id;
                        c.point = asPoint(meters);
                        c.nextId = p.nextId;
                        c.dn = p.dn == null ? 0 : p.dn.intValue();
                        scene.chambers.add(c);
                    }
                    break;
                case SOURCE:
                    if (asPoint(meters) != null) {
                        HeatSource src = new HeatSource();
                        src.id = p.id;
                        src.point = asPoint(meters);
                        scene.sources.add(src);
                    }
                    break;
                case OKS_PROSPECTIVE:
                    ProspectiveOks o = new ProspectiveOks();
                    o.id = p.id;
                    o.footprint = meters;
                    o.flowTph = p.flow == null ? 0 : p.flow;
                    o.heatLoad = p.heatLoad;
                    o.name = p.name;
                    if (meters != null) {
                        o.connection = meters.getCentroid();
                    }
                    oks.put(o.id, o);
                    break;
                case CONNECTION_POINT:
                    Point connection = asPoint(meters);
                    if (connection != null) {
                        String oksId = p.oksId != null ? p.oksId : p.id;
                        connections.put(oksId, connection);
                        ProspectiveOks standalone = oks.get(oksId);
                        if (standalone == null) {
                            standalone = new ProspectiveOks();
                            standalone.id = oksId;
                            standalone.flowTph = p.flow == null ? 0 : p.flow;
                            standalone.name = p.name;
                            oks.put(oksId, standalone);
                        } else if (standalone.flowTph <= 0 && p.flow != null) {
                            standalone.flowTph = p.flow;
                        }
                        standalone.connection = connection;
                    }
                    break;
                case CONSTRAINT:
                case OKS_EXISTING:
                    if (meters != null) {
                        SpatialConstraint c = new SpatialConstraint();
                        c.id = p.id;
                        c.type = p.constraintType != null ? p.constraintType : "oks";
                        c.geometry = meters;
                        c.rule = appendix.constraintRule(c.type);
                        scene.constraints.add(c);
                    }
                    break;
                default:
                    scene.warnings.put(p.id == null ? "?" : p.id, "Неизвестный тип объекта");
            }
        }
        connections.forEach((oksId, point) -> {
            ProspectiveOks o = oks.get(oksId);
            if (o != null) {
                o.connection = point;
            }
        });
        scene.oks.addAll(oks.values());
        scene.envelope();
        NetworkTopology.infer(scene, appendix);
        return scene;
    }

    private Parsed parse(IngestedFeature f, AppendixModel appendix) {
        Parsed p = new Parsed();
        p.kind = f.getKind();
        p.geometry = GeoJsonGeometries.read(f.getGeometryJson());
        JsonNode props;
        try {
            props = mapper.readTree(f.getPropertiesJson() == null ? "{}" : f.getPropertiesJson());
        } catch (Exception e) {
            props = mapper.createObjectNode();
        }
        PropertyReader reader = new PropertyReader(props);
        p.id = f.getExternalId() != null ? f.getExternalId() : reader.firstNonBlank(appendix.aliases("id"));
        p.flow = reader.firstNum(appendix.aliases("flow"));
        p.nextId = reader.firstNonBlank(appendix.aliases("next_id"));
        p.oksId = reader.firstNonBlank(appendix.aliases("oks_id"));
        Double dn = reader.firstNum(appendix.aliases("dn"));
        p.dn = dn;
        p.heatLoad = reader.firstNum(appendix.aliases("heat_load"));
        p.name = reader.firstNonBlank(appendix.aliases("name"));
        p.constraintType = reader.firstNonBlank(appendix.aliases("constraint_type"));
        if (p.constraintType == null) {
            p.constraintType = reader.firstNonBlank(appendix.aliases("kind"));
        }
        return p;
    }

    private static Point asPoint(Geometry g) {
        if (g instanceof Point) {
            return (Point) g;
        }
        if (g != null && !g.isEmpty()) {
            return g.getCentroid();
        }
        return null;
    }

    private static final class Parsed {
        FeatureKind kind;
        String id;
        Geometry geometry;
        Double flow;
        Double dn;
        String nextId;
        String oksId;
        Double heatLoad;
        String name;
        String constraintType;
    }
}
