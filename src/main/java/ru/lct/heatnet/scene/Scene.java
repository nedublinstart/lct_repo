package ru.lct.heatnet.scene;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.locationtech.jts.geom.Envelope;
import ru.lct.heatnet.geo.CrsProjector;

public class Scene {
    public final List<ExistingSegment> segments = new ArrayList<>();
    public final List<Chamber> chambers = new ArrayList<>();
    public final List<HeatSource> sources = new ArrayList<>();
    public final List<ProspectiveOks> oks = new ArrayList<>();
    public final List<SpatialConstraint> constraints = new ArrayList<>();
    public final Map<String, String> warnings = new HashMap<>();
    public CrsProjector projector;
    public Envelope envelopeMeters;

    public Envelope envelope() {
        Envelope env = new Envelope();
        segments.forEach(s -> env.expandToInclude(s.line.getEnvelopeInternal()));
        chambers.forEach(c -> env.expandToInclude(c.point.getEnvelopeInternal()));
        sources.forEach(s -> env.expandToInclude(s.point.getEnvelopeInternal()));
        oks.forEach(o -> {
            if (o.connection != null) {
                env.expandToInclude(o.connection.getEnvelopeInternal());
            }
            if (o.footprint != null) {
                env.expandToInclude(o.footprint.getEnvelopeInternal());
            }
        });
        if (!env.isNull()) {
            env.expandBy(150);
        }
        envelopeMeters = env;
        return env;
    }
}
