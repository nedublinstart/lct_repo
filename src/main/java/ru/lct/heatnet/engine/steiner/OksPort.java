package ru.lct.heatnet.engine.steiner;

import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.scene.ProspectiveOks;

public final class OksPort {
    public final ProspectiveOks oks;
    public final Coordinate origin;
    public final Coordinate at;

    public OksPort(ProspectiveOks oks, Coordinate origin, Coordinate at) {
        this.oks = oks;
        this.origin = origin;
        this.at = at;
    }

    public String id() {
        return oks.id;
    }

    public double flow() {
        return oks.flowTph;
    }
}
