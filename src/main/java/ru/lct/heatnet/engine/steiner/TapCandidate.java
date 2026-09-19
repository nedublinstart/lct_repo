package ru.lct.heatnet.engine.steiner;

import org.locationtech.jts.geom.Coordinate;
import ru.lct.heatnet.engine.greedy.NetworkSnapper;
import ru.lct.heatnet.scene.Chamber;
import ru.lct.heatnet.scene.ExistingSegment;

public final class TapCandidate {
    public final String id;
    public final Coordinate coordinate;
    public final boolean chamber;
    public final String existingId;
    public final String existingKind;
    public final int existingDn;
    public final double existingFlow;
    public final int incidentCount;

    public TapCandidate(String id, Coordinate coordinate, boolean chamber, String existingId,
                        String existingKind, int existingDn, double existingFlow, int incidentCount) {
        this.id = id;
        this.coordinate = coordinate;
        this.chamber = chamber;
        this.existingId = existingId;
        this.existingKind = existingKind;
        this.existingDn = existingDn;
        this.existingFlow = existingFlow;
        this.incidentCount = incidentCount;
    }

    public static TapCandidate chamber(Chamber ch, double existingFlow) {
        return new TapCandidate("TAP-" + ch.id, ch.point.getCoordinate(), true, ch.id, "heat_chamber",
                ch.dn, existingFlow, ch.incidentCount);
    }

    public static TapCandidate segment(ExistingSegment seg, Coordinate c) {
        return new TapCandidate("TAP-" + seg.id + "-" + Math.round(c.x) + "-" + Math.round(c.y),
                c, false, seg.id, "heat_network", seg.dn, seg.existingFlowTph, 0);
    }

    public static TapCandidate of(NetworkSnapper.Snap snap, double existingFlow, int incident) {
        boolean ch = "chamber".equals(snap.kind) || "source".equals(snap.kind);
        String kind = ch ? "heat_chamber" : "heat_network";
        return new TapCandidate("TAP-" + snap.objectId, snap.coordinate, ch, snap.objectId, kind,
                snap.dn, existingFlow, incident);
    }

    public String key() {
        return existingKind + ":" + existingId + ":" + Math.round(coordinate.x) + ":" + Math.round(coordinate.y);
    }

    public double spare(double capacityTph) {
        return capacityTph - existingFlow;
    }
}
