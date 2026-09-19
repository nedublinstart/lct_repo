package ru.lct.heatnet.engine.steiner;

import java.util.HashMap;
import java.util.Map;

/**
 * Степень тепловой камеры: не больше 4 примыкающих участков (существующие + новые).
 */
public final class DegreeBoard {

    private final int maxDegree;
    private final Map<String, Integer> added = new HashMap<>();
    private final boolean unbounded;

    public DegreeBoard(int maxDegree) {
        this.maxDegree = maxDegree;
        this.unbounded = false;
    }

    private DegreeBoard() {
        this.maxDegree = Integer.MAX_VALUE;
        this.unbounded = true;
    }

    public static DegreeBoard unbounded() {
        return new DegreeBoard();
    }

    public boolean canAttach(TapCandidate tap, int children) {
        if (unbounded) {
            return true;
        }
        int base = tap.chamber ? tap.incidentCount : 1;
        int used = added.getOrDefault(tap.existingId, 0);
        return base + used + children <= maxDegree;
    }

    public void attach(TapCandidate tap, int children) {
        if (unbounded) {
            return;
        }
        added.merge(tap.existingId, children, Integer::sum);
    }
}
