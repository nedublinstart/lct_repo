package ru.lct.heatnet.engine.greedy;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.scene.Scene;

public final class GridPathfinder {

    private static final int[] DX = {1, -1, 0, 0, 1, 1, -1, -1};
    private static final int[] DY = {0, 0, 1, -1, 1, -1, 1, -1};
    private static final double[] DC = {1, 1, 1, 1, Math.sqrt(2), Math.sqrt(2), Math.sqrt(2), Math.sqrt(2)};

    private final double minX;
    private final double minY;
    private final double cell;
    private final int w;
    private final int h;
    private final byte[] blocked;
    private final short[] extra;
    private final int maxIter;

    private GridPathfinder(double minX, double minY, double cell, int w, int h, byte[] blocked, short[] extra, int maxIter) {
        this.minX = minX;
        this.minY = minY;
        this.cell = cell;
        this.w = w;
        this.h = h;
        this.blocked = blocked;
        this.extra = extra;
        this.maxIter = maxIter;
    }

    public static GridPathfinder build(Scene scene, AppendixModel appendix, ObstacleIndex obstacles) {
        Envelope env = scene.envelopeMeters;
        if (env == null || env.isNull()) {
            env = new Envelope(0, 100, 0, 100);
        }
        env.expandBy(40);
        double width = Math.max(40, env.getWidth());
        double height = Math.max(40, env.getHeight());
        int maxCells = Math.max(80, appendix.getRouting().gridMaxCells);
        double cell = Math.max(appendix.getRouting().minCellM, Math.max(width, height) / maxCells);
        int w = Math.max(8, (int) Math.ceil(width / cell) + 1);
        int h = Math.max(8, (int) Math.ceil(height / cell) + 1);
        byte[] blocked = new byte[w * h];
        short[] extra = new short[w * h];
        double minX = env.getMinX();
        double minY = env.getMinY();
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                Coordinate c = new Coordinate(minX + (x + 0.5) * cell, minY + (y + 0.5) * cell);
                int i = y * w + x;
                if (obstacles.blocked(c)) {
                    blocked[i] = 1;
                } else {
                    extra[i] = (short) obstacles.extra(c);
                }
            }
        }
        return new GridPathfinder(minX, minY, cell, w, h, blocked, extra, appendix.getRouting().maxPathIterations);
    }

    public List<Coordinate> find(Coordinate start, Coordinate goal) {
        int s = nearestFree(start);
        int g = nearestFree(goal);
        if (s < 0 || g < 0) {
            return null;
        }
        if (s == g) {
            List<Coordinate> trivial = new ArrayList<>();
            trivial.add(new Coordinate(start));
            trivial.add(new Coordinate(goal));
            return trivial;
        }
        double[] dist = new double[w * h];
        int[] parent = new int[w * h];
        byte[] seen = new byte[w * h];
        java.util.Arrays.fill(dist, Double.POSITIVE_INFINITY);
        java.util.Arrays.fill(parent, -1);
        dist[s] = 0;
        PriorityQueue<Node> pq = new PriorityQueue<>();
        pq.add(new Node(s, heuristic(s, g)));
        int iter = 0;
        while (!pq.isEmpty() && iter++ < maxIter) {
            Node cur = pq.poll();
            if (seen[cur.i] == 1) {
                continue;
            }
            seen[cur.i] = 1;
            if (cur.i == g) {
                break;
            }
            int cx = cur.i % w;
            int cy = cur.i / w;
            for (int k = 0; k < 8; k++) {
                int nx = cx + DX[k];
                int ny = cy + DY[k];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
                    continue;
                }
                int ni = ny * w + nx;
                if (blocked[ni] == 1) {
                    continue;
                }
                double step = DC[k] * cell * (1.0 + extra[ni] / 10.0);
                double nd = dist[cur.i] + step;
                if (nd < dist[ni]) {
                    dist[ni] = nd;
                    parent[ni] = cur.i;
                    pq.add(new Node(ni, nd + heuristic(ni, g)));
                }
            }
        }
        if (parent[g] < 0 && s != g) {
            return null;
        }
        List<Coordinate> path = new ArrayList<>();
        path.add(new Coordinate(goal));
        int i = g;
        int guard = 0;
        while (i != s && i >= 0 && guard++ < w * h) {
            path.add(cellCenter(i));
            i = parent[i];
        }
        path.add(new Coordinate(start));
        Collections.reverse(path);
        return path;
    }

    private int nearestFree(Coordinate c) {
        int x = (int) Math.floor((c.x - minX) / cell);
        int y = (int) Math.floor((c.y - minY) / cell);
        x = clamp(x, 0, w - 1);
        y = clamp(y, 0, h - 1);
        int start = y * w + x;
        if (blocked[start] == 0) {
            return start;
        }
        for (int r = 1; r < Math.max(w, h); r++) {
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    if (Math.abs(dx) != r && Math.abs(dy) != r) {
                        continue;
                    }
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
                        continue;
                    }
                    int i = ny * w + nx;
                    if (blocked[i] == 0) {
                        return i;
                    }
                }
            }
        }
        return -1;
    }

    private Coordinate cellCenter(int i) {
        int x = i % w;
        int y = i / w;
        return new Coordinate(minX + (x + 0.5) * cell, minY + (y + 0.5) * cell);
    }

    private double heuristic(int a, int b) {
        int ax = a % w;
        int ay = a / w;
        int bx = b % w;
        int by = b / w;
        return Math.hypot(ax - bx, ay - by) * cell;
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static final class Node implements Comparable<Node> {
        final int i;
        final double f;

        Node(int i, double f) {
            this.i = i;
            this.f = f;
        }

        @Override
        public int compareTo(Node o) {
            return Double.compare(f, o.f);
        }
    }
}
