package ru.lct.heatnet.engine;

@FunctionalInterface
public interface ProgressListener {
    void progress(int percent, String message);
}
