package ru.lct.heatnet.engine;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class Variant {
    public String code;
    public String title;
    public String description;
    public List<NewSegment> segments = new ArrayList<>();
    public List<NewChamber> chambers = new ArrayList<>();
    public List<TapPoint> taps = new ArrayList<>();
    public List<TechnicalNode> technicalNodes = new ArrayList<>();
    public List<ReconstructionSegment> reconstructionSegments = new ArrayList<>();
    public List<ReconstructionChamber> reconstructionChambers = new ArrayList<>();
    public List<String> unconnectedOks = new ArrayList<>();
    public Map<String, Double> costBreakdown = new LinkedHashMap<>();
    public double constructionCost;
    public double penalty;
    public double totalCost;
    public double newLengthM;
    public double reconLengthM;
    public double score;
    public List<String> notes = new ArrayList<>();
}
