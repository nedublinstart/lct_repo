package ru.lct.heatnet.appendix;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@JsonIgnoreProperties(ignoreUnknown = true)
public class AppendixModel {

    private Map<String, List<String>> aliases = new HashMap<>();
    @JsonProperty("feature-kinds")
    private Map<String, KindSpec> featureKinds = new HashMap<>();
    private RoutingSpec routing = new RoutingSpec();
    private List<DiameterSpec> diameters = new ArrayList<>();
    private Map<String, ConstraintSpec> constraints = new HashMap<>();
    private CostsSpec costs = new CostsSpec();
    private RankingSpec ranking = new RankingSpec();
    private DepthSpec depth = new DepthSpec();
    private ExportSpec export = new ExportSpec();
    @JsonProperty("laying-methods")
    private Map<String, Object> layingMethods = new HashMap<>();

    public List<String> aliases(String key) {
        List<String> values = aliases.get(key);
        if (values == null || values.isEmpty()) {
            return List.of(key);
        }
        return values;
    }

    public DiameterSpec diameter(int dn) {
        for (DiameterSpec spec : diameters) {
            if (spec.dn == dn) {
                return spec;
            }
        }
        return null;
    }

    public double newPerM(int dn) {
        DiameterSpec spec = diameter(dn);
        if (spec != null && spec.newPerM > 0) {
            return spec.newPerM;
        }
        return costs.pipePerM(dn);
    }

    public double reconPerM(int dn) {
        DiameterSpec spec = diameter(dn);
        if (spec != null && spec.reconPerM > 0) {
            return spec.reconPerM;
        }
        return costs.reconPipe(dn);
    }

    public ConstraintSpec constraintRule(String type) {
        if (type == null || type.isBlank()) {
            return constraints.getOrDefault("default", ConstraintSpec.avoidDefault());
        }
        ConstraintSpec spec = constraints.get(type);
        if (spec != null) {
            return spec;
        }
        String n = type.toLowerCase(Locale.ROOT).replace('ё', 'е');
        spec = constraints.get(n);
        if (spec != null) {
            return spec;
        }
        String alias = constraintAlias(n);
        if (alias != null) {
            spec = constraints.get(alias);
            if (spec != null) {
                return spec;
            }
        }
        return constraints.getOrDefault("default", ConstraintSpec.avoidDefault());
    }

    static String constraintAlias(String n) {
        if (n.contains("tdtp") || n.contains("тдтп") || n.contains("проезж")
                || n.contains("carriage") || n.contains("roadway")) {
            return "tdtp";
        }
        if (n.contains("tram") || n.contains("трам")) {
            return "tram_tracks";
        }
        if (n.contains("road") || n.contains("дорог") || n.contains("улиц") || n.contains("street")) {
            return "road";
        }
        if (n.contains("rail") || n.contains("желез") || n.contains("жд")) {
            return "railway";
        }
        if (n.contains("water") || n.contains("вод") || n.contains("река") || n.contains("river")) {
            return "water";
        }
        if (n.contains("gas") || n.contains("газ")) {
            return "gas_pipeline";
        }
        if (n.contains("cable") || n.contains("кабел") || n.contains("power")) {
            return "power_cable";
        }
        return null;
    }

    public Map<String, List<String>> getAliases() {
        return aliases;
    }

    public void setAliases(Map<String, List<String>> aliases) {
        this.aliases = aliases != null ? aliases : new HashMap<>();
    }

    public Map<String, KindSpec> getFeatureKinds() {
        return featureKinds;
    }

    public void setFeatureKinds(Map<String, KindSpec> featureKinds) {
        this.featureKinds = featureKinds != null ? featureKinds : new HashMap<>();
    }

    public RoutingSpec getRouting() {
        return routing;
    }

    public void setRouting(RoutingSpec routing) {
        this.routing = routing != null ? routing : new RoutingSpec();
    }

    public List<DiameterSpec> getDiameters() {
        return diameters;
    }

    public void setDiameters(List<DiameterSpec> diameters) {
        this.diameters = diameters != null ? diameters : new ArrayList<>();
    }

    public Map<String, ConstraintSpec> getConstraints() {
        return constraints;
    }

    public void setConstraints(Map<String, ConstraintSpec> constraints) {
        this.constraints = constraints != null ? constraints : new HashMap<>();
    }

    public CostsSpec getCosts() {
        return costs;
    }

    public void setCosts(CostsSpec costs) {
        this.costs = costs != null ? costs : new CostsSpec();
    }

    public RankingSpec getRanking() {
        return ranking;
    }

    public void setRanking(RankingSpec ranking) {
        this.ranking = ranking != null ? ranking : new RankingSpec();
    }

    public DepthSpec getDepth() {
        return depth;
    }

    public void setDepth(DepthSpec depth) {
        this.depth = depth != null ? depth : new DepthSpec();
    }

    public ExportSpec getExport() {
        return export;
    }

    public void setExport(ExportSpec export) {
        this.export = export != null ? export : new ExportSpec();
    }

    public Map<String, Object> getLayingMethods() {
        return layingMethods;
    }

    public void setLayingMethods(Map<String, Object> layingMethods) {
        this.layingMethods = layingMethods != null ? layingMethods : new HashMap<>();
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class KindSpec {
        @JsonProperty("type-values")
        public List<String> typeValues = new ArrayList<>();
        public String geometry;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class RoutingSpec {
        @JsonProperty("grid-max-cells")
        public int gridMaxCells = 500;
        @JsonProperty("min-cell-m")
        public double minCellM = 4;
        @JsonProperty("tap-merge-m")
        public double tapMergeM = 8;
        @JsonProperty("chamber-snap-m")
        public double chamberSnapM = 4;
        @JsonProperty("turn-keep-deg")
        public double turnKeepDeg = 12;
        @JsonProperty("max-path-iterations")
        public int maxPathIterations = 250000;
        @JsonProperty("endpoint-snap-m")
        public double endpointSnapM = 3;
        @JsonProperty("candidate-step-m")
        public double candidateStepM = 35;
        @JsonProperty("max-chamber-degree")
        public int maxChamberDegree = 4;
        @JsonProperty("block-close-m")
        public double blockCloseM = 10;
        @JsonProperty("clearance-m")
        public double clearanceM = 2.0;
        @JsonProperty("path-width-m")
        public double pathWidthM = 0.6;
        @JsonProperty("sidewalk-m")
        public double sidewalkM = 3.5;
        @JsonProperty("street-min-m")
        public double streetMinM = 7;
        @JsonProperty("street-max-m")
        public double streetMaxM = 52;
        @JsonProperty("graze-m")
        public double grazeM = 4.0;
        @JsonProperty("max-street-edge-m")
        public double maxStreetEdgeM = 56;
        @JsonProperty("max-open-edge-m")
        public double maxOpenEdgeM = 90;
        @JsonProperty("along-road-penalty")
        public double alongRoadPenalty = 6.5;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class DiameterSpec {
        public int dn;
        @JsonProperty("capacity_tph")
        public double capacityTph;
        @JsonProperty("max_run_m")
        public double maxRunM = 300;
        @JsonProperty("width_m")
        public double widthM = 1;
        @JsonProperty("height_m")
        public double heightM = 1;
        @JsonProperty("min_depth_m")
        public double minDepthM = 1.2;
        @JsonProperty("new_per_m")
        public double newPerM;
        @JsonProperty("recon_per_m")
        public double reconPerM;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class ConstraintSpec {
        public String action = "AVOID";
        @JsonProperty("buffer_m")
        public double bufferM = 2;
        @JsonProperty("min_angle_deg")
        public Double minAngleDeg;
        @JsonProperty("extra_cost_per_m")
        public double extraCostPerM;
        @JsonProperty("extra_grid_cost")
        public int extraGridCost;
        public String method;
        @JsonProperty("min_distance_m")
        public Double minDistanceM;
        /** Расстояние, зависящее от DN новой сети: ключ — DN, начиная с которого действует значение. */
        @JsonProperty("min_distance_by_dn")
        public Map<Integer, Double> minDistanceByDn = new HashMap<>();
        @JsonProperty("k_spec")
        public double kSpec = 1.0;
        @JsonProperty("extend_m")
        public double extendM;

        public static ConstraintSpec avoidDefault() {
            ConstraintSpec spec = new ConstraintSpec();
            spec.action = "AVOID";
            spec.bufferM = 2;
            return spec;
        }

        public boolean avoid() {
            return "AVOID".equalsIgnoreCase(action);
        }

        /** Минимальное горизонтальное расстояние между габаритами для новой сети диаметра dn, м. */
        public double minDistance(int dn) {
            double d = minDistanceM != null && minDistanceM > 0 ? minDistanceM : Math.max(0, bufferM);
            int from = -1;
            if (minDistanceByDn != null) {
                for (Map.Entry<Integer, Double> e : minDistanceByDn.entrySet()) {
                    if (e.getKey() != null && e.getValue() != null && dn >= e.getKey() && e.getKey() > from) {
                        from = e.getKey();
                        d = e.getValue();
                    }
                }
            }
            return d;
        }

        public boolean special() {
            return "SPECIAL".equalsIgnoreCase(action);
        }

        public boolean cross() {
            return "CROSS".equalsIgnoreCase(action);
        }
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class CostsSpec {
        @JsonProperty("new_pipe_per_m")
        public Map<Integer, Double> newPipePerM = new HashMap<>();
        @JsonProperty("special_multiplier")
        public Map<String, Double> specialMultiplier = new HashMap<>();
        @JsonProperty("new_chamber")
        public Map<Integer, Double> newChamber = new HashMap<>();
        @JsonProperty("tap_in_pipe")
        public double tapInPipe = 900000;
        @JsonProperty("tap_in_chamber")
        public double tapInChamber = 400000;
        @JsonProperty("reconstruction_pipe_per_m")
        public Map<Integer, Double> reconstructionPipePerM = new HashMap<>();
        @JsonProperty("reconstruction_chamber")
        public Map<Integer, Double> reconstructionChamber = new HashMap<>();
        @JsonProperty("unconnected_penalty")
        public double unconnectedPenalty = 100_000_000;
        @JsonProperty("unconnected_fixed")
        public double unconnectedFixed = 100_000_000;
        @JsonProperty("unconnected_per_tph")
        public double unconnectedPerTph = 500_000;
        @JsonProperty("new_chamber_tiers")
        public List<ChamberTier> newChamberTiers = new ArrayList<>();

        public double pipePerM(int dn) {
            return newPipePerM.getOrDefault(dn, 100000.0);
        }

        public double chamber(int dn) {
            if (newChamberTiers != null) {
                for (ChamberTier tier : newChamberTiers) {
                    if (dn <= tier.maxDn) {
                        return tier.cost;
                    }
                }
                if (!newChamberTiers.isEmpty()) {
                    return newChamberTiers.get(newChamberTiers.size() - 1).cost;
                }
            }
            return newChamber.getOrDefault(dn, 3_000_000.0);
        }

        public double reconPipe(int dn) {
            return reconstructionPipePerM.getOrDefault(dn, 80_000.0);
        }

        public double reconChamber(int dn) {
            return chamber(dn);
        }

        public double specialMul(String method) {
            if (method == null) {
                return 1.0;
            }
            return specialMultiplier.getOrDefault(method, 1.0);
        }
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class RankingSpec {
        @JsonProperty("cost_weight")
        public double costWeight = 0.7;
        @JsonProperty("length_weight")
        public double lengthWeight = 0.3;
        @JsonProperty("length_to_cost")
        public double lengthToCost = 80_000;
        @JsonProperty("cost_base")
        public double costBase = 25_000_000;
        @JsonProperty("length_base")
        public double lengthBase = 100;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class ChamberTier {
        @JsonProperty("max_dn")
        public int maxDn;
        public double cost;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class DepthSpec {
        @JsonProperty("enabled-extra")
        public boolean enabledExtra = true;
        @JsonProperty("z_ground")
        public double zGround = 0;
        @JsonProperty("step_m")
        public double stepM = 0.5;
        @JsonProperty("min_depth_m")
        public double minDepthM = 1.0;
        @JsonProperty("max_depth_m")
        public double maxDepthM = 6.0;
        @JsonProperty("utility_depth_m")
        public Map<String, Double> utilityDepthM = new HashMap<>();
        @JsonProperty("clearance_m")
        public double clearanceM = 0.4;
        @JsonProperty("cost_factor_per_m_depth")
        public double costFactorPerMDepth = 0.10;
        @JsonProperty("default_depth_m")
        public double defaultDepthM = 3.0;
        @JsonProperty("max_slope")
        public double maxSlope = 0.10;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class ExportSpec {
        @JsonProperty("collection-name")
        public String collectionName = "heatnet-result";
        public Map<String, String> types = Collections.emptyMap();
    }
}
