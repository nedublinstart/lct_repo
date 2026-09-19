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

    public ConstraintSpec constraintRule(String type) {
        if (type == null) {
            return constraints.getOrDefault("default", ConstraintSpec.avoidDefault());
        }
        ConstraintSpec spec = constraints.get(type);
        if (spec != null) {
            return spec;
        }
        spec = constraints.get(type.toLowerCase(Locale.ROOT));
        if (spec != null) {
            return spec;
        }
        return constraints.getOrDefault("default", ConstraintSpec.avoidDefault());
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
        public int gridMaxCells = 800;
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

        public static ConstraintSpec avoidDefault() {
            ConstraintSpec spec = new ConstraintSpec();
            spec.action = "AVOID";
            spec.bufferM = 2;
            return spec;
        }

        public boolean avoid() {
            return "AVOID".equalsIgnoreCase(action);
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
        public double unconnectedPenalty = 250_000_000;

        public double pipePerM(int dn) {
            return newPipePerM.getOrDefault(dn, 100000.0);
        }

        public double chamber(int dn) {
            return newChamber.getOrDefault(dn, 3_000_000.0);
        }

        public double reconPipe(int dn) {
            return reconstructionPipePerM.getOrDefault(dn, 80_000.0);
        }

        public double reconChamber(int dn) {
            return reconstructionChamber.getOrDefault(dn, 2_000_000.0);
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
        public double costFactorPerMDepth = 0.12;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class ExportSpec {
        @JsonProperty("collection-name")
        public String collectionName = "heatnet-result";
        public Map<String, String> types = Collections.emptyMap();
    }
}
