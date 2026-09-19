package ru.lct.heatnet.persist;

import java.util.UUID;
import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.EnumType;
import javax.persistence.Enumerated;
import javax.persistence.Id;
import javax.persistence.Index;
import javax.persistence.PrePersist;
import javax.persistence.Table;
import ru.lct.heatnet.ingest.FeatureKind;

@Entity
@Table(name = "ingested_features", indexes = {
        @Index(name = "idx_feat_dataset_kind", columnList = "datasetId,kind")
})
public class IngestedFeature {

    @Id
    private UUID id;
    private UUID datasetId;
    private String externalId;

    @Enumerated(EnumType.STRING)
    private FeatureKind kind;

    @Column(length = 8000)
    private String propertiesJson;

    @Column(columnDefinition = "clob")
    private String geometryJson;

    private double minX;
    private double minY;
    private double maxX;
    private double maxY;

    @PrePersist
    public void prePersist() {
        if (id == null) {
            id = UUID.randomUUID();
        }
    }

    public UUID getId() {
        return id;
    }

    public void setId(UUID id) {
        this.id = id;
    }

    public UUID getDatasetId() {
        return datasetId;
    }

    public void setDatasetId(UUID datasetId) {
        this.datasetId = datasetId;
    }

    public String getExternalId() {
        return externalId;
    }

    public void setExternalId(String externalId) {
        this.externalId = externalId;
    }

    public FeatureKind getKind() {
        return kind;
    }

    public void setKind(FeatureKind kind) {
        this.kind = kind;
    }

    public String getPropertiesJson() {
        return propertiesJson;
    }

    public void setPropertiesJson(String propertiesJson) {
        this.propertiesJson = propertiesJson;
    }

    public String getGeometryJson() {
        return geometryJson;
    }

    public void setGeometryJson(String geometryJson) {
        this.geometryJson = geometryJson;
    }

    public double getMinX() {
        return minX;
    }

    public void setMinX(double minX) {
        this.minX = minX;
    }

    public double getMinY() {
        return minY;
    }

    public void setMinY(double minY) {
        this.minY = minY;
    }

    public double getMaxX() {
        return maxX;
    }

    public void setMaxX(double maxX) {
        this.maxX = maxX;
    }

    public double getMaxY() {
        return maxY;
    }

    public void setMaxY(double maxY) {
        this.maxY = maxY;
    }
}
