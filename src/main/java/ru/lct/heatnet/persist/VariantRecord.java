package ru.lct.heatnet.persist;

import java.util.UUID;
import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.Id;
import javax.persistence.Lob;
import javax.persistence.PrePersist;
import javax.persistence.Table;

@Entity
@Table(name = "variant_records")
public class VariantRecord {

    @Id
    private UUID id;
    private UUID jobId;
    private int rank;
    private String title;
    private double cost;
    private double lengthM;
    private double score;
    private int unconnectedCount;
    @Column(length = 4000)
    private String unconnectedIds;
    @Column(length = 8000)
    private String summaryJson;
    private String geoJsonPath;
    @Lob
    private String geoJsonInline;

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

    public UUID getJobId() {
        return jobId;
    }

    public void setJobId(UUID jobId) {
        this.jobId = jobId;
    }

    public int getRank() {
        return rank;
    }

    public void setRank(int rank) {
        this.rank = rank;
    }

    public String getTitle() {
        return title;
    }

    public void setTitle(String title) {
        this.title = title;
    }

    public double getCost() {
        return cost;
    }

    public void setCost(double cost) {
        this.cost = cost;
    }

    public double getLengthM() {
        return lengthM;
    }

    public void setLengthM(double lengthM) {
        this.lengthM = lengthM;
    }

    public double getScore() {
        return score;
    }

    public void setScore(double score) {
        this.score = score;
    }

    public int getUnconnectedCount() {
        return unconnectedCount;
    }

    public void setUnconnectedCount(int unconnectedCount) {
        this.unconnectedCount = unconnectedCount;
    }

    public String getUnconnectedIds() {
        return unconnectedIds;
    }

    public void setUnconnectedIds(String unconnectedIds) {
        this.unconnectedIds = unconnectedIds;
    }

    public String getSummaryJson() {
        return summaryJson;
    }

    public void setSummaryJson(String summaryJson) {
        this.summaryJson = summaryJson;
    }

    public String getGeoJsonPath() {
        return geoJsonPath;
    }

    public void setGeoJsonPath(String geoJsonPath) {
        this.geoJsonPath = geoJsonPath;
    }

    public String getGeoJsonInline() {
        return geoJsonInline;
    }

    public void setGeoJsonInline(String geoJsonInline) {
        this.geoJsonInline = geoJsonInline;
    }
}
