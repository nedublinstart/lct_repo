package ru.lct.heatnet.repo;

import java.util.List;
import java.util.UUID;
import org.springframework.data.domain.Pageable;
import org.springframework.data.jpa.repository.JpaRepository;
import ru.lct.heatnet.ingest.FeatureKind;
import ru.lct.heatnet.persist.IngestedFeature;

public interface IngestedFeatureRepository extends JpaRepository<IngestedFeature, UUID> {
    List<IngestedFeature> findByDatasetId(UUID datasetId);

    List<IngestedFeature> findByDatasetIdAndKind(UUID datasetId, FeatureKind kind);

    List<IngestedFeature> findByDatasetId(UUID datasetId, Pageable pageable);

    long countByDatasetId(UUID datasetId);
}
