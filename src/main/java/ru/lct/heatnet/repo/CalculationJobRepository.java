package ru.lct.heatnet.repo;

import java.util.List;
import java.util.UUID;
import org.springframework.data.jpa.repository.JpaRepository;
import ru.lct.heatnet.persist.CalculationJob;

public interface CalculationJobRepository extends JpaRepository<CalculationJob, UUID> {
    List<CalculationJob> findByDatasetIdOrderByCreatedAtDesc(UUID datasetId);
}
