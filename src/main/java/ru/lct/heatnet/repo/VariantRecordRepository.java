package ru.lct.heatnet.repo;

import java.util.List;
import java.util.Optional;
import java.util.UUID;
import org.springframework.data.jpa.repository.JpaRepository;
import ru.lct.heatnet.persist.VariantRecord;

public interface VariantRecordRepository extends JpaRepository<VariantRecord, UUID> {
    List<VariantRecord> findByJobIdOrderByRankAsc(UUID jobId);

    Optional<VariantRecord> findByJobIdAndRank(UUID jobId, int rank);
}
