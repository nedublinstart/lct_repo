package ru.lct.heatnet.repo;

import java.util.List;
import java.util.UUID;
import org.springframework.data.jpa.repository.JpaRepository;
import ru.lct.heatnet.persist.Dataset;

public interface DatasetRepository extends JpaRepository<Dataset, UUID> {
    List<Dataset> findAllByOrderByCreatedAtDesc();
}
