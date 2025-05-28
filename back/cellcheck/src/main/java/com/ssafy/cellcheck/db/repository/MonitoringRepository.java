package com.ssafy.cellcheck.db.repository;

import com.ssafy.cellcheck.db.entity.Id.MonitoringId;
import com.ssafy.cellcheck.db.entity.Monitoring;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface MonitoringRepository extends JpaRepository<Monitoring, MonitoringId> {
}
