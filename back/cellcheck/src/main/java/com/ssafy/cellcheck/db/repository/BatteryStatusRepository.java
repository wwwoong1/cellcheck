package com.ssafy.cellcheck.db.repository;

import com.ssafy.cellcheck.db.entity.BatteryStatus;
import com.ssafy.cellcheck.db.entity.Id.BatteryStatusId;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface BatteryStatusRepository extends JpaRepository<BatteryStatus, BatteryStatusId> {
}



