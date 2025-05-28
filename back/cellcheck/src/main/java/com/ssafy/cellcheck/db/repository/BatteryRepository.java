package com.ssafy.cellcheck.db.repository;

import com.ssafy.cellcheck.db.entity.Battery;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface BatteryRepository extends JpaRepository<Battery, Long> {

    Battery findByBatteryType(String batteryType);

}
