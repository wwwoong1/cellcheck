package com.ssafy.cellcheck.db.repository;

import com.ssafy.cellcheck.db.entity.Device;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface DeviceRepository extends JpaRepository<Device, Long> {
}
