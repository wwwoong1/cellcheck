package com.ssafy.cellcheck.db.repository;

import com.ssafy.cellcheck.db.entity.CircuitStatus;
import com.ssafy.cellcheck.db.entity.Id.CircuitStatusId;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface CircuitStatusRepository extends JpaRepository<CircuitStatus, CircuitStatusId> {
}
