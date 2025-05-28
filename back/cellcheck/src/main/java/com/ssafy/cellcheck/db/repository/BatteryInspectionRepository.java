package com.ssafy.cellcheck.db.repository;

import com.ssafy.cellcheck.db.entity.BatteryInspection;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;

import java.time.LocalDateTime;
import java.util.List;

@Repository
public interface BatteryInspectionRepository extends JpaRepository<BatteryInspection, Long> {

    //  세션 및 배터리 정보와 함께 모든 배터리 검사 정보 조회
    @Query ("SELECT bi FROM BatteryInspection bi " +
            "JOIN FETCH bi.session s " +
            "JOIN FETCH s.battery b " +
            "ORDER BY s.createdAt DESC")
    List<BatteryInspection> findAllWithSessionAndBattery();

    //특정 일시 이후의 배터리 검사 정보 조회

    @Query("SELECT bi FROM BatteryInspection bi " +
            "JOIN FETCH bi.session s " +
            "JOIN FETCH s.battery b " +
            "WHERE s.createdAt >= :startDate " +
            "ORDER BY s.createdAt DESC")
    List<BatteryInspection> findByCreatedAtAfter(@Param("startDate") LocalDateTime startDate);
}