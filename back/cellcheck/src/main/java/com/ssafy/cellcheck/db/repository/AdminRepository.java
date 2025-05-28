package com.ssafy.cellcheck.db.repository;

import com.ssafy.cellcheck.db.entity.Admin;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.Optional;

@Repository
public interface AdminRepository extends JpaRepository<Admin, Long> {

    // 관리자 정보 조회
    Optional<Admin> findByLoginId(String loginId);

    // 중복확인용 ID 존재 여부 확인
    Boolean existsByLoginId(String loginId);

}
