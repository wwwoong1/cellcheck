package com.ssafy.cellcheck.db.repository;

import com.ssafy.cellcheck.db.entity.RefreshToken;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;
import org.springframework.transaction.annotation.Transactional;

import java.util.Optional;

@Repository
public interface RefreshTokenRepository extends JpaRepository<RefreshToken, Long> {

    // 토큰 값을 통한 reftoken 조회
    Optional<RefreshToken> findByToken(String token);

    // adminId를 통한 refresh 조회
    Optional<RefreshToken> findByAdminAdminId(Long adminid);

    // userId를 통한 refresh 조회
    Optional<RefreshToken> findByUserUserId(Long userId);

    // adminId를 통한 refresh 삭제
    @Transactional
    void deleteByAdminAdminId(Long adminId);

    // userId를 통한 refresh 삭제
    @Transactional
    void deleteByUserUserId(Long userId);


}
