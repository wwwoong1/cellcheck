package com.ssafy.cellcheck.domain.auth.service;

import com.ssafy.cellcheck.db.entity.RefreshToken;
import com.ssafy.cellcheck.global.auth.jwt.JWTToken;

import java.util.Optional;

public interface RefreshTokenService {
    // refresh 저장 or 갱신
    void saveRefreshToken(Long id, boolean isAdmin, String token);

    JWTToken refreshToken(String refreshToken);// refresh 새 토큰세트 발급

    // 토큰 값으로 리프레시 토큰 조회
    Optional<RefreshToken> findByToken(String token);

    //id와 isadmin여부로 리프래시 토큰 조회
    Optional<RefreshToken> findById(Long id, boolean isAdmin);

    // id와 isadmin여부로 리프래시토큰 삭제
    void deleteById(Long id, boolean isAdmin);
}
