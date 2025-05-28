package com.ssafy.cellcheck.domain.auth.service;

import com.ssafy.cellcheck.db.entity.Admin;
import com.ssafy.cellcheck.db.entity.RefreshToken;
import com.ssafy.cellcheck.db.entity.User;
import com.ssafy.cellcheck.db.repository.AdminRepository;
import com.ssafy.cellcheck.db.repository.RefreshTokenRepository;
import com.ssafy.cellcheck.db.repository.UserRepository;
import com.ssafy.cellcheck.global.auth.jwt.JWTToken;
import com.ssafy.cellcheck.global.auth.jwt.JWTUtil;
import com.ssafy.cellcheck.global.constant.ErrorCode;
import com.ssafy.cellcheck.global.exception.CustomException;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.dao.DataIntegrityViolationException;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.Optional;

@Service
@RequiredArgsConstructor
@Slf4j
@Transactional(readOnly = true)
public class RefreshTokenServiceimpl implements RefreshTokenService {

    private final RefreshTokenRepository refreshTokenRepository;
    private final AdminRepository adminRepository;
    private final UserRepository userRepository;
    private final JWTUtil jwtUtil;



    private void saveAdminRefreshToken(Long adminId, String token) {
        try {
            // 관리자 조회
            Admin admin = adminRepository.findById(adminId)
                    .orElseThrow(() -> new CustomException(ErrorCode.ADMIN_NOT_FOUND));

            // 기존 토큰 찾기
            Optional<RefreshToken> existingToken = refreshTokenRepository.findByAdminAdminId(adminId);

            if (existingToken.isPresent()) {
                // 기존 토큰 업데이트
                RefreshToken refreshToken = existingToken.get();
                refreshToken.updateToken(token);
                log.info("관리자 ID {} 리프레시 토큰 업데이트 완료", adminId);
            } else {
                // 새 토큰 생성 및 저장
                RefreshToken refreshToken = RefreshToken.builder()
                        .admin(admin)
                        .user(null)
                        .token(token)
                        .build();
                refreshTokenRepository.save(refreshToken);
                log.info("관리자 ID {} 신규 리프레시 토큰 저장 완료", adminId);
            }
        } catch (Exception e) {
            log.error("관리자 리프레시 토큰 저장 실패: {}", e.getMessage(), e);
            throw new CustomException(ErrorCode.INTERNAL_SERVER_ERROR);
        }
    }


    private void saveUserRefreshToken(Long userId, String token) {
        try {
            // 사용자 조회
            User user = userRepository.findById(userId)
                    .orElseThrow(() -> new CustomException(ErrorCode.USER_NOT_FOUND));

            // 기존 토큰 찾기
            Optional<RefreshToken> existingToken = refreshTokenRepository.findByUserUserId(userId);

            if (existingToken.isPresent()) {
                // 기존 토큰 업데이트
                RefreshToken refreshToken = existingToken.get();
                refreshToken.updateToken(token);
                log.info("사용자 ID {} 리프레시 토큰 업데이트 완료", userId);
            } else {
                // 새 토큰 생성 및 저장
                RefreshToken refreshToken = RefreshToken.builder()
                        .user(user)
                        .admin(null)
                        .token(token)
                        .build();
                refreshTokenRepository.save(refreshToken);
                log.info("사용자 ID {} 신규 리프레시 토큰 저장 완료", userId);
            }
        } catch (Exception e) {
            log.error("사용자 리프레시 토큰 저장 실패: {}", e.getMessage(), e);
            throw new CustomException(ErrorCode.INTERNAL_SERVER_ERROR);
        }
    }

    @Override
    @Transactional
    public void saveRefreshToken(Long id, boolean isAdmin, String token){

        try {
            if (isAdmin) {
                // 관리자 토큰 저장
                saveAdminRefreshToken(id, token);
            } else {
                saveUserRefreshToken(id, token); // 이후 사용자 로그인 및 등록 구현시 교체
            }
        } catch (Exception e) {
            log.error("리프레시 토큰 저장 실패: {}", e.getMessage());
            throw new CustomException(ErrorCode.INTERNAL_SERVER_ERROR);
        }
    }

    // 리프레시 토큰으로 새 토큰 발급
    @Override
    @Transactional
    public JWTToken refreshToken(String refreshToken) {
        try {
            // 토큰 검증
            if (!jwtUtil.verifyRefreshToken(refreshToken)) {
                log.error("유효하지 않은 리프레시 토큰입니다");
                throw new CustomException(ErrorCode.INVALID_REFRESH_TOKEN);
            }

            // 토큰에서 사용자 정보 추출
            Long id = jwtUtil.getId(refreshToken);
            boolean isAdmin = jwtUtil.isAdmin(refreshToken);

            // DB에서 토큰 조회
            RefreshToken tokenEntity = findByToken(refreshToken)
                    .orElseThrow(() -> {
                        log.error("저장된 리프레시 토큰이 없습니다");
                        return new CustomException(ErrorCode.UNAUTHORIZED);
                    });

            // 액세스 토큰만 새로 발급 (리프레시 토큰은 유지)
            JWTToken newTokens = jwtUtil.refreshAccessTokenOnly(refreshToken);

            log.info("액세스 토큰 갱신 완료");
            return newTokens;
        } catch (CustomException e) {
            throw e;
        } catch (Exception e) {
            log.error("토큰 갱신 중 오류 발생: {}", e.getMessage());
            throw new CustomException(ErrorCode.INTERNAL_SERVER_ERROR);
        }
    }

    // 토큰 값으로 리프레시 토큰 조회
    @Override
    public Optional<RefreshToken> findByToken(String token) {
        return refreshTokenRepository.findByToken(token);
    }

    @Override
    public Optional<RefreshToken> findById(Long id, boolean isAdmin) {
        if (isAdmin) {
            return refreshTokenRepository.findByAdminAdminId(id);
        } else {
            return refreshTokenRepository.findByUserUserId(id);
        }
    }


    @Override
    @Transactional
    public void deleteById(Long id, boolean isAdmin) {
        try {
            if (isAdmin) {
                refreshTokenRepository.deleteByAdminAdminId(id);
                log.info("관리자 ID {} 의 리프레시 토큰이 삭제되었습니다", id);
            } else {
                refreshTokenRepository.deleteByUserUserId(id);
                log.info("사용자 ID {} 의 리프레시 토큰이 삭제되었습니다", id);
            }
        } catch (Exception e) {
            log.error("리프레시 토큰 삭제 중 오류 발생: {}", e.getMessage());
            throw new CustomException(ErrorCode.INTERNAL_SERVER_ERROR);
        }
    }

}
