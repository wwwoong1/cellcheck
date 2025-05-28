package com.ssafy.cellcheck.domain.auth.service;

import com.ssafy.cellcheck.domain.auth.dto.*;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public interface AuthService {

    // 관리자 등록
    boolean adminSignup(AdminSignupRequest adminSignupRequest);

    // 관리자 중복 확인
    boolean isAdminIdDuplicate(String loginId);

    // 관리자 로그인
    AdminLoginResponse adminLogin(AdminLoginRequest adminLoginRequest);

    // 사용자 등록 (관리자만 가능)
    boolean registerUser(Long adminId, UserSignupRequest userSignupRequest);

    // 사용자 로그인
    UserLoginResponse userLogin(UserLoginRequest userLoginRequest);

    // 리프레시 토큰으로 새 토큰 발급
    RefreshTokenResponseDto refreshToken(RefreshTokenRequestDto refreshTokenRequestDto);

    // 모든 사용자 정보 조회 (관리자 전용)
    List<UserInfoResponse> getAllUsers();

    // 특정 사용자 조회
    UserInfoResponse getUserById(Long userId);


}
