package com.ssafy.cellcheck.domain.auth.service;

import com.ssafy.cellcheck.db.entity.Admin;
import com.ssafy.cellcheck.db.entity.User;
import com.ssafy.cellcheck.db.repository.AdminRepository;
import com.ssafy.cellcheck.db.repository.UserRepository;
import com.ssafy.cellcheck.domain.auth.dto.*;
import com.ssafy.cellcheck.domain.notice.service.NoticeService;
import com.ssafy.cellcheck.global.auth.jwt.JWTToken;
import com.ssafy.cellcheck.global.auth.jwt.JWTUtil;
import com.ssafy.cellcheck.global.constant.ErrorCode;
import com.ssafy.cellcheck.global.exception.CustomException;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.client.RestClient;

import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
@Slf4j
public class AuthServiceImpl implements AuthService {

    private final AdminRepository adminRepository;

    private final UserRepository userRepository;

    private final PasswordEncoder passwordEncoder;

    private final JWTUtil jwtUtil;

    private final RefreshTokenService refreshTokenService;

    private final RestClient.Builder builder;
    private final NoticeService noticeService;

    @Override
    @Transactional
    public boolean adminSignup(AdminSignupRequest adminSignupRequest) {
        try {
            // ID 중복 확인
            if (isAdminIdDuplicate(adminSignupRequest.getLoginId())) {
                throw new CustomException(ErrorCode.EXIST_ID);
            } else {
                Admin admin = Admin.builder()
                        .loginId(adminSignupRequest.getLoginId())
                        .password(passwordEncoder.encode(adminSignupRequest.getPassword())) // 비밀번호 암호화
                        .adminName(adminSignupRequest.getAdminName())
                        .build();

                // DB에 저장
                adminRepository.save(admin);
                return true;
            }
        } catch (Exception e) {
            log.error("관리자 등록 실패 : {}", e.getMessage());
            return false;
        }
    }

    @Override
    public  boolean isAdminIdDuplicate(String loginId) {
        return adminRepository.existsByLoginId(loginId);
    }

    // 관리자 로그인
    @Override
    @Transactional
    public AdminLoginResponse adminLogin(AdminLoginRequest adminLoginRequest) {
        // 사용자 정보 조회
        Admin admin = adminRepository.findByLoginId(adminLoginRequest.getLoginId())
                .orElseThrow(() -> new CustomException(ErrorCode.USER_NOT_FOUND));

        // 비밀번호 검증
        boolean matchPassword = passwordEncoder.matches(adminLoginRequest.getPassword(), admin.getPassword());

        if (matchPassword) {
            // JWT 토큰 발급 ( 관리자 여부 : true)
            JWTToken jwtToken = jwtUtil.createTokens(admin.getAdminId(), true);

            // 리프레시 토큰 저장
            refreshTokenService.saveRefreshToken(admin.getAdminId(), true, jwtToken.getRefreshToken());

            return AdminLoginResponse.builder()
                    .adminId(admin.getAdminId())
                    .loginId(admin.getLoginId())
                    .adminName(admin.getAdminName())
                    .accessToken(jwtToken.getAccessToken())
                    .refreshToken(jwtToken.getRefreshToken())
                    .build();

        } else {
            throw new CustomException(ErrorCode.AUTH_FAILURE);
        }

    }

    // 사용자 등록
    @Override
    @Transactional
    public boolean registerUser(Long adminId, UserSignupRequest userSignupRequest) {
        try {
            // 관리자  존재 여부 확인
            Admin admin = adminRepository.findById(adminId)
                    .orElseThrow(() -> new CustomException(ErrorCode.USER_NOT_FOUND));

            // 로그인 ID 중복 확인
            if (userRepository.existsByLoginId(userSignupRequest.getLoginId())) {
                throw new CustomException(ErrorCode.EXIST_ID);
            }

            User user = User.builder()
                    .admin(admin) // 관리자 연결
                    .loginId(userSignupRequest.getLoginId())
                    .password(passwordEncoder.encode(userSignupRequest.getPassword()))
                    .userName(userSignupRequest.getUserName())
                    .region(userSignupRequest.getRegion())
                    .build();

            // 사용자 저장 및 저장된 사용자 객체 받기
            User savedUser = userRepository.save(user);

            // 기존 공지사항 복제 - 저장된 사용자 객체 전달
            noticeService.copyExistingNoticesToNewUser(savedUser);

            return true;
        } catch (Exception e) {
            log.error("사용자 등록 실패 : {}", e.getMessage());
            return false;
        }

    }

    @Override
    @Transactional
    public UserLoginResponse userLogin(UserLoginRequest userLoginRequest) {
        // 사용자 정보 조회
        User user = userRepository.findByLoginId(userLoginRequest.getLoginId())
                .orElseThrow(() -> new CustomException(ErrorCode.USER_NOT_FOUND));

        // 비밀번호 검증
        boolean matchPassword = passwordEncoder.matches(userLoginRequest.getPassword(), user.getPassword());

        if (matchPassword) {
            // JWT 토큰 발급 및 관리자여부 false
            JWTToken jwtToken = jwtUtil.createTokens(user.getUserId(), false);

            // 리프레시 토큰 저장
            refreshTokenService.saveRefreshToken(user.getUserId(), false, jwtToken.getRefreshToken());

            // 응답 객체 생성
            return UserLoginResponse.builder()
                    .userId(user.getUserId())
                    .loginId(user.getLoginId())
                    .userName(user.getUserName())
                    .region(user.getRegion())
                    .accessToken(jwtToken.getAccessToken())
                    .refreshToken(jwtToken.getRefreshToken())
                    .build();

        } else {
            throw new CustomException(ErrorCode.AUTH_FAILURE);
        }

    }

    // 토큰 갱신
    @Override
    @Transactional
    public RefreshTokenResponseDto refreshToken(RefreshTokenRequestDto refreshTokenRequestDto) {
        // 새 토큰 발급
        JWTToken newToken = refreshTokenService.refreshToken(refreshTokenRequestDto.getRefreshToken());

        // 토큰에서 사용자 정보 추출
        Long id = jwtUtil.getId(newToken.getAccessToken());
        boolean isAdmin = jwtUtil.isAdmin(newToken.getAccessToken());

        // 공통 객체 생성
        RefreshTokenResponseDto.RefreshTokenResponseDtoBuilder builder = RefreshTokenResponseDto.builder()
                .id(id)
                .isAdmin(isAdmin)
                .accessToken(newToken.getAccessToken())
                .refreshToken(newToken.getRefreshToken());

        if (isAdmin) {
            Admin admin = adminRepository.findById(id)
                    .orElseThrow(() -> new CustomException(ErrorCode.ADMIN_NOT_FOUND));
            builder.name(admin.getAdminName());
        } else {
            User user = userRepository.findById(id)
                    .orElseThrow(() -> new  CustomException(ErrorCode.USER_NOT_FOUND));
            builder.name(user.getUserName())
                    .region(user.getRegion());
        }

        return builder.build();
    }

    @Override
    @Transactional(readOnly = true)
    public List<UserInfoResponse> getAllUsers() {
        List<User> users = userRepository.findAll();

        return users.stream()
                .map(user -> UserInfoResponse.builder()
                        .userId(user.getUserId())
                        .userName(user.getUserName())
                        .region(user.getRegion())
                        .createdAt(user.getCreatedAt())
                        .build())
                .collect(Collectors.toList());
    }

    @Override
    @Transactional(readOnly = true)
    public UserInfoResponse getUserById(Long userId) {
        User user = userRepository.findById(userId)
                .orElseThrow(() -> new CustomException(ErrorCode.USER_NOT_FOUND));

        return UserInfoResponse.builder()
                .userId(user.getUserId())
                .userName(user.getUserName())
                .region(user.getRegion())
                .createdAt(user.getCreatedAt())
                .build();
    }
}