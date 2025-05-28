package com.ssafy.cellcheck.domain.auth.controller;

import com.ssafy.cellcheck.domain.auth.dto.*;
import com.ssafy.cellcheck.domain.auth.service.AuthService;
import com.ssafy.cellcheck.domain.auth.service.RefreshTokenService;
import com.ssafy.cellcheck.global.auth.dto.CustomUserDetails;
import jakarta.servlet.http.Cookie;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.security.access.prepost.PreAuthorize;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@Slf4j
@RequestMapping("/api/auth")
@RequiredArgsConstructor
public class AuthController {

    private final AuthService authService;
    private final RefreshTokenService refreshTokenService;

    @PostMapping("/admin-signup")
    public ResponseEntity<Boolean> adminSignup(@RequestBody AdminSignupRequest adminSignupRequest) {
        log.info("관리자 회원 가입 요청 : {}", adminSignupRequest.getLoginId());
        Boolean isSuccess = authService.adminSignup(adminSignupRequest);
        return ResponseEntity.ok(isSuccess);
    }

    @GetMapping("/admin-deplicate-check")
    public ResponseEntity<Boolean> checkAdminIdDuplicate(@RequestParam(name = "loginId") String loginId) {
        log.info("관리자 ID 중복 확인: {}", loginId);
        return ResponseEntity.ok(authService.isAdminIdDuplicate(loginId));
    }

    @PostMapping("/admin-login")
    public ResponseEntity<AdminLoginResponse> adminLogin(@RequestBody AdminLoginRequest adminLoginRequest) {
        log.info("관리자 로그인 요청 : {}", adminLoginRequest.getLoginId());
        AdminLoginResponse loginResponse = authService.adminLogin(adminLoginRequest);
        return ResponseEntity.ok(loginResponse);
    }

    @PostMapping("/user-register")
    public ResponseEntity<Boolean> registerUser(@RequestBody UserSignupRequest userSignupRequest) {
        log.info("사용자 등록 요청 : {}", userSignupRequest.getLoginId());

        // 인증된 관리자 ID 가져오기
        Authentication authentication = SecurityContextHolder.getContext().getAuthentication();
        CustomUserDetails userDetails = (CustomUserDetails) authentication.getPrincipal();
        Long adminId = userDetails.getId();

        Boolean isSuccess = authService.registerUser(adminId, userSignupRequest);
        return ResponseEntity.ok(isSuccess);
    }


    @PostMapping("/user-login")
    public ResponseEntity<UserLoginResponse> userLogin(@RequestBody UserLoginRequest userLoginRequest) {
        log.info("사용자 로그인 요청 : {}", userLoginRequest.getLoginId());
        UserLoginResponse loginResponse = authService.userLogin(userLoginRequest);
        return ResponseEntity.ok(loginResponse);
    }

    // 토큰 갱신
    @PostMapping("/refresh")
    public ResponseEntity<RefreshTokenResponseDto> refreshToken(@RequestBody RefreshTokenRequestDto refreshTokenRequestDto) {
        log.info("토큰 갱신 요청");
        RefreshTokenResponseDto responseDto = authService.refreshToken(refreshTokenRequestDto);
        return ResponseEntity.ok(responseDto);
    }

    @PostMapping("/logout")
    public ResponseEntity<Void> logout(HttpServletRequest request, HttpServletResponse response) {
        log.info("로그아웃 요청");

        Authentication authentication = SecurityContextHolder.getContext().getAuthentication();
        if (authentication != null && authentication.getPrincipal() instanceof CustomUserDetails) {
            CustomUserDetails userDetails = (CustomUserDetails) authentication.getPrincipal();
            Long id = userDetails.getId();
            boolean isAdmin = userDetails.isAdmin();

            // 리프레시 토큰 삭제
            refreshTokenService.deleteById(id, isAdmin);

            // 쿠키 삭제
            Cookie cookie = new Cookie("refreshToken", null);
            cookie.setMaxAge(0);
            cookie.setPath("/");
            response.addCookie(cookie);

            // SecurityContext 초기화
            SecurityContextHolder.clearContext();
        }

        return ResponseEntity.ok().build();


    }

    @GetMapping("/user-info")
    @PreAuthorize("hasRole('ROLE_ADMIN')")
    public ResponseEntity<?> getUserInfo(
            @RequestParam(name = "userId", required = false) Long userId) {
        log.info("사용자 정보 조회 요청, userId: {}", userId != null ? userId : "전체");

        // 인증된 관리자 확인
        Authentication authentication = SecurityContextHolder.getContext().getAuthentication();
        CustomUserDetails userDetails = (CustomUserDetails) authentication.getPrincipal();

        // 관리자 권한 확인
        if (!userDetails.isAdmin()) {
            log.warn("관리자 권한이 없는 사용자의 사용자 정보 조회 시도");
            return ResponseEntity.status(403).build(); // ACCESS_DENIED
        }

        if (userId != null) {
            // 특정 사용자 정보 조회
            UserInfoResponse userInfo = authService.getUserById(userId);
            return ResponseEntity.ok(userInfo);
        } else {
            // 전체 사용자 정보 조회
            List<UserInfoResponse> users = authService.getAllUsers();
            return ResponseEntity.ok(users);
        }
    }
}