package com.ssafy.cellcheck.domain.battery.controller;

import com.ssafy.cellcheck.domain.battery.dto.BatteryLogResponse;
import com.ssafy.cellcheck.domain.battery.service.BatteryService;
import com.ssafy.cellcheck.global.auth.dto.CustomUserDetails;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.security.access.prepost.PreAuthorize;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;

@RestController
@Slf4j
@RequestMapping("/api/battery")
@RequiredArgsConstructor
public class BatteryController {

    private final BatteryService batteryService;

    @GetMapping("/log")
    @PreAuthorize("hasRole('ROLE_ADMIN')")
    public ResponseEntity<List<BatteryLogResponse>> getBatteryLogs(
            @RequestParam(name = "days", required = false, defaultValue = "0") int days) {

        log.info("배터리 로그 조회 요청, 최근 {}일 데이터", days > 0 ? days : "전체");

        // 인증된 관리자 확인
        Authentication authentication = SecurityContextHolder.getContext().getAuthentication();
        CustomUserDetails userDetails = (CustomUserDetails) authentication.getPrincipal();

        // 관리자 권한 확인
        if (!userDetails.isAdmin()) {
            log.warn("관리자 권한이 없는 사용자의 배터리 로그 조회 시도");
            return ResponseEntity.status(403).build(); // ACCESS_DENIED
        }

        List<BatteryLogResponse> logs = batteryService.getAllBatteryLogs(days);
        return ResponseEntity.ok(logs);
    }
}