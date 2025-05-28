package com.ssafy.cellcheck.domain.notice.controller;

import com.ssafy.cellcheck.domain.notice.dto.NoticeRequest;
import com.ssafy.cellcheck.domain.notice.dto.NoticeResponse;
import com.ssafy.cellcheck.domain.notice.service.NoticeService;
import com.ssafy.cellcheck.global.auth.dto.CustomUserDetails;
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
@RequestMapping("/api/notice")
@RequiredArgsConstructor
public class NoticeController {

    private final NoticeService noticeService;

    @PostMapping
    @PreAuthorize("hasRole('ROLE_ADMIN')")
    public ResponseEntity<Void> createNotice(@RequestBody NoticeRequest noticeRequest) {
        log.info("공지사항 작성 요청: {}", noticeRequest.getNoticeTitle());

        // 인증된 관리자 ID 가져오기
        Authentication authentication = SecurityContextHolder.getContext().getAuthentication();
        CustomUserDetails userDetails = (CustomUserDetails) authentication.getPrincipal();

        // 관리자 권한 확인
        if (!userDetails.isAdmin()) {
            log.warn("관리자 권한이 없는 사용자의 공지사항 작성 시도");
            return ResponseEntity.status(403).build(); // ACCESS_DENIED
        }

        noticeService.createNotice(userDetails.getId(), noticeRequest);
        return ResponseEntity.ok().build();
    }

    @GetMapping
    public ResponseEntity<List<NoticeResponse>> getNotices() {
        log.info("공지사항 조회 요청");

        // 현재 인증된 사용자 정보 가져오기
        Authentication authentication = SecurityContextHolder.getContext().getAuthentication();
        CustomUserDetails userDetails = (CustomUserDetails) authentication.getPrincipal();

        List<NoticeResponse> notices;

        // 사용자 유형에 따라 다른 메서드 호출
        if (userDetails.isAdmin()) {
            // 관리자는 중복 제거된 모든 공지사항 조회
            notices = noticeService.getAllUniqueNotices();
        } else {
            // 일반 사용자는 자신에게 할당된 공지사항만 조회
            notices = noticeService.getNoticesByUserId(userDetails.getId());
        }

        return ResponseEntity.ok(notices);
    }
}