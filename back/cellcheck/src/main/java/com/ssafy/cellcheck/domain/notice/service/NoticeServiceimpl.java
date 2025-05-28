package com.ssafy.cellcheck.domain.notice.service;

import com.ssafy.cellcheck.db.entity.Admin;
import com.ssafy.cellcheck.db.entity.Notice;
import com.ssafy.cellcheck.db.entity.User;
import com.ssafy.cellcheck.db.repository.AdminRepository;
import com.ssafy.cellcheck.db.repository.NoticeRepository;
import com.ssafy.cellcheck.db.repository.UserRepository;
import com.ssafy.cellcheck.domain.notice.dto.NoticeRequest;
import com.ssafy.cellcheck.domain.notice.dto.NoticeResponse;
import com.ssafy.cellcheck.global.constant.ErrorCode;
import com.ssafy.cellcheck.global.exception.CustomException;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.*;
import java.util.stream.Collectors;

@Service
@Slf4j
@RequiredArgsConstructor
public class NoticeServiceimpl implements NoticeService {

    private final NoticeRepository noticeRepository;
    private final AdminRepository adminRepository;
    private final UserRepository userRepository;


    @Override
    @Transactional
    public void createNotice(Long adminId, NoticeRequest noticeRequest) {

        // 관리자 존재 여부 확인
        Admin admin = adminRepository.findById(adminId)
                .orElseThrow(() -> new CustomException(ErrorCode.ADMIN_NOT_FOUND));


        // 모든 사용자 조회
        List<User> users = userRepository.findAll();

        if (users.isEmpty()) {
            log.warn(" 공지사항을 받을 사용자가 없습니다");
            return;
        }

        List<Notice> notices = new ArrayList<>();

        // 각 사용자별로 공지사항 생성
        for (User user : users) {
            Notice notice = Notice.builder()
                    .noticeTitle(noticeRequest.getNoticeTitle())
                    .noticeContent(noticeRequest.getNoticeContent())
                    .noticeType(noticeRequest.getNoticeType())
                    .admin(admin)
                    .user(user)
                    .build();

            notices.add(notice);
        }

        // 일괄 저장
        noticeRepository.saveAll(notices);
    }

    @Override
    @Transactional(readOnly = true)
    public List<NoticeResponse> getNoticesByUserId(Long userId) {
        // 특정 사용자에게 해당하는 공지사항만 조회 (최신순)
        List<Notice> notices = noticeRepository.findByUserUserIdOrderByCreatedAtDesc(userId);

        return notices.stream()
                .map(notice -> NoticeResponse.builder()
                        .noticeTitle(notice.getNoticeTitle())
                        .noticeContent(notice.getNoticeContent())
                        .noticeType(notice.getNoticeType())
                        .createdAt(notice.getCreatedAt())
                        .build())
                .collect(Collectors.toList());
    }

    @Override
    @Transactional(readOnly = true)
    public List<NoticeResponse> getAllUniqueNotices() {
        // 모든 공지사항 조회 (중복 제거)
        // 공지 제목 + 내용 + 타입을 키로 하고, 가장 최근에 생성된 공지를 값으로 저장
        Map<String, Notice> uniqueNoticesMap = new HashMap<>();

        List<Notice> allNotices = noticeRepository.findAll();

        for (Notice notice : allNotices) {
            String key = notice.getNoticeTitle() + ":" +
                    notice.getNoticeContent() + ":" +
                    notice.getNoticeType();

            // 이미 존재하는 키인 경우, 더 최근 날짜의 공지로 대체
            if (uniqueNoticesMap.containsKey(key)) {
                Notice existingNotice = uniqueNoticesMap.get(key);
                if (notice.getCreatedAt().isAfter(existingNotice.getCreatedAt())) {
                    uniqueNoticesMap.put(key, notice);
                }
            } else {
                uniqueNoticesMap.put(key, notice);
            }
        }

        // Map의 값들을 리스트로 변환하고 생성일 기준 내림차순 정렬
        List<Notice> uniqueNotices = new ArrayList<>(uniqueNoticesMap.values());
        uniqueNotices.sort((n1, n2) -> n2.getCreatedAt().compareTo(n1.getCreatedAt()));

        // DTO로 변환
        return uniqueNotices.stream()
                .map(notice -> NoticeResponse.builder()
                        .noticeTitle(notice.getNoticeTitle())
                        .noticeContent(notice.getNoticeContent())
                        .noticeType(notice.getNoticeType())
                        .createdAt(notice.getCreatedAt())
                        .build())
                .collect(Collectors.toList());
    }
    @Override
    @Transactional
    public void copyExistingNoticesToNewUser(User newUser) {
        log.info("새 사용자 {}에게 기존 공지사항 복제 시작", newUser.getUserId());

        // 이미 존재하는 공지사항들 찾기
        // 중복 방지를 위해 Set 사용 (공지 제목, 내용, 타입, 관리자 ID가 모두 같으면 동일한 공지로 간주)
        Set<String> existingNoticeKeys = new HashSet<>();
        List<Notice> existingNotices = noticeRepository.findAll();

        for (Notice notice : existingNotices) {
            String key = notice.getNoticeTitle() + ":" +
                    notice.getNoticeContent() + ":" +
                    notice.getNoticeType() + ":" +
                    notice.getAdmin().getAdminId();
            existingNoticeKeys.add(key);
        }

        // 중복되지 않는 공지사항만 복제 (이미 해당 사용자에게 배포된 공지가 아닌 경우만)
        List<Notice> noticesForNewUser = new ArrayList<>();

        for (String key : existingNoticeKeys) {
            String[] parts = key.split(":");
            String title = parts[0];
            String content = parts[1];
            Integer type = Integer.parseInt(parts[2]);
            Long adminId = Long.parseLong(parts[3]);

            // 관리자 조회
            Admin admin = adminRepository.findById(adminId)
                    .orElseThrow(() -> new CustomException(ErrorCode.ADMIN_NOT_FOUND));

            // 새 사용자를 위한 공지사항 생성
            Notice newNotice = Notice.builder()
                    .noticeTitle(title)
                    .noticeContent(content)
                    .noticeType(type)
                    .admin(admin)
                    .user(newUser)
                    .build();

            noticesForNewUser.add(newNotice);
        }

        // 한번에 저장
        if (!noticesForNewUser.isEmpty()) {
            noticeRepository.saveAll(noticesForNewUser);
            log.info("새 사용자 {}에게 {} 개의 공지사항이 복제되었습니다", newUser.getUserId(), noticesForNewUser.size());
        } else {
            log.info("새 사용자 {}에게 복제할 공지사항이 없습니다", newUser.getUserId());
        }
    }
}