package com.ssafy.cellcheck.domain.notice.service;

import com.ssafy.cellcheck.db.entity.User;
import com.ssafy.cellcheck.domain.notice.dto.NoticeRequest;
import com.ssafy.cellcheck.domain.notice.dto.NoticeResponse;

import java.util.List;

public interface NoticeService {

    // 공지 사항 생성
    void createNotice(Long adminId, NoticeRequest noticeRequest);

    // 공지사항 조회
    List<NoticeResponse> getNoticesByUserId(Long userId);

    // 모든 공지사항 조회
    // 모든 공지사항 조회 (중복 제거, 관리자용)
    List<NoticeResponse> getAllUniqueNotices();

    // 새 사용자에게 기존 공지사항 복제
    void copyExistingNoticesToNewUser(User user);

}
