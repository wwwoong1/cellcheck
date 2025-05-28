package com.ssafy.cellcheck.domain.notice.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;

@Getter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class NoticeResponse {
    private String noticeTitle;
    private String noticeContent;
    private Integer noticeType; // 0 - 점검, 1 - 긴급, 2 - 일반
    private LocalDateTime createdAt;
}