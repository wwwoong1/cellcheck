package com.ssafy.cellcheck.domain.notice.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class NoticeRequest {
    private String noticeTitle;
    private String noticeContent;
    private Integer noticeType; // 0 - 점검, 1 - 긴급, 2 - 일반
}