package com.ssafy.cellcheck.domain.auth.dto;

import lombok.*;

import java.time.LocalDateTime;

@Getter
@Builder
@AllArgsConstructor(access = AccessLevel.PROTECTED)
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class UserInfoResponse {
    private Long userId;

    private String userName;

    private String region;

    private LocalDateTime createdAt;

}
