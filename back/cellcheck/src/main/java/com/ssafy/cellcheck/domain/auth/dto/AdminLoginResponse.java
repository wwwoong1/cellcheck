package com.ssafy.cellcheck.domain.auth.dto;

import lombok.*;

@Getter
@Builder
@AllArgsConstructor(access = AccessLevel.PROTECTED)
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class AdminLoginResponse {

    private Long adminId;

    private String loginId;

    private String adminName;

    private String accessToken;

    private String refreshToken;
}
