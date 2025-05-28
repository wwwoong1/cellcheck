package com.ssafy.cellcheck.domain.auth.dto;

import lombok.*;

@Getter
@Builder
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor(access = AccessLevel.PROTECTED)

public class RefreshTokenResponseDto {

    // 공통 필드
    private Long id;
    private String name;
    private boolean isAdmin;
    private String accessToken;
    private String refreshToken;

    // 사용자 일시 (관리자면 null 반환)
    private String region;

}
