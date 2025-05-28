package com.ssafy.cellcheck.domain.auth.dto;

import lombok.*;

@Getter
@Builder
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor(access = AccessLevel.PROTECTED)

public class RefreshTokenRequestDto {

    private String refreshToken;
}
