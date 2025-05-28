package com.ssafy.cellcheck.domain.auth.dto;

import lombok.*;

@Getter
@Builder
@AllArgsConstructor(access = AccessLevel.PROTECTED)
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class UserLoginResponse {

    private Long userId;

    private String loginId;

    private String userName;

    private String region;

    private String accessToken;

    private String refreshToken;

}
