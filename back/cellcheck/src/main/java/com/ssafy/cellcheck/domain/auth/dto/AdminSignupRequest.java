package com.ssafy.cellcheck.domain.auth.dto;

import lombok.*;

@Getter
@Builder
@AllArgsConstructor(access = AccessLevel.PROTECTED)
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class AdminSignupRequest {

    private String loginId;

    private String password;

    private String adminName;
}
