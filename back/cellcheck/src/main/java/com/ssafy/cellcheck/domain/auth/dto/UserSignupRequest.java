package com.ssafy.cellcheck.domain.auth.dto;

import lombok.*;

@Getter
@Builder
@AllArgsConstructor(access = AccessLevel.PROTECTED)
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class UserSignupRequest {

    private String loginId;
    private String password;
    private String userName;
    private String region;

}
