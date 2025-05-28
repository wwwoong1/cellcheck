package com.ssafy.cellcheck.domain.mqtt.dto;

import lombok.Data;

@Data
public class AppearanceDTO {

    // 외관 검사 결과
    private boolean appearanceInspection;

    // 배터리타입 (AA/AAA/C/D 비정상품은 null)
    private String batteryType;

    // 이미지 URL (보류)
    // private String imageUrl;

}
