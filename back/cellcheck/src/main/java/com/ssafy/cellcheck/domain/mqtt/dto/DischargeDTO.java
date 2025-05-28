package com.ssafy.cellcheck.domain.mqtt.dto;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Data;

// 배터리 방전
@Data
public class DischargeDTO {

    // 배터리 잔량
    @JsonProperty("SoC")
    private float SoC;

    // 작동중인 회로 타입 (A/B/O (아무회로 동작 x))
    private String circuitType;

}
