package com.ssafy.cellcheck.domain.mqtt.dto;

import lombok.Data;

@Data
public class EnvironmentDTO {
    // 회로 A의 저항 온도
    private String resistanceTemperatureA;

    // 회로 B의 저항 온도

    private String resistanceTemperatureB;

    // 외부 온도 (실내 온도)

    private String ambientTemp;

    // 쿨링팬 A 작동 여부 (0: off, 1: on)

    private int coolingFanA;

    // 쿨링팬 B 작동 여부 (0: off, 1: on)

    private int coolingFanB;

    //  비정상 배터리통 가득 참 여부 (0: 가득참, 1: 안참) DB에 저장할 때는 반대로 저장 (0 -> true, 1 -> false)

    private int isFullError;

    // DB에 저장할 때는 반대로 저장 (0 -> true, 1 -> false)

    private int isFullNormal;


}
