package com.ssafy.cellcheck.domain.mqtt.dto;

import lombok.Data;

// 젯슨나노 DTO
@Data
public class SystemDTO {

    //CPU 사용률 (%)
    private float cpu_percent;

    // 메모리 사용량 (바이트)
    private int mem_used;

    //CPU 온도
    private float cpu_temp;

    //SoC 온도
    private float soc_temp;
}
