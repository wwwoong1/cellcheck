package com.ssafy.cellcheck.domain.battery.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;

@Getter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class BatteryLogResponse {
    private Integer id;
    private String batteryType;
    private Boolean inspectionResult;
    private LocalDateTime createdAt;
}