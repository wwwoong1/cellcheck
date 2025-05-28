package com.ssafy.cellcheck.db.entity.Id;

import jakarta.persistence.Column;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.EqualsAndHashCode;
import lombok.NoArgsConstructor;

import java.io.Serializable;
import java.time.LocalDateTime;

@EqualsAndHashCode
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor(access = AccessLevel.PROTECTED)
public class BatteryStatusId implements Serializable {
    @Column(name = "battery_status_id")
    private Long batteryStatusId;

    @Column(name = "recorded_at")
    private LocalDateTime recordedAt;
}