package com.ssafy.cellcheck.db.entity.Id;

import jakarta.persistence.Column;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.EqualsAndHashCode;
import lombok.NoArgsConstructor;

import java.io.Serializable;
import java.time.LocalDateTime;

@EqualsAndHashCode //복합키 명시를 위해 필요
@AllArgsConstructor(access = AccessLevel.PROTECTED)
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class MonitoringId implements Serializable {
    @Column(name = "monitoring_id")
    private Long monitoringId;

    @Column(name = "recorded_at")
    private LocalDateTime recordedAt;

}
