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
public class CircuitStatusId implements Serializable {
    @Column(name = "circuit_status_id")
    private Long circuitStatusId;

    @Column(name = "recorded_at")
    private LocalDateTime recordedAt;
}