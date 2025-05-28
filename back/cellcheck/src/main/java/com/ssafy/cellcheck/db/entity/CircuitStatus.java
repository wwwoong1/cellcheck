package com.ssafy.cellcheck.db.entity;

import com.ssafy.cellcheck.db.entity.Id.CircuitStatusId;
import jakarta.persistence.*;
import lombok.*;
import org.hibernate.annotations.CreationTimestamp;

import java.time.LocalDateTime;

@Entity
@Table(name = "circuit_status")
@IdClass(CircuitStatusId.class)
@Getter
@Builder
@AllArgsConstructor(access = AccessLevel.PROTECTED)
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class CircuitStatus {
    @Id
    @Column(name = "circuit_status_id")
    private Long circuitStatusId;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "device_id", nullable = false)
    private Device device;

    @Column(name = "resistance_temp_A", nullable = false)
    private Float resistanceTempA;

    @Column(name = "resistance_temp_B", nullable = false)
    private Float resistanceTempB;

    @Column(name = "ambient_temp", nullable = false)
    private Float ambientTemp;

    @Column(name = "cooling_fan_A", nullable = false)
    private Boolean coolingFanA;

    @Column(name = "cooling_fan_B", nullable = false)
    private Boolean coolingFanB;

    @Column(name = "is_full_error", nullable = false)
    private Boolean isFullError;

    @Column(name = "is_full_normal", nullable = false)
    private Boolean isFullNormal;

    @Id
    @Column(name = "recorded_at")
    private LocalDateTime recordedAt;
}
