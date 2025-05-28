package com.ssafy.cellcheck.db.entity;

import com.ssafy.cellcheck.db.entity.Id.BatteryStatusId;
import jakarta.persistence.*;
import lombok.*;
import org.hibernate.annotations.CreationTimestamp;

import java.time.LocalDateTime;

@Entity
@Table(name = "battery_status")
@IdClass(BatteryStatusId.class)
@Getter
@Builder
@AllArgsConstructor(access = AccessLevel.PROTECTED)
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class BatteryStatus {
    @Id
    @Column(name = "battery_status_id")
    private Long batteryStatusId;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "session_id", nullable = false)
    private Session session;

    @Column(name = "soc", nullable = false)
    private Long soc;

    // 0, 1, 2
    @Column(name = "circuit_type", nullable = false)
    private Integer circuitType;

    @Id
    @Column(name = "recorded_at")
    private LocalDateTime recordedAt;

}
