package com.ssafy.cellcheck.db.entity;

import com.ssafy.cellcheck.db.entity.Id.MonitoringId;
import jakarta.persistence.*;
import lombok.*;
import org.hibernate.annotations.CreationTimestamp;

import java.time.LocalDateTime;

@Entity
@Table(name = "monitoring")
@IdClass(MonitoringId.class)
@Getter
@Builder
@AllArgsConstructor(access = AccessLevel.PROTECTED)
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class Monitoring {

    @Id
    @Column(name = "monitoring_id")
    private Long monitoringId;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "device_id")
    private Device device;

    @Column(name = "cpu_percent")
    private Float cpuPercent;

    @Column(name = "mem_usage")
    private Float memUsage;

    @Column(name = "cpu_temp")
    private Float cpuTemp;

    @Column(name = "soc_temp")
    private Float socTemp;

    @Id
    @Column(name = "recorded_at")
    private LocalDateTime recordedAt;



}
