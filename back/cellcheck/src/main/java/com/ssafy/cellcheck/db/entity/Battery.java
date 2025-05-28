package com.ssafy.cellcheck.db.entity;

import jakarta.persistence.*;
import lombok.*;

@Entity
@Table(name = "battery")
@Getter
@Builder
@AllArgsConstructor(access = AccessLevel.PROTECTED)
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class Battery {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "battery_id")
    private Long batteryId;

    @Column(name = "battery_type")
    private String batteryType;
}
