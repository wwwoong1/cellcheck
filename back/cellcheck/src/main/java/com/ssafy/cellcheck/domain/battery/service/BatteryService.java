package com.ssafy.cellcheck.domain.battery.service;

import com.ssafy.cellcheck.domain.battery.dto.BatteryLogResponse;

import java.util.List;

public interface BatteryService {

    List<BatteryLogResponse> getAllBatteryLogs(int days);
}