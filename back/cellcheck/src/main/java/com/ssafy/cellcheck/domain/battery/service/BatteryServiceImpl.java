package com.ssafy.cellcheck.domain.battery.service;

import com.ssafy.cellcheck.db.entity.BatteryInspection;
import com.ssafy.cellcheck.db.repository.BatteryInspectionRepository;
import com.ssafy.cellcheck.domain.battery.dto.BatteryLogResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.time.temporal.ChronoUnit;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;

@Service
@Slf4j
@RequiredArgsConstructor
public class BatteryServiceImpl implements BatteryService {

    private final BatteryInspectionRepository batteryInspectionRepository;

    @Override
    @Transactional(readOnly = true)
    public List<BatteryLogResponse> getAllBatteryLogs(int days) {
        List<BatteryInspection> inspections;

        if (days > 0) {
            // 최근 n일 데이터 조회
            LocalDateTime startDate = LocalDateTime.now().minusDays(days);
            inspections = batteryInspectionRepository.findByCreatedAtAfter(startDate);
            log.info("최근 {}일 배터리 로그 조회: {} 건", days, inspections.size());
        } else {
            // 전체 데이터 조회
            inspections = batteryInspectionRepository.findAllWithSessionAndBattery();
            log.info("전체 배터리 로그 조회: {} 건", inspections.size());
        }

        // 중복 제거 (1분 이내 기록)
        Map<String, BatteryInspection> dedupMap = new HashMap<>();

        for (BatteryInspection inspection : inspections) {
            LocalDateTime createdAt = inspection.getSession().getCreatedAt();
            String batteryType = inspection.getSession().getBattery().getBatteryType();
            if (batteryType == null) {
                batteryType = "Unknown";
            }

            // 1분 단위로 반올림한 시간을 키로 사용하여 중복 제거
            // 배터리 타입도 키에 포함
            String key = batteryType + "_" + createdAt.truncatedTo(ChronoUnit.MINUTES);

            // 이미 존재하는 키이고, 기존 기록이 더 최근인 경우, 현재 기록은 무시
            if (dedupMap.containsKey(key)) {
                BatteryInspection existing = dedupMap.get(key);
                if (existing.getSession().getCreatedAt().isAfter(createdAt)) {
                    continue;
                }
            }

            // 새 키이거나, 현재 기록이 더 최근인 경우 맵에 추가
            dedupMap.put(key, inspection);
        }

        // 중복 제거된 결과를 시간 내림차순으로 정렬
        List<BatteryInspection> dedupList = new ArrayList<>(dedupMap.values());
        dedupList.sort((a, b) -> b.getSession().getCreatedAt().compareTo(a.getSession().getCreatedAt()));

        // 인덱스를 순차적으로 부여 (1부터 시작)
        AtomicInteger index = new AtomicInteger(1);

        // DTO 변환
        return dedupList.stream()
                .map(inspection -> {
                    String batteryType = inspection.getSession().getBattery().getBatteryType();
                    if (batteryType == null) {
                        batteryType = "Unknown";
                    }

                    return BatteryLogResponse.builder()
                            .id(index.getAndIncrement()) // 순차적 인덱스 부여
                            .batteryType(batteryType)
                            .inspectionResult(inspection.getInspectionResult())
                            .createdAt(inspection.getSession().getCreatedAt())
                            .build();
                })
                .collect(Collectors.toList());
    }
}