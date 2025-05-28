package com.ssafy.cellcheck.domain.mqtt.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.cellcheck.db.entity.*;
import com.ssafy.cellcheck.db.repository.*;
import com.ssafy.cellcheck.domain.mqtt.dto.AppearanceDTO;
import com.ssafy.cellcheck.domain.mqtt.dto.DischargeDTO;
import com.ssafy.cellcheck.domain.mqtt.dto.EnvironmentDTO;
import com.ssafy.cellcheck.domain.mqtt.dto.SystemDTO;
import jakarta.persistence.EntityManager;
import jakarta.persistence.PersistenceContext;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.math.BigInteger;
import java.time.LocalDateTime;

@Slf4j
@Service
@RequiredArgsConstructor
public class MqttServiceImpl implements MqttService {

    private final BatteryRepository batteryRepository;
    private final DeviceRepository deviceRepository;
    private final SessionRepository sessionRepository;
    private final BatteryInspectionRepository batteryInspectionRepository;
    private final BatteryStatusRepository batteryStatusRepository;
    private final CircuitStatusRepository circuitStatusRepository;
    private final MonitoringRepository monitoringRepository;

    // 하이퍼테이블 처리를 위한 entitymanager
    @PersistenceContext
    private EntityManager entityManager;

    private final ObjectMapper objectMapper = new ObjectMapper();


    // 배터리 외관검사 결과 저장
    @Override
    @Transactional
    public void processAppearanceData(String payload) throws Exception {
        try {
            // JSON -> DTO
            AppearanceDTO dto = objectMapper.readValue(payload, AppearanceDTO.class);

            log.debug("외관 검사 데이터 파싱 : {}", dto);

            // 배터리 타입에 따른 엔티티 조회
            Battery battery = getBatteryByType(dto.getBatteryType());

            // 디바이스 ID 고정 (1번만 사용)
            Device device = deviceRepository.findById(1L)
                    .orElseThrow(() -> new RuntimeException("디바이스를 찾을 수 없습니다."));

            // 세션 생성
            Session session = Session.builder()
                    .battery(battery)
                    .device(device)
                    .build();
            Session savedSession = sessionRepository.save(session);

            // 검사 결과 저장
            BatteryInspection batteryInspection = BatteryInspection.builder()
                    .session(savedSession)
                    .inspectionResult(!dto.isAppearanceInspection())
                    .build();
            batteryInspectionRepository.save(batteryInspection);

            log.info("외관 검사 데이터 저장 완료 - 배터리 타입: {}, 검사 결과: {}",
                    dto.getBatteryType(), !dto.isAppearanceInspection() ? "정상" : "비정상");

        }   catch (Exception e) {
                log.error("외관 검사 데이터 처리 중 오류 발생", e);
                throw new RuntimeException("외관 검사 데이터 처리 실패: " + e.getMessage(), e);
        }
    }

    // 모니터링 데이터 처리 및 DB 저장
    @Override
    @Transactional
    public void processEnvironmentData(String payload) throws Exception {
        try {
            // JSON 문자열을 DTO로 변환
            EnvironmentDTO dto = objectMapper.readValue(payload, EnvironmentDTO.class);

            log.debug("환경 모니터링 데이터 파싱: {}", dto);

            // 고정된 디바이스 ID 사용 (Jetson Nano)
            Device device = deviceRepository.findById(1L)
                    .orElseThrow(() -> new RuntimeException("디바이스를 찾을 수 없습니다."));

            // 시퀀스를 위한 ID 조회 (하이퍼테이블 사용시 필요)
            Long nextId = getNextCircuitStatusId();

            // 회로 상태 저장 (저항 온도, 쿨링팬 상태, 배터리통 상태 등)
            CircuitStatus circuitStatus = CircuitStatus.builder()
                    .circuitStatusId(nextId)
                    .device(device)
                    .resistanceTempA(Float.parseFloat(dto.getResistanceTemperatureA()))
                    .resistanceTempB(Float.parseFloat(dto.getResistanceTemperatureB()))
                    .ambientTemp(Float.parseFloat(dto.getAmbientTemp()))
                    .coolingFanA(dto.getCoolingFanA() == 1) // int -> boolean 변환
                    .coolingFanB(dto.getCoolingFanB() == 1) // int -> boolean 변환
                    .isFullError(dto.getIsFullError() == 0) // DB에 반대로 저장 (0 -> true, 1 -> false)
                    .isFullNormal(dto.getIsFullNormal() == 0) // DB에 반대로 저장 (0 -> true, 1 -> false)
                    .recordedAt(LocalDateTime.now())
                    .build();

            circuitStatusRepository.save(circuitStatus);

            log.info("환경 모니터링 데이터 저장 완료 - 저항 온도 A: {}, 저항 온도 B: {}, 주변 온도: {}",
                    dto.getResistanceTemperatureA(), dto.getResistanceTemperatureB(), dto.getAmbientTemp());
        } catch (Exception e) {
            log.error("환경 모니터링 데이터 처리 중 오류 발생", e);
            throw new RuntimeException("환경 모니터링 데이터 처리 실패: " + e.getMessage(), e);
        }

    }

    // 배터리 잔량 및 타입 정보 저장
    public void processDischargeData(String payload) throws Exception {
        try {
            // JSON 문자열을 DTO로 변환
            DischargeDTO dto = objectMapper.readValue(payload, DischargeDTO.class);

            log.debug("방전 데이터 파싱: {}", dto);

            // 최신 세션 ID 조회 (배터리 방전 처리를 위해 가장 최근 세션 사용)
            Session latestSession = sessionRepository.findLatestSession();
            if (latestSession == null) {
                throw new RuntimeException("활성화된 세션이 없습니다. 먼저 배터리 검사를 실행해주세요.");
            }

            // 시퀀스를 위한 ID 조회 (하이퍼테이블 사용시 필요)
            Long nextId = getNextBatteryStatusId();

            // circuitType 변환 (A -> 0, B -> 1, 그 외(O) -> 2)
            Integer circuitType;
            if ("A".equals(dto.getCircuitType())) {
                circuitType = 0;
            } else if ("B".equals(dto.getCircuitType())) {
                circuitType = 1;
            } else {
                circuitType = 2; // 기본값 ("O" 또는 다른 값)
            }

            // 배터리 상태 저장 (SoC, 회로 타입)
            BatteryStatus batteryStatus = BatteryStatus.builder()
                    .batteryStatusId(nextId)
                    .session(latestSession)
                    .soc((long) Math.round(dto.getSoC() * 100)) // float -> long 변환 (백분율로 저장)
                    .circuitType(circuitType)
                    .recordedAt(LocalDateTime.now())
                    .build();

            batteryStatusRepository.save(batteryStatus);

            log.info("방전 데이터 저장 완료 - SoC: {}%, 회로 타입: {}",
                    dto.getSoC(), getCircuitTypeString(circuitType));
        } catch (Exception e) {
            log.error("방전 데이터 처리 중 오류 발생", e);
            throw new RuntimeException("방전 데이터 처리 실패: " + e.getMessage(), e);
        }
    }

    // 시스템 상태 데이터 처리 및 DB 저장
    @Override
    @Transactional
    public void processSystemData(String payload) throws Exception {
        try {
            // JSON 문자열을 DTO로 변환
            SystemDTO dto = objectMapper.readValue(payload, SystemDTO.class);

            log.debug("시스템 상태 데이터 파싱: {}", dto);

            // 고정된 디바이스 ID 사용 (Jetson Nano)
            Device device = deviceRepository.findById(1L)
                    .orElseThrow(() -> new RuntimeException("디바이스를 찾을 수 없습니다."));

            // 시퀀스를 위한 ID 조회 (하이퍼테이블 사용시 필요)
            Long nextId = getNextMonitoringId();

            // 시스템 모니터링 정보 저장 (CPU/메모리 사용량, 온도)
            Monitoring monitoring = Monitoring.builder()
                    .monitoringId(nextId)
                    .device(device)
                    .cpuPercent(dto.getCpu_percent())
                    .memUsage((float) dto.getMem_used()) // int -> float 변환
                    .cpuTemp(dto.getCpu_temp())
                    .socTemp(dto.getSoc_temp())
                    .recordedAt(LocalDateTime.now())
                    .build();

            monitoringRepository.save(monitoring);

            log.info("시스템 상태 데이터 저장 완료 - CPU 사용률: {}%, CPU 온도: {}°C",
                    dto.getCpu_percent(), dto.getCpu_temp());
        } catch (Exception e) {
            log.error("시스템 상태 데이터 처리 중 오류 발생", e);
            throw new RuntimeException("시스템 상태 데이터 처리 실패: " + e.getMessage(), e);
        }
    }



    private String getCircuitTypeString(Integer circuitType) {
        switch (circuitType) {
            case 0:
                return "A";
            case 1:
                return "B";
            case 2:
                return "O";
            default:
                return "UnKnown";
        }
    }

    private Battery getBatteryByType(String batteryType) {
        if (batteryType == null) {
            return batteryRepository.findById(5L)
                    .orElseThrow(() -> new RuntimeException("Null 배터리 타입을 찾을 수 없습니다."));
        }

        Battery battery = batteryRepository.findByBatteryType(batteryType);
        if (battery == null) {
            throw new RuntimeException("해당 배터리 타입을 찾을 수 없습니다: " + batteryType);
        }

        return battery;

    }

    // Hyper 테이블에서 사용할 ID 생성
    private Long getNextSequenceValue(String sequenceName) {

        String query = "SELECT nextval('" + sequenceName + "')";

        Object result = entityManager.createNativeQuery(query).getSingleResult();

        // 반환된 결과 타입에 따라 처리
        if (result instanceof Long) {
            return (Long) result;
        } else if (result instanceof BigInteger) {
            return ((BigInteger) result).longValue();
        } else if (result instanceof Number) {
            return ((Number) result).longValue();
        } else {
            // 다른 타입의 경우 문자열로 변환하여 Long으로 파싱
            return Long.parseLong(result.toString());
        }

    }

    // 회로 상태 ID 가져오기

    private Long getNextCircuitStatusId() {
        return getNextSequenceValue("circuit_status_id_seq");
    }

    // 배터리 상태 ID 가져오기
    private Long getNextBatteryStatusId() {
        return getNextSequenceValue("battery_status_id_seq");
    }

    // 모니터링 ID 가져오기
    private Long getNextMonitoringId() {
        return getNextSequenceValue("monitoring_id_seq");
    }

}
