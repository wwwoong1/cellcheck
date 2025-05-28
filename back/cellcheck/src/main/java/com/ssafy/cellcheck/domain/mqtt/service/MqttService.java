package com.ssafy.cellcheck.domain.mqtt.service;

public interface MqttService {

    // 외관 검사 처리
    void processAppearanceData(String payload) throws Exception;

    // 환경 모니터링 데이터 처리
    void processEnvironmentData(String payload) throws Exception;

    // 방전 데이터 처리
    void processDischargeData(String payload) throws Exception;

    // 시스템 상태 데이터 처리
    void processSystemData(String payload) throws Exception;


}
