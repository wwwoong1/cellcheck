package com.ssafy.cellcheck.domain.mqtt.service;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.cellcheck.global.websocket.WebSocketHandler;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.context.annotation.Primary;
import org.springframework.stereotype.Service;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.Map;

@Slf4j
@Service
@Primary
@RequiredArgsConstructor
public class MqttServiceWebSocketAdapter implements MqttService {

    private final MqttServiceImpl mqttServiceImpl;
    private final WebSocketHandler webSocketHandler;
    private final ObjectMapper objectMapper = new ObjectMapper();
    private static final DateTimeFormatter DATE_FORMATTER = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss.SSS");

    @Override
    public void processAppearanceData(String payload) throws Exception {
        // 1. 먼저 WebSocket으로 전송
        try {
            JsonNode dataNode = objectMapper.readTree(payload);
            Map<String, Object> wsData = new HashMap<>();

            // 원본 데이터 포함
            objectMapper.readerForUpdating(wsData).readValue(dataNode);

            // 메타데이터 추가
            wsData.put("sent_at", LocalDateTime.now().format(DATE_FORMATTER));
            wsData.put("type", "appearance");

            // 웹소켓으로 전송
            webSocketHandler.broadcastMessage(wsData);
            log.info("외관 검사 데이터 웹소켓으로 전송 완료");
        } catch (Exception e) {
            log.error("외관 검사 데이터 웹소켓 전송 중 오류", e);
            // 웹소켓 전송 실패해도 DB 저장은 계속 진행
        }

        // 2. 그 다음 DB에 저장
        mqttServiceImpl.processAppearanceData(payload);
    }

    @Override
    public void processEnvironmentData(String payload) throws Exception {
        // 1. 먼저 WebSocket으로 전송
        try {
            JsonNode dataNode = objectMapper.readTree(payload);
            Map<String, Object> wsData = new HashMap<>();

            // 원본 데이터 포함
            objectMapper.readerForUpdating(wsData).readValue(dataNode);

            // 메타데이터 추가
            wsData.put("sent_at", LocalDateTime.now().format(DATE_FORMATTER));
            wsData.put("type", "environment");

            // 웹소켓으로 전송
            webSocketHandler.broadcastMessage(wsData);
            log.info("환경 모니터링 데이터 웹소켓으로 전송 완료");
        } catch (Exception e) {
            log.error("환경 모니터링 데이터 웹소켓 전송 중 오류", e);
        }

        // 2. 그 다음 DB에 저장
        mqttServiceImpl.processEnvironmentData(payload);
    }

    @Override
    public void processDischargeData(String payload) throws Exception {
        // 1. 먼저 WebSocket으로 전송
        try {
            JsonNode dataNode = objectMapper.readTree(payload);
            Map<String, Object> wsData = new HashMap<>();

            // 원본 데이터 포함
            objectMapper.readerForUpdating(wsData).readValue(dataNode);

            // 메타데이터 추가
            wsData.put("sent_at", LocalDateTime.now().format(DATE_FORMATTER));
            wsData.put("type", "discharge");

            // 웹소켓으로 전송
            webSocketHandler.broadcastMessage(wsData);
            log.info("방전 데이터 웹소켓으로 전송 완료");
        } catch (Exception e) {
            log.error("방전 데이터 웹소켓 전송 중 오류", e);
        }

        // 2. 그 다음 DB에 저장
        mqttServiceImpl.processDischargeData(payload);
    }

    @Override
    public void processSystemData(String payload) throws Exception {
        // 1. 먼저 WebSocket으로 전송
        try {
            JsonNode dataNode = objectMapper.readTree(payload);
            Map<String, Object> wsData = new HashMap<>();

            // 원본 데이터 포함
            objectMapper.readerForUpdating(wsData).readValue(dataNode);

            // 메타데이터 추가
            wsData.put("sent_at", LocalDateTime.now().format(DATE_FORMATTER));
            wsData.put("type", "system");

            // 웹소켓으로 전송
            webSocketHandler.broadcastMessage(wsData);
            log.info("시스템 데이터 웹소켓으로 전송 완료");
        } catch (Exception e) {
            log.error("시스템 데이터 웹소켓 전송 중 오류", e);
        }

        // 2. 그 다음 DB에 저장
        mqttServiceImpl.processSystemData(payload);
    }
}