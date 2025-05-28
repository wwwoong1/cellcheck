package com.ssafy.cellcheck.global.mqtt;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.cellcheck.domain.mqtt.service.MqttService;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.messaging.Message;
import org.springframework.messaging.MessageHandler;
import org.springframework.messaging.MessagingException;
import org.springframework.stereotype.Component;

// 브로커에서 수신한 메시지 처리
@Slf4j
@Component
public class MqttMessageHandler implements MessageHandler {

    // JSON 데이터 변환
    private final ObjectMapper objectMapper = new ObjectMapper();

    @Autowired
    private MqttService mqttService;

    // 메세지 수신후 서비스로 전달
    @Override
    public void handleMessage(Message<?> message) throws MessagingException{
        try {
            // 토픽 추출
            String topic = (String) message.getHeaders().get("mqtt_receivedTopic");

            String payload;
            Object rawPayload = message.getPayload();

            if (rawPayload instanceof byte[]) {
                payload = new String((byte[]) rawPayload);
            } else {
                payload = rawPayload.toString();
            }

            // 수신된 메시지 로깅
            log.info("MQTT 메시지 수신 - 토픽: {}, 페이로드: {}", topic, payload);

            // WS 처리 코드 (이후 추가)

            // 적절한 서비스로 메시지 전달
            processMessage(topic, payload);

        } catch (Exception e) {
            log.error("MQTT 메시지 처리 중 오류 발생", e);
            throw new MessagingException("MQTT 메시지 처리 실패 : " + e.getMessage(), e);
        }

    }

    private void processMessage(String topic, String payload) throws Exception {

        if (topic.endsWith("/appearance")) {
            mqttService.processAppearanceData(payload);
        } else if (topic.endsWith("/environment")) {
            mqttService.processEnvironmentData(payload);
        } else if (topic.endsWith("/discharge")) {
            mqttService.processDischargeData(payload);
        } else if (topic.endsWith("/system")) {
            mqttService.processSystemData(payload);
        } else {
            log.warn("처리되지 않은 토픽: {}", topic);
        }
    }

}
