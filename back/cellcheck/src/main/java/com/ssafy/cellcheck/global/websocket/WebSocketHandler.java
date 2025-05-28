package com.ssafy.cellcheck.global.websocket;

import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.io.IOException;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

@Component
@Slf4j
public class WebSocketHandler extends TextWebSocketHandler {
    // JSON 변환 처리 위한 Mapper
    private final ObjectMapper objectMapper = new ObjectMapper();

    private final Map<String, WebSocketSession> sessions = new ConcurrentHashMap<>();

    //연결 호출 메서드
    @Override
    public void afterConnectionEstablished(WebSocketSession session){
        // 세션 ID 가져오기
        String sessionId = session.getId();

        // 세션 앱 추가
        sessions.put(sessionId, session);
        log.info("웹소켓 연결 성공: {}", sessionId);
    }

    //메시지 수신 호출 메서드
    protected void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
        // 수신된 메시지 내용 가져오기
        String payload = message.getPayload();
        log.info("수신 메시지: {}", payload);

        // JSON 문자열을 Java 객체(Map)으로 변환
        Map<String, Object> jsonData = objectMapper.readValue(payload, Map.class);

        // 메시지 타입이 "analyze_images"인 경우 처리
        if ("analyze_images".equals(jsonData.get("type"))) {
            // 이미지 데이터 추출 (Base64로 인코딩된 이미지 배열)
            Object images = jsonData.get("images");

            // TODO: 여기에 실제 이미지 처리 및 분석 로직 구현
            // 현재는 더미 응답을 생성하여 반환
            Map<String, Object> response = Map.of(
                    "type", "analysis_result",
                    "status", "success",
                    "results", "분석 결과가 여기에 들어갑니다"
            );

            // 클라이언트에게 응답 메시지 전송
            sendMessage(session, response);
        }
    }

    // 연결 종료 시 호출
    @Override
    public void afterConnectionClosed(WebSocketSession session, org.springframework.web.socket.CloseStatus status) {
        // 세션 ID 가져오기
        String sessionId = session.getId();
        // 세션 맵에서 제거
        sessions.remove(sessionId);
        log.info("웹소켓 연결 종료: {}, 상태: {}", sessionId, status);
    }

    // 메시지 전송 메서드
    public void sendMessage(WebSocketSession session, Object message) throws IOException {
        // 객체를 JSON 문자열로 변환
        String jsonMessage = objectMapper.writeValueAsString(message);
        // 웹소켓 세션을 통해 메시지 전송
        session.sendMessage(new TextMessage(jsonMessage));
    }

    // 브로드 캐스트 메서드
    public void broadcastMessage(Object message) {
        try {
            // 객체를 JSON 문자열로 변환
            String jsonMessage = objectMapper.writeValueAsString(message);
            TextMessage textMessage = new TextMessage(jsonMessage);

            // 모든 연결된 세션에 메시지 전송
            for (WebSocketSession session : sessions.values()) {
                if (session.isOpen()) {
                    session.sendMessage(textMessage);
                }
            }
            log.info("브로드캐스트 메시지 전송 완료: {}", jsonMessage);
        } catch (IOException e) {
            log.error("메시지 브로드캐스트 중 오류 발생", e);
        }
    }

}
