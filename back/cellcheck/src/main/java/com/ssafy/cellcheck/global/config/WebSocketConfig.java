package com.ssafy.cellcheck.global.config;

import com.ssafy.cellcheck.global.websocket.WebSocketHandler;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;


@Configuration
@EnableWebSocket
public class WebSocketConfig implements WebSocketConfigurer {

    private WebSocketHandler webSocketHandler;

    @Value("${url.http}")
    private String httpServerUrl;

    @Value("${url.https}")
    private String httpsServerUrl;

    public WebSocketConfig(WebSocketHandler webSocketHandler) {
        this.webSocketHandler = webSocketHandler;
    }

    @Override
    public void registerWebSocketHandlers(WebSocketHandlerRegistry registry) {
        registry.addHandler(webSocketHandler, "/api/ws/analyze", "/api/ws/mqtt")
                .setAllowedOrigins(
                        "http://localhost:5173",
                        "http://localhost:8000",
                        "http://localhost:8080",
                        httpServerUrl,
                        httpsServerUrl
                )
                .setAllowedOriginPatterns("*");
    }
}
