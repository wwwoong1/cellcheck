package com.ssafy.cellcheck.global.config;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

import io.swagger.v3.oas.models.Components;
import io.swagger.v3.oas.models.OpenAPI;
import io.swagger.v3.oas.models.info.Info;
import io.swagger.v3.oas.models.security.SecurityRequirement;
import io.swagger.v3.oas.models.security.SecurityScheme;
import io.swagger.v3.oas.models.servers.Server;

@Configuration
public class SwaggerConfig {
    @Value("${url.https}")
    private String httpsServerUrl;

    @Bean
    public OpenAPI openAPI() {
        // API 정보
        Info info = new Info()
                .title("CellCheck API")
                .version("1.0")
                .description("CellCheck API 문서");
        SecurityScheme securityScheme = new SecurityScheme()
                .type(SecurityScheme.Type.HTTP)
                .scheme("bearer")
                .bearerFormat("JWT");

        // Security 요청 설정
        SecurityRequirement securityRequirement = new SecurityRequirement().addList("bearer-jwt");

        // 여러줄 형식 추가
        Map<String, Object> extensions = new HashMap<>();
        Map<String, Object> swaggerUiConfig = new HashMap<>();
        swaggerUiConfig.put("paragraphStyle", true);

        extensions.put("x-swagger-ui-config", swaggerUiConfig);

        return new OpenAPI()
                .info(info)
                .addSecurityItem(securityRequirement)
                .components(new Components().addSecuritySchemes("bearer-jwt", securityScheme))
                .extensions(extensions)
                .servers(List.of(
                        new Server().url("http://localhost:8080"),
                        new Server().url(httpsServerUrl).description("CellCheck API 서버")
                ));
    }

}
