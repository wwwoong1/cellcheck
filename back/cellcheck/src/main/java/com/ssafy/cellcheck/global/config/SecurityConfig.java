package com.ssafy.cellcheck.global.config;

import com.ssafy.cellcheck.domain.auth.service.RefreshTokenService;
import com.ssafy.cellcheck.global.auth.jwt.JWTFilter;
import com.ssafy.cellcheck.global.auth.jwt.JWTUtil;
import jakarta.servlet.http.HttpServletRequest;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.core.annotation.Order;
import org.springframework.security.config.Customizer;
import org.springframework.security.config.annotation.web.builders.HttpSecurity;
import org.springframework.security.config.annotation.web.configurers.AbstractHttpConfigurer;
import org.springframework.security.config.http.SessionCreationPolicy;
import org.springframework.security.core.userdetails.User;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.crypto.factory.PasswordEncoderFactories;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.security.provisioning.InMemoryUserDetailsManager;
import org.springframework.security.web.SecurityFilterChain;
import org.springframework.security.web.authentication.UsernamePasswordAuthenticationFilter;
import org.springframework.web.cors.CorsConfiguration;
import org.springframework.web.cors.CorsConfigurationSource;

import java.util.Collections;
import java.util.List;

@Configuration
public class SecurityConfig {

    private final JWTUtil jwtUtil;

    @Value("${url.http}")
    private String httpServerUrl;

    @Value("${url.https}")
    private String httpsServerUrl;

    @Value("${swagger.auth.username}")
    private String swaggerUsername;

    @Value("${swagger.auth.password}")
    private String swaggerPassword;

    public SecurityConfig(JWTUtil jwtUtil) {
        this.jwtUtil = jwtUtil;
    }

    @Bean
    @Order(1)
    SecurityFilterChain swaggerSecurityFilterChain(HttpSecurity http) throws Exception {    http
            .securityMatcher("/swagger-ui/**", "/swagger-ui.html", "/swagger-resources/**",
                    "/v3/api-docs", "/v3/api-docs/**", "/webjars/**")
            .authorizeHttpRequests(authorize -> authorize
                    .anyRequest().authenticated()
            )
            .httpBasic(Customizer.withDefaults())
            .csrf(AbstractHttpConfigurer::disable)
            .sessionManagement(sessionConfig ->
                    sessionConfig.sessionCreationPolicy(SessionCreationPolicy.STATELESS));

        return http.build();
    }


    @Bean
    SecurityFilterChain defaultSecurityFilterChain(HttpSecurity http, RefreshTokenService refreshTokenService) throws Exception {
        http
                // 세션 사용 X (JWT 기반 인증 사용)
                .sessionManagement(sessionConfig -> sessionConfig.sessionCreationPolicy(SessionCreationPolicy.STATELESS))
                // CORS 설정
                .cors(corsConfig -> corsConfig.configurationSource(new CorsConfigurationSource() {
                    @Override
                    public CorsConfiguration getCorsConfiguration(HttpServletRequest request) {
                        CorsConfiguration config = new CorsConfiguration();
                        config.setAllowedOriginPatterns(List.of(
                                "http://localhost:5173",
                                "http://localhost:8000",
                                "http://localhost:8080",
                                httpServerUrl,
                                httpsServerUrl
                        ));
                        config.setAllowedMethods(Collections.singletonList("*"));
                        config.setAllowedHeaders(Collections.singletonList("*"));
                        config.setMaxAge(3600L);
                        config.setAllowCredentials(true);
                        config.setExposedHeaders(Collections.singletonList("Authorization"));
                        return config;
                    }
                }))
                // CSRF 비활성화
                .csrf(AbstractHttpConfigurer::disable)
                // 경로별 접근권한
                .authorizeHttpRequests((requests) -> requests
                        .requestMatchers("/api/auth/**").permitAll() // 인증 관련 경로는 모두 허용
                        .requestMatchers("/error").permitAll() // 오류 페이지 허용
                        // WebSocket 경로 허용
                        .requestMatchers("/api/ws/**").permitAll()
                        // 그 외 모든 요청은 인증 필요
                        .anyRequest().authenticated())
                // JWT 필터 추가
                .addFilterBefore(new JWTFilter(jwtUtil, refreshTokenService), UsernamePasswordAuthenticationFilter.class);

        return http.build();
    }

    // Swagger 접근용 사용자 설정
    @Bean
    public InMemoryUserDetailsManager userDetailsService() {
        UserDetails swaggerUser = User.builder()
                .username(swaggerUsername)
                .password(passwordEncoder().encode(swaggerPassword))
                .roles("SWAGGER_ADMIN")
                .build();
        return new InMemoryUserDetailsManager(swaggerUser);
    }

    @Bean
    public PasswordEncoder passwordEncoder() {
        return PasswordEncoderFactories.createDelegatingPasswordEncoder();

    }
}
