package com.ssafy.cellcheck.global.auth.jwt;

import com.ssafy.cellcheck.global.constant.ErrorCode;
import com.ssafy.cellcheck.global.exception.CustomException;
import io.jsonwebtoken.ExpiredJwtException;
import io.jsonwebtoken.Jwt;
import io.jsonwebtoken.JwtException;
import io.jsonwebtoken.Jwts;
import jakarta.servlet.http.HttpServletRequest;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

import javax.crypto.SecretKey;
import javax.crypto.spec.SecretKeySpec;
import java.nio.charset.StandardCharsets;
import java.util.Date;

@Component
@Slf4j
public class JWTUtil {

    @Value("${jwt.expiration.access}")
    private Long accessTokenValidTime;

    @Value("${jwt.expiration.refresh}")
    private Long refreshTokenValidTime;

    private final SecretKey secretKey;

    public JWTUtil(@Value("${jwt.secret}") String secret) {
        this.secretKey = new SecretKeySpec(
                // 문자열 시크릿키 반환
                secret.getBytes(StandardCharsets.UTF_8),
                Jwts.SIG.HS256.key().build().getAlgorithm()
        );
    }

    //토큰 추출
    public String getJwtFromRequest(HttpServletRequest request) {
        String authorization = request.getHeader("Authorization");
        if (authorization == null || !authorization.startsWith("Bearer ")) {
            throw new CustomException(ErrorCode.INVALID_TOKEN_FORM);
        }
        String token = authorization.split(" ")[1];
        return token;

    }

    // 토큰에서 관리자 ID 추출
    public Long getAdminId(String token) {
        if (!isAdmin(token)) {
            log.warn("관리자 토큰이 아닙니다");
            throw new CustomException(ErrorCode.UNAUTHORIZED);
        }
        return getId(token);
    }

    // 토큰 만료 여부 확인
    public boolean isExpired(String token) {
        try {
            // 만료 시간과 현재 시간 비교
            Date expiration = Jwts.parser()
                    .verifyWith(secretKey)
                    .build()
                    .parseSignedClaims(token)
                    .getPayload()
                    .getExpiration();
            return expiration.before(new Date());
        } catch (ExpiredJwtException e) {
            // 토큰이 이미 만료된 경우
            throw new CustomException(ErrorCode.EXPIRED_ACCESS_TOKEN);
        } catch (JwtException e) {
            throw new CustomException(ErrorCode.UNAUTHORIZED);
        }
    }

    // access + refresh 토큰 생성
    public JWTToken createTokens(Long id, boolean isAdmin) {
        String accessToken = createToken(id, isAdmin, accessTokenValidTime);
        String refreshToken = createToken(id, isAdmin, refreshTokenValidTime);
        return new JWTToken("Bearer", accessToken, refreshToken);
    }

    // 토큰에서 ID 추출
    public Long getId(String token) {
        try {
            return Jwts.parser()
                    .verifyWith(secretKey)
                    .build()
                    .parseSignedClaims(token)
                    .getPayload()
                    .get("id", Long.class);
        } catch (JwtException | IllegalArgumentException e) {
            log.warn("유효하지 않은 토큰입니다: {}", e.getMessage());
            throw new CustomException(ErrorCode.UNAUTHORIZED);
        }
    }

    // 토큰에서 관리자 여부 추출
    public boolean isAdmin(String token) {
        try {
            return Jwts.parser()
                    .verifyWith(secretKey)
                    .build()
                    .parseSignedClaims(token)
                    .getPayload()
                    .get("isAdmin", Boolean.class);
        } catch (JwtException | IllegalArgumentException e) {
            log.warn("유효하지 않은 토큰입니다: {}", e.getMessage());
            throw new CustomException(ErrorCode.UNAUTHORIZED);
        }

    }



    // access 토큰만 생성
    public String createAccessToken(Long id, boolean isAdmin) {
        return createToken(id, isAdmin, accessTokenValidTime);
    }

    // refresh 토큰만 생성
    public String createRefreshToken(Long id, boolean isAdmin) {
        return createToken(id, isAdmin, refreshTokenValidTime);
    }

    // 토큰 생성 공통 메서드
    public String createToken(Long id, boolean isAdmin, Long expiredMS) {
        return Jwts.builder()
                .claim("id", id)
                .claim("isAdmin", isAdmin)
                .issuedAt(new Date(System.currentTimeMillis()))
                .expiration(new Date(System.currentTimeMillis() + expiredMS))
                .signWith(secretKey)
                .compact();
    }

    // access 토큰만 재발급
    public JWTToken refreshAccessTokenOnly(String refreshToken) {
        if (verifyRefreshToken(refreshToken)) {
            Long id = getId(refreshToken);
            boolean isAdmin = isAdmin(refreshToken);
            String newAccessToken = createAccessToken(id, isAdmin);
            return new JWTToken("Bearer", newAccessToken, refreshToken);
        }
        return null;
    }

    // refresh 토큰 유효성 검증
    public Boolean verifyRefreshToken(String token) {
        try {
            Jwts.parser().verifyWith(secretKey).build().parseSignedClaims(token);
            return true;
        } catch (JwtException e) {
            log.error("리프레시 토큰 검증 실패 : {}", e.getMessage());
            throw new CustomException(ErrorCode.INVALID_REFRESH_TOKEN);
        }
    }

}
