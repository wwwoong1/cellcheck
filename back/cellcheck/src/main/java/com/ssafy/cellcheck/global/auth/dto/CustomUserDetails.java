package com.ssafy.cellcheck.global.auth.dto;

import ch.qos.logback.core.util.StringUtil;
import org.springframework.security.core.GrantedAuthority;
import org.springframework.security.core.authority.SimpleGrantedAuthority;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.util.StringUtils;

import java.util.ArrayList;
import java.util.Collection;

public class CustomUserDetails implements UserDetails {

    private Long id; // 식별자(관리자 사용자)
    private boolean isAdmin; // 관리자 사용자 구분

    // 권한 목록
    private Collection<GrantedAuthority> authorities;

    public CustomUserDetails(Long id, boolean isAdmin) {
        this.id = id;
        this.isAdmin = isAdmin;
        this.authorities = new ArrayList<>();

        // 권한 추가
        if (isAdmin) {
            this.authorities.add(new SimpleGrantedAuthority("ROLE_ADMIN"));
        } else {
            this.authorities.add(new SimpleGrantedAuthority("ROLE_USER"));
        }

    }

    public Long getId() {
        return id;
    }

    public boolean isAdmin() { return isAdmin; }

    // 동적 유틸리티 메서드
    public Collection<GrantedAuthority> createAuthorities(String roles) {
        Collection<GrantedAuthority> authorities = new ArrayList<>();
        for (String role : roles.split(",")) {
            if (!StringUtils.hasText(role)) continue;
            authorities.add(new SimpleGrantedAuthority(role));
        }

        return authorities;
    }

    @Override
    public Collection<GrantedAuthority> getAuthorities() {
        return authorities;
    }

    @Override
    public String getPassword() {
        return null; //JWT 기반으로 대체
    }

    @Override
    public String getUsername() {
        return String.valueOf(id); // 문자열 반환
    }

    @Override
    public boolean isAccountNonExpired() {
        return true;
    }

    @Override
    public boolean isAccountNonLocked() {
        return true;
    }

    @Override
    public boolean isCredentialsNonExpired() {
        return true;
    }

    @Override
    public boolean isEnabled() {
        return true;
    }
}
