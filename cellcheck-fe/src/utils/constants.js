// src/utils/constants.js

// 시스템 상태 코드
export const SYSTEM_STATUS = {
  ONLINE: 'ONLINE',
  OFFLINE: 'OFFLINE',
  MAINTENANCE: 'MAINTENANCE'
};

// 사용자 역할
export const USER_ROLES = {
  ADMIN: 'admin',
  USER: 'user'
};

// 알림 유형
export const NOTICE_TYPES = {
  INFO: 'info',
  WARNING: 'warning',
  ERROR: 'error',
  SUCCESS: 'success',
  MAINTENANCE: 0, // 점검
  URGENT: 1,      // 긴급
  NORMAL: 2       // 일반
};

// 애니메이션 지속 시간 (밀리초)
export const ANIMATION_DURATION = {
  SHORT: 500,
  MEDIUM: 1000,
  LONG: 2000
};

// 로컬 스토리지 키
export const STORAGE_KEYS = {
  TOKEN: 'token',
  USER: 'user',
  THEME: 'theme'
};

// API 엔드포인트 (실제 API가 구현된다면 사용)
export const API_ENDPOINTS = {
  // 인증 관련
  LOGIN_USER: '/api/auth/user-login',
  LOGIN_ADMIN: '/api/auth/admin-login',
  REGISTER_USER: '/api/auth/user-register',
  REGISTER_ADMIN: '/api/auth/admin-signup',
  LOGOUT: '/api/auth/logout',
  REFRESH_TOKEN: '/api/auth/refresh',
  USER_INFO: '/api/auth/user-info',
  ADMIN_USER_LIST: '/api/auth/admin/users',
  
  // 시스템 관련
  SYSTEM_STATUS: '/api/system/status',
  
  // 배터리 관련
  BATTERIES: '/api/batteries',
  BATTERY_DETAIL: (id) => `/api/batteries/${id}`,
  BATTERY_LOGS: '/api/battery/log',
  
  // 장치 관련
  DEVICES: '/api/devices',
  DEVICE_DETAIL: (id) => `/api/devices/${id}`,
  DEVICE_STATUS: (id) => `/api/devices/${id}/status`,
  
  // 공지사항 관련
  NOTICES: '/api/notice',
  CREATE_NOTICE: '/api/notice',
};

// 인증 관련 상수
export const AUTH_CONSTANTS = {
  ACCESS_TOKEN: 'accessToken',
  REFRESH_TOKEN: 'refreshToken',
  USER_INFO: 'user',
  IS_MENTOR: 'isMentor',
  TOKEN_EXPIRES: 'tokenExpires',
};

// HTTP 상태 코드
export const HTTP_STATUS = {
  OK: 200,
  CREATED: 201,
  BAD_REQUEST: 400,
  UNAUTHORIZED: 401,
  FORBIDDEN: 403,
  NOT_FOUND: 404,
  INTERNAL_SERVER_ERROR: 500,
};

// 에러 메시지
export const ERROR_MESSAGES = {
  DEFAULT: '오류가 발생했습니다. 다시 시도해주세요.',
  NETWORK_ERROR: '네트워크 연결 오류가 발생했습니다.',
  UNAUTHORIZED: '인증에 실패했습니다. 다시 로그인해주세요.',
  SESSION_EXPIRED: '세션이 만료되었습니다. 다시 로그인해주세요.',
  SERVER_ERROR: '서버 오류가 발생했습니다. 잠시 후 다시 시도해주세요.',
};

// 시스템 버전
export const SYSTEM_VERSION = 'v1.0.0';

// 테마 모드
export const THEME_MODES = {
  LIGHT: 'light',
  DARK: 'dark'
};

// 관리자 키 (실제 구현에서는 환경 변수로 관리하거나 서버에서 검증)
export const ADMIN_KEY = '1234';