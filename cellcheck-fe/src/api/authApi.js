// src/api/authApi.js
import { authAxios, publicAxios } from '../utils/AxiosInstance';
import { handleApiResponse, handleApiError } from '../utils/helpers';
import { API_ENDPOINTS, AUTH_CONSTANTS } from '../utils/constants';

/**
 * 인증 관련 API 함수들 - 실제 API 연결 구현
 */

// 관리자 회원가입
export const registerAdmin = async (loginId, password, adminName) => {
  try {
    const response = await publicAxios.post(API_ENDPOINTS.REGISTER_ADMIN, {
      loginId,
      password,
      adminName
    }, { 
      withCredentials: true 
    });
    
    // 응답이 boolean인 경우 직접 처리
    if (typeof response.data === 'boolean') {
      return {
        success: response.data,
        message: response.data ? '관리자 계정이 성공적으로 등록되었습니다.' : '회원가입에 실패했습니다. 다시 시도해주세요.',
        data: response.data
      };
    }
    
    return handleApiResponse(response);
  } catch (error) {
    console.error('관리자 회원가입 실패:', error.message);
    return handleApiError(error);
  }
};

// 관리자 로그인
export const loginAdmin = async (loginId, password, rememberMe = false) => {
  try {
    const response = await publicAxios.post(API_ENDPOINTS.LOGIN_ADMIN, {
      loginId,
      password
    }, {
      withCredentials: true
    });
    
    // 응답이 직접 객체인 경우(백엔드에서 AdminLoginResponse 반환)
    if (response.data && response.data.accessToken) {
      // 토큰 및 사용자 정보 저장
      localStorage.setItem(AUTH_CONSTANTS.ACCESS_TOKEN, response.data.accessToken);
      
      // rememberMe가 true인 경우에만 리프레시 토큰 저장
      if (rememberMe) {
        localStorage.setItem(AUTH_CONSTANTS.REFRESH_TOKEN, response.data.refreshToken);
      }
      
      localStorage.setItem(AUTH_CONSTANTS.USER_INFO, JSON.stringify({
        id: response.data.adminId,
        loginId: response.data.loginId,
        name: response.data.adminName,
        role: 'admin',
        isAdmin: true
      }));
      localStorage.setItem(AUTH_CONSTANTS.IS_MENTOR, 'true'); // 관리자는 멘토 권한도 있다고 가정
      
      return {
        success: true,
        message: '로그인에 성공했습니다.',
        data: response.data
      };
    }
    
    return handleApiResponse(response);
  } catch (error) {
    console.error('관리자 로그인 실패:', error.message);
    return handleApiError(error);
  }
};

// 사용자 등록 (관리자가 사용자 생성)
export const registerUser = async (loginId, password, userName, region) => {
  try {
    // 관리자 계정으로 로그인되어 있어야 사용자를 등록할 수 있음
    const response = await authAxios.post(API_ENDPOINTS.REGISTER_USER, {
      loginId,
      password,
      userName,
      region
    }, {
      withCredentials: true
    });
    
    // 응답이 boolean인 경우 직접 처리
    if (typeof response.data === 'boolean') {
      return {
        success: response.data,
        message: response.data ? '사용자가 성공적으로 등록되었습니다.' : '사용자 등록에 실패했습니다. 다시 시도해주세요.',
        data: response.data
      };
    }
    
    return handleApiResponse(response);
  } catch (error) {
    console.error('사용자 등록 실패:', error.message);
    return handleApiError(error);
  }
};

// 사용자 로그인
export const loginUser = async (loginId, password, rememberMe = false) => {
  try {
    const response = await publicAxios.post(API_ENDPOINTS.LOGIN_USER, {
      loginId,
      password
    }, {
      withCredentials: true
    });
    
    // 응답이 직접 객체인 경우(백엔드에서 UserLoginResponse 반환)
    if (response.data && response.data.accessToken) {
      // 토큰 및 사용자 정보 저장
      localStorage.setItem(AUTH_CONSTANTS.ACCESS_TOKEN, response.data.accessToken);
      
      // rememberMe가 true인 경우에만 리프레시 토큰 저장
      if (rememberMe) {
        localStorage.setItem(AUTH_CONSTANTS.REFRESH_TOKEN, response.data.refreshToken);
      }
      
      localStorage.setItem(AUTH_CONSTANTS.USER_INFO, JSON.stringify({
        id: response.data.userId,
        loginId: response.data.loginId,
        name: response.data.userName,
        region: response.data.region,
        role: 'user',
        isAdmin: false
      }));
      localStorage.setItem(AUTH_CONSTANTS.IS_MENTOR, 'false'); // 일반 사용자는 멘토 권한이 없다고 가정
      
      return {
        success: true,
        message: '로그인에 성공했습니다.',
        data: response.data
      };
    }
    
    return handleApiResponse(response);
  } catch (error) {
    console.error('사용자 로그인 실패:', error.message);
    return handleApiError(error);
  }
};

// 로그아웃
export const logoutUser = async () => {
  try {
    const response = await authAxios.post(API_ENDPOINTS.LOGOUT, {}, {
      withCredentials: true
    });
    
    // 로그아웃 처리 (로컬 스토리지 정리)
    clearUserSession();
    
    // 백엔드는 void를 반환하지만 프론트엔드는 성공 여부를 반환
    return {
      success: true,
      message: '로그아웃 되었습니다.',
      data: null
    };
  } catch (error) {
    console.error('로그아웃 요청 실패:', error.message);
    
    // 로그아웃 요청이 실패해도 로컬 스토리지 정리
    clearUserSession();
    
    // 로그아웃은 항상 성공으로 처리 (서버 오류와 상관없이)
    return {
      success: true,
      message: '로그아웃 되었습니다.',
      data: null
    };
  }
};

// 로컬 스토리지에서 사용자 세션 정보 제거
const clearUserSession = () => {
  localStorage.removeItem(AUTH_CONSTANTS.ACCESS_TOKEN);
  localStorage.removeItem(AUTH_CONSTANTS.REFRESH_TOKEN);
  localStorage.removeItem(AUTH_CONSTANTS.USER_INFO);
  localStorage.removeItem(AUTH_CONSTANTS.TOKEN_EXPIRES);
  localStorage.removeItem(AUTH_CONSTANTS.IS_MENTOR);
};

// 토큰 갱신
export const refreshUserToken = async () => {
  try {
    const refreshToken = localStorage.getItem(AUTH_CONSTANTS.REFRESH_TOKEN);
    if (!refreshToken) {
      throw new Error('Refresh token not found');
    }
    
    const response = await publicAxios.post(API_ENDPOINTS.REFRESH_TOKEN, {
      refreshToken: refreshToken
    });
    
    // 응답이 직접 객체인 경우(백엔드 RefreshTokenResponseDto)
    if (response.data && response.data.accessToken) {
      // 새 토큰 저장
      localStorage.setItem(AUTH_CONSTANTS.ACCESS_TOKEN, response.data.accessToken);
      localStorage.setItem(AUTH_CONSTANTS.REFRESH_TOKEN, response.data.refreshToken);
      
      // 사용자 정보 업데이트
      const userObj = getCurrentUser();
      if (userObj) {
        // 관리자인지 일반 사용자인지에 따라 다른 필드 업데이트
        userObj.id = response.data.id;
        userObj.name = response.data.name;
        if (!response.data.isAdmin && response.data.region) {
          userObj.region = response.data.region;
        }
        
        localStorage.setItem(AUTH_CONSTANTS.USER_INFO, JSON.stringify(userObj));
      }
      
      return {
        success: true,
        message: '토큰이 성공적으로 갱신되었습니다.',
        data: response.data
      };
    }
    
    return handleApiResponse(response);
  } catch (error) {
    // 토큰 갱신 실패 시 로그아웃 처리
    clearUserSession();
    
    if (window.location.pathname !== '/login' && window.location.pathname !== '/') {
      window.location.href = '/?reason=session_expired';
    }
    
    return handleApiError(error);
  }
};

// 현재 인증된 사용자 정보 가져오기
export const getCurrentUser = () => {
  const userStr = localStorage.getItem(AUTH_CONSTANTS.USER_INFO);
  if (userStr) {
    try {
      return JSON.parse(userStr);
    } catch (error) {
      console.error('사용자 정보 파싱 오류:', error.message);
      return null;
    }
  }
  return null;
};

// 인증 상태 확인
export const isAuthenticated = () => {
  return localStorage.getItem(AUTH_CONSTANTS.ACCESS_TOKEN) !== null && getCurrentUser() !== null;
};

// 관리자 권한 확인
export const isAdmin = () => {
  const user = getCurrentUser();
  return user && user.isAdmin === true;
};

// 시스템 상태 가져오기 (실제 API가 없는 경우 더미 데이터 유지)
export const getSystemStatus = async () => {
  try {
    // 실제 API 엔드포인트가 있다면 아래 코드 사용
    // const response = await authAxios.get(API_ENDPOINTS.SYSTEM_STATUS);
    // return handleApiResponse(response);
    
    // 임시 더미 데이터
    await new Promise(resolve => setTimeout(resolve, 200));
    return {
      success: true,
      data: {
        serverStatus: 'ONLINE',
        deviceCount: 42,
        version: 'v1.0.0'
      }
    };
  } catch (error) {
    return handleApiError(error);
  }
};

// 관리자 공지사항 가져오기 API
export const getAdminNotices = async () => {
  await delay(300); // 가짜 네트워크 지연
  
  return [
    {
      id: 1,
      type: 'warning',
      message: '시스템 보안 업데이트 적용 예정 (05/20)',
      date: '2025-05-15'
    },
    {
      id: 2,
      type: 'info',
      message: '백업 서버 점검 완료',
      date: '2025-05-13'
    }
  ];
};

// 사용자 정보 조회 (전체 사용자 조회)
export const getUserInfo = async () => {
  try {
    const response = await authAxios.get(API_ENDPOINTS.USER_INFO);
    return handleApiResponse(response);
  } catch (error) {
    console.error('사용자 정보 조회 실패:', error.message);
    return handleApiError(error);
  }
};