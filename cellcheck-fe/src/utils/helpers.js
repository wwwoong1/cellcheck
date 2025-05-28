// src/utils/helpers.js

// 날짜 포맷팅 함수
export const formatDate = (date, formatType = 'full') => {
  if (!date) return '';
  
  const dateObj = new Date(date);
  
  switch (formatType) {
    case 'full':
      return dateObj.toLocaleString();
    case 'date':
      return dateObj.toLocaleDateString();
    case 'time':
      return dateObj.toLocaleTimeString();
    case 'short':
      return `${dateObj.getFullYear()}-${
        String(dateObj.getMonth() + 1).padStart(2, '0')
      }-${
        String(dateObj.getDate()).padStart(2, '0')
      }`;
    default:
      return dateObj.toLocaleString();
  }
};

// 로컬 스토리지 헬퍼 함수
export const storage = {
  get: (key) => {
    try {
      const value = localStorage.getItem(key);
      return value ? JSON.parse(value) : null;
    } catch (error) {
      console.error(`Error getting item from localStorage: ${error}`);
      return null;
    }
  },
  
  set: (key, value) => {
    try {
      localStorage.setItem(key, JSON.stringify(value));
      return true;
    } catch (error) {
      console.error(`Error setting item in localStorage: ${error}`);
      return false;
    }
  },
  
  remove: (key) => {
    try {
      localStorage.removeItem(key);
      return true;
    } catch (error) {
      console.error(`Error removing item from localStorage: ${error}`);
      return false;
    }
  },
  
  clear: () => {
    try {
      localStorage.clear();
      return true;
    } catch (error) {
      console.error(`Error clearing localStorage: ${error}`);
      return false;
    }
  }
};

// 디바이스 상태 아이콘 결정
export const getStatusIcon = (status, isAdminMode = false) => {
  switch (status) {
    case 'ONLINE':
      return isAdminMode ? '🟢' : '🟢';
    case 'OFFLINE':
      return isAdminMode ? '🔴' : '🔴';
    case 'MAINTENANCE':
      return isAdminMode ? '🟠' : '🟠';
    default:
      return isAdminMode ? '⚪' : '⚪';
  }
};

// 알림 유형 아이콘 결정
export const getNoticeTypeIcon = (type) => {
  switch (type) {
    case 'warning':
      return '⚠️';
    case 'error':
      return '❌';
    case 'success':
      return '✅';
    case 'info':
    default:
      return 'ℹ️';
  }
};

// 키보드 단축키 이벤트 핸들러
export const setupKeyboardShortcuts = (handlers) => {
  const handleKeyDown = (event) => {
    const { key, ctrlKey, shiftKey, altKey } = event;
    
    // Ctrl+Shift+F : 관리자 모드 단축키
    if (ctrlKey && shiftKey && key === 'F' && handlers.adminMode) {
      event.preventDefault();
      handlers.adminMode();
    }
    
    // ESC : 모달 닫기 단축키
    if (key === 'Escape' && handlers.closeModal) {
      handlers.closeModal();
    }
  };
  
  window.addEventListener('keydown', handleKeyDown);
  
  // 클린업 함수 반환
  return () => {
    window.removeEventListener('keydown', handleKeyDown);
  };
};

/**
 * API 응답 처리 헬퍼 함수
 * @param {Object} response - axios 응답 객체
 * @returns {Object} 표준화된 응답 객체
 */
export const handleApiResponse = (response) => {
  // 응답이 없는 경우 에러 반환
  if (!response) {
    console.error('API 응답이 없음');
    return {
      success: false,
      message: '서버로부터 응답이 없습니다.',
      data: null
    };
  }
  
  console.log('API 응답 구조:', {
    responseType: typeof response,
    hasData: !!response.data,
    dataType: response.data ? typeof response.data : 'N/A',
    status: response.status,
    statusText: response.statusText,
    hasSuccess: 'success' in response,
    successValue: response.success,
    hasMessage: 'message' in response,
    messageValue: response.message,
    dataHasSuccess: response.data && 'success' in response.data,
    dataSuccessValue: response.data && response.data.success,
    dataHasMessage: response.data && 'message' in response.data,
    dataMessageValue: response.data && response.data.message,
  });
  
  // 응답 자체가 boolean 타입인 경우 처리
  if (typeof response === 'boolean') {
    return {
      success: response,
      message: response ? '요청이 성공적으로 처리되었습니다.' : '요청을 처리하는 중 오류가 발생했습니다.',
      data: response
    };
  }
  
  // 응답 데이터가 boolean 타입인 경우 처리
  if (typeof response.data === 'boolean') {
    return {
      success: response.data,
      message: response.data ? '요청이 성공적으로 처리되었습니다.' : '요청을 처리하는 중 오류가 발생했습니다.',
      data: response.data
    };
  }
  
  // 2xx 상태 코드가 있으면 성공으로 간주
  if (response.status && response.status >= 200 && response.status < 300) {
    return {
      success: true,
      message: response.message || (response.data && response.data.message) || '요청이 성공적으로 처리되었습니다.',
      data: response.data || response
    };
  }
  
  // 기존 로직 - API 응답 형식에 따라 표준화
  const success = response.success || (response.data && response.data.success);
  const message = response.message || (response.data && response.data.message) || '';
  const data = response.data || response;
  
  return {
    success: !!success,
    message,
    data
  };
};

/**
 * API 에러 처리 헬퍼 함수
 * @param {Error} error - axios 에러 객체
 * @returns {Object} 표준화된 에러 객체
 */
export const handleApiError = (error) => {
  // 네트워크 에러
  if (!error.response) {
    return {
      success: false,
      status: 0,
      message: '네트워크 연결 오류가 발생했습니다.'
    };
  }
  
  // 서버 에러
  const status = error.response.status;
  let message = error.response.data?.message || '오류가 발생했습니다.';
  
  // 상태 코드별 메시지 처리
  if (status === 401) {
    message = '인증에 실패했습니다. 다시 로그인해주세요.';
  } else if (status === 403) {
    message = '접근 권한이 없습니다.';
  } else if (status === 404) {
    message = '요청한 리소스를 찾을 수 없습니다.';
  } else if (status >= 500) {
    message = '서버 오류가 발생했습니다. 잠시 후 다시 시도해주세요.';
  }
  
  return {
    success: false,
    status,
    message
  };
};

/**
 * 토큰 만료 시간 계산
 * @param {string} token - JWT 토큰
 * @returns {number} 만료 시간 (밀리초)
 */
export const getTokenExpiration = (token) => {
  if (!token) return 0;
  
  try {
    // JWT 토큰의 페이로드 부분 디코딩
    const base64Url = token.split('.')[1];
    const base64 = base64Url.replace(/-/g, '+').replace(/_/g, '/');
    const payload = JSON.parse(window.atob(base64));
    
    // exp 필드가 있는지 확인
    if (payload.exp) {
      // JWT의 exp는 초 단위, JavaScript는 밀리초 단위
      return payload.exp * 1000;
    }
  } catch (error) {
    console.error('토큰 파싱 오류:', error);
  }
  
  return 0;
};