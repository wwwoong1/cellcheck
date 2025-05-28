import axios from 'axios';

// 기본 API URL 설정 (환경변수에서 가져오기)
// 상대 경로를 사용하여 CORS 및 프록시 관련 문제 해결
const BASE_URL = ''; // 프록시를 사용하기 위해 baseURL을 비워둡니다

// 인증이 필요하지 않은 요청을 위한 Axios 인스턴스
export const publicAxios = axios.create({
  baseURL: BASE_URL,
  headers: {
    'Content-Type': 'application/json'
  },
  withCredentials: true, // CORS 자격 증명 허용
  timeout: 10000 // 10초
});

// 인증이 필요한 요청을 위한 Axios 인스턴스
export const authAxios = axios.create({
  baseURL: BASE_URL,
  headers: {
    'Content-Type': 'application/json'
  },
  withCredentials: true, // CORS 자격 증명 허용
  timeout: 10000 // 10초
});

// 요청 인터셉터 추가
publicAxios.interceptors.request.use(
  config => {
    // 개발 환경에서만 로깅
    if (import.meta.env.DEV) {
      console.log(`🚀 ${config.method?.toUpperCase()} 요청: ${config.url}`);
    }
    return config;
  },
  error => {
    console.error('❌ 요청 오류:', error.message);
    return Promise.reject(error);
  }
);

// 응답 인터셉터 추가
publicAxios.interceptors.response.use(
  response => {
    // 개발 환경에서만 로깅
    if (import.meta.env.DEV) {
      console.log(`✅ ${response.status} 응답: ${response.config.url}`);
    }
    return response;
  },
  error => {
    console.error('❌ 응답 오류:', error.message);
    if (error.response) {
      console.error(`📋 상태 코드: ${error.response.status}`);
    }
    return Promise.reject(error);
  }
);

// 동시에 여러 요청이 토큰을 갱신하는 것을 방지하기 위한 변수
let isRefreshing = false;
let refreshSubscribers = []; // 토큰 갱신 대기 중인 요청을 저장하는 배열
let refreshAttempts = 0; // 토큰 갱신 시도 횟수
const MAX_REFRESH_ATTEMPTS = 3; // 최대 갱신 시도 횟수

// 토큰이 갱신되면 대기 중인 요청들을 다시 시도하는 함수
const onRefreshed = (token) => {
  console.log(`🔄 토큰 갱신 완료 - 대기 중인 요청 ${refreshSubscribers.length}개 처리 중`);
  refreshSubscribers.forEach(callback => callback(token));
  refreshSubscribers = [];
};

// 토큰 갱신 중에 새로운 요청이 들어오면 대기열에 추가하는 함수
const addRefreshSubscriber = (callback) => {
  console.log('🔄 새로운 요청을 토큰 갱신 대기열에 추가');
  refreshSubscribers.push(callback);
};

// 토큰 갱신 상태 초기화
const resetRefreshState = () => {
  console.log('🔄 토큰 갱신 상태 초기화');
  isRefreshing = false;
  refreshAttempts = 0;
  refreshSubscribers = [];
};

// 토큰 상태 디버깅 함수
const logTokenState = () => {
  const accessToken = localStorage.getItem('accessToken');
  const refreshToken = localStorage.getItem('refreshToken');
  console.log(`🔐 토큰 상태 확인:
  - 액세스 토큰 존재: ${!!accessToken}
  - 리프레시 토큰 존재: ${!!refreshToken}
  - 갱신 시도 횟수: ${refreshAttempts}/${MAX_REFRESH_ATTEMPTS}
  - 현재 갱신 중: ${isRefreshing}
  - 대기 중인 요청 수: ${refreshSubscribers.length}`);

  // JWT 토큰 만료 시간 확인
  if (accessToken) {
    try {
      const payload = JSON.parse(atob(accessToken.split('.')[1]));
      const expiryDate = new Date(payload.exp * 1000);
      const now = new Date();
      const timeLeft = Math.floor((expiryDate - now) / 1000);
      
      console.log(`🕒 액세스 토큰 만료 정보:
      - 만료 시간: ${expiryDate.toLocaleString()}
      - 현재 시간: ${now.toLocaleString()}
      - 남은 시간: ${timeLeft}초 (${Math.floor(timeLeft / 60)}분 ${timeLeft % 60}초)`);
      
      if (timeLeft < 0) {
        console.warn('⚠️ 액세스 토큰이 이미 만료되었습니다!');
      } else if (timeLeft < 60) {
        console.warn(`⚠️ 액세스 토큰이 곧 만료됩니다 (${timeLeft}초 남음)`);
      }
    } catch (e) {
      console.error('💥 토큰 디코딩 중 오류 발생:', e);
    }
  }
};

// 요청 인터셉터 추가 - 모든 인증 요청에 토큰 첨부
authAxios.interceptors.request.use(
  config => {
    const accessToken = localStorage.getItem('accessToken');
    
    // 현재 페이지 경로 로깅
    const currentPath = window.location.pathname;
    console.log(`🔒 인증 요청 (${currentPath} 페이지에서) - ${config.method?.toUpperCase()}: ${config.url}`);
    
    if (accessToken) {
      config.headers['Authorization'] = `Bearer ${accessToken}`;
      
      // 토큰 상태 로그
      if (import.meta.env.DEV) {
        try {
          const payload = JSON.parse(atob(accessToken.split('.')[1]));
          const expiryDate = new Date(payload.exp * 1000);
          const now = new Date();
          const timeLeft = Math.floor((expiryDate - now) / 1000);
          
          if (timeLeft < 300) { // 5분 미만으로 남았을 때 경고
            console.warn(`⚠️ 토큰 만료 임박: ${timeLeft}초 남음 (요청: ${config.url})`);
          }
        } catch (e) {
          console.error('💥 토큰 파싱 중 오류:', e);
        }
      }
    } else {
      console.warn('⚠️ 인증 토큰 없음 - 인증이 필요한 요청:', config.url);
    }
    
    return config;
  },
  error => {
    console.error('❌ 인증 요청 전송 오류:', error);
    return Promise.reject(error);
  }
);

// 응답 인터셉터 추가
authAxios.interceptors.response.use(
  response => {
    // 성공적인 응답인 경우 갱신 시도 횟수 초기화
    refreshAttempts = 0;
    console.log(`✅ 인증 요청 성공 (${window.location.pathname}): ${response.config.method?.toUpperCase()} ${response.config.url}`);
    return response;
  },
  async error => {
    const originalRequest = error.config;
    const currentPath = window.location.pathname;
    
    console.error(`❌ 인증 요청 실패 (${currentPath}): ${originalRequest?.method?.toUpperCase()} ${originalRequest?.url}`);
    
    if (error.response) {
      console.error(`📋 오류 상태 코드: ${error.response.status}, 메시지: ${error.response.data?.message || '응답 메시지 없음'}`);
    }
    
    // 토큰 만료 에러(401)이고 재시도되지 않은 요청인 경우
    if (error.response?.status === 401 && !originalRequest._retry) {
      originalRequest._retry = true;
      console.log(`🔄 토큰 만료 감지: 페이지 ${currentPath}에서 ${originalRequest.url} 요청 중 401 오류 발생`);
      logTokenState(); // 현재 토큰 상태 출력
      
      try {
        // 최대 갱신 시도 횟수를 넘으면 로그아웃
        if (refreshAttempts >= MAX_REFRESH_ATTEMPTS) {
          console.error(`⛔ 토큰 갱신 시도 횟수(${MAX_REFRESH_ATTEMPTS})를 초과했습니다. 로그아웃 처리합니다.`);
          console.error(`⛔ 마지막 실패 요청: ${originalRequest.url}, 현재 페이지: ${currentPath}`);
          localStorage.clear();
          window.location.href = '/login?reason=max_refresh_attempts';
          return Promise.reject(error);
        }
        
        // 이미 토큰 갱신 중이면 현재 요청을 대기열에 추가
        if (isRefreshing) {
          console.log(`🔄 토큰 갱신 이미 진행 중 - 요청(${originalRequest.url})을 대기열에 추가`);
          return new Promise((resolve) => {
            addRefreshSubscriber((token) => {
              originalRequest.headers['Authorization'] = `Bearer ${token}`;
              console.log(`🔄 갱신된 토큰으로 대기 중이던 요청 재시도: ${originalRequest.url}`);
              resolve(authAxios(originalRequest));
            });
          });
        }
        
        isRefreshing = true;
        refreshAttempts++;
        console.log(`🔄 토큰 갱신 시도 ${refreshAttempts}/${MAX_REFRESH_ATTEMPTS} - 페이지: ${currentPath}`);
        
        // 리프레시 토큰 가져오기
        const refreshToken = localStorage.getItem('refreshToken');
        if (!refreshToken) {
          // 리프레시 토큰이 없으면 로그인 페이지로 이동
          console.warn('⚠️ 리프레시 토큰이 없습니다. 로그인이 필요합니다. 페이지:', currentPath);
          localStorage.clear();
          
          // 현재 로그인 페이지나 홈 페이지가 아닌 경우에만 리디렉션
          if (currentPath !== '/login' && currentPath !== '/') {
            window.location.href = '/login?reason=no_refresh_token';
          }
          
          isRefreshing = false;
          return Promise.reject(error);
        }
        
        console.log('🔄 토큰 갱신 API 호출 시작');
        // 토큰 갱신 요청 - 리프레시 토큰을 요청 본문에 포함
        const response = await publicAxios.post('/api/auth/refresh', {
          refreshToken: refreshToken
        });
        
        console.log('🔄 토큰 갱신 API 응답 수신:', response.status);
        
        if (response.data && response.data.accessToken) {
          // 새 토큰 저장
          localStorage.setItem('accessToken', response.data.accessToken);
          console.log('✅ 새 액세스 토큰 저장 완료');
          
          if (response.data.refreshToken) {
            localStorage.setItem('refreshToken', response.data.refreshToken);
            console.log('✅ 새 리프레시 토큰 저장 완료');
          }
          
          // 헤더 업데이트
          originalRequest.headers['Authorization'] = `Bearer ${response.data.accessToken}`;
          
          // 토큰 상태 확인
          logTokenState();
          
          // 대기 중인 요청들을 처리
          onRefreshed(response.data.accessToken);
          
          isRefreshing = false;
          return authAxios(originalRequest);
        } else {
          // 갱신 실패 시 로그아웃 처리
          console.error('❌ 토큰 갱신 실패: 응답 형식이 올바르지 않습니다.');
          console.error('❌ 응답 데이터:', response.data);
          
          // 최대 시도 횟수에 도달했을 때만 로그아웃 처리
          if (refreshAttempts >= MAX_REFRESH_ATTEMPTS) {
            localStorage.clear();
            console.error(`⛔ 토큰 갱신 최대 시도 횟수 도달 - 현재 페이지: ${currentPath}`);
            
            // 현재 로그인 페이지나 홈 페이지가 아닌 경우에만 리디렉션
            if (currentPath !== '/login' && currentPath !== '/') {
              window.location.href = '/login?reason=token_refresh_failed';
            }
          }
          
          isRefreshing = false;
          return Promise.reject(error);
        }
      } catch (refreshError) {
        // 갱신 중 에러 발생 시
        console.error('❌ 토큰 갱신 중 오류 발생:', refreshError);
        console.error('❌ 오류 상세:', refreshError.response || refreshError.message);
        
        // 최대 시도 횟수에 도달했을 때만 로그아웃 처리
        if (refreshAttempts >= MAX_REFRESH_ATTEMPTS) {
          localStorage.clear();
          console.error(`⛔ 토큰 갱신 오류로 인한 로그아웃 - 현재 페이지: ${currentPath}`);
          
          // 현재 로그인 페이지나 홈 페이지가 아닌 경우에만 리디렉션
          if (currentPath !== '/login' && currentPath !== '/') {
            window.location.href = '/login?reason=refresh_error';
          }
        }
        
        isRefreshing = false;
        return Promise.reject(refreshError);
      }
    }
    
    return Promise.reject(error);
  }
); 