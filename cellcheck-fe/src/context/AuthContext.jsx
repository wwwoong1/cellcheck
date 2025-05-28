import React, { createContext, useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import { 
  loginUser as apiLoginUser,
  loginAdmin as apiLoginAdmin,
  logoutUser as apiLogoutUser,
  getCurrentUser,
  isAuthenticated as checkAuthenticated,
  isAdmin as checkIsAdmin,
  getSystemStatus
} from '../api/authApi';

// 인증 컨텍스트 생성
export const AuthContext = createContext(null);

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [isAdminMode, setIsAdminMode] = useState(false);
  const [systemStatus, setSystemStatus] = useState({
    serverStatus: 'ONLINE',
    deviceCount: 42,
    version: 'v1.0.0'
  });
  const [lastTokenCheck, setLastTokenCheck] = useState(Date.now());
  const [tokenExpiryTime, setTokenExpiryTime] = useState(null);
  
  // 페이지 네비게이션을 위한 hook
  const navigate = useNavigate();

  // JWT 토큰 파싱 및 만료 확인
  const checkTokenExpiry = () => {
    const accessToken = localStorage.getItem('accessToken');
    const now = Date.now();
    setLastTokenCheck(now);
    
    if (!accessToken) {
      console.warn('🔑 토큰이 존재하지 않음 - 인증 상태 확인 필요');
      return { valid: false, timeLeft: 0 };
    }
    
    try {
      // JWT 토큰 디코딩
      const payload = JSON.parse(atob(accessToken.split('.')[1]));
      const expiryTimestamp = payload.exp * 1000; // 밀리초로 변환
      const timeLeft = expiryTimestamp - now;
      setTokenExpiryTime(expiryTimestamp);
      
      // 토큰 만료 시간 로깅
      const expiryDate = new Date(expiryTimestamp);
      const timeLeftSec = Math.floor(timeLeft / 1000);
      console.log(`🔐 토큰 확인:
      - 만료 시간: ${expiryDate.toLocaleString()}
      - 현재 시간: ${new Date().toLocaleString()}
      - 남은 시간: ${timeLeftSec}초 (${Math.floor(timeLeftSec / 60)}분 ${timeLeftSec % 60}초)
      - 페이지: ${window.location.pathname}`);
      
      if (timeLeft < 0) {
        console.warn(`⚠️ 토큰이 만료되었습니다! (${Math.abs(Math.floor(timeLeft/1000))}초 전)`);
        if (window.location.pathname !== '/' && window.location.pathname !== '/login') {
          console.warn('🚪 로그인 페이지로 이동 필요');
        }
        return { valid: false, timeLeft: 0, expired: true };
      }
      
      // 만료 임박 경고 (5분 이내)
      if (timeLeft < 300000) { // 5분 (300초)
        console.warn(`⚠️ 토큰 만료 임박: ${Math.floor(timeLeft/1000)}초 남음`);
      }
      
      return { valid: true, timeLeft, expired: false };
    } catch (error) {
      console.error('💥 토큰 파싱 오류:', error);
      return { valid: false, timeLeft: 0, error: true };
    }
  };
  
  // 주기적 토큰 확인 
  useEffect(() => {
    // 60초마다 토큰 상태 확인
    const intervalId = setInterval(() => {
      const result = checkTokenExpiry();
      
      // 토큰이 만료되었거나 유효하지 않은데 인증된 상태라면
      if (!result.valid && isAuthenticated) {
        console.warn('⚠️ 인증 토큰이 만료되었지만 로그인 상태가 유지되고 있습니다. 상태 초기화가 필요합니다.');
        
        // 30초 더 지났는데도 여전히 인증 상태라면 강제 로그아웃
        if ((Date.now() - lastTokenCheck) > 30000) {
          console.error('⛔ 토큰 만료 감지 후 30초가 경과했으나 여전히 인증 상태입니다. 강제 로그아웃합니다.');
          logout(true);
        }
      }
    }, 60000); // 60초마다 확인
    
    return () => clearInterval(intervalId);
  }, [isAuthenticated, lastTokenCheck]);
  
  // 페이지 변경 감지 (특히 /admin → / 이동 시)
  useEffect(() => {
    const handleRouteChange = () => {
      const prevPath = sessionStorage.getItem('currentPath') || '';
      const currentPath = window.location.pathname;
      
      // 이전 경로와 현재 경로 저장
      sessionStorage.setItem('previousPath', prevPath);
      sessionStorage.setItem('currentPath', currentPath);
      
      // 관리자 페이지에서 홈으로 이동하는 경우 특별 로깅
      if (prevPath === '/admin' && currentPath === '/') {
        console.log('👀 관리자 페이지에서 홈으로 이동 감지');
        const tokenStatus = checkTokenExpiry();
        
        if (!tokenStatus.valid) {
          console.warn('⚠️ 관리자 페이지에서 홈으로 이동 시 유효하지 않은 토큰 상태 감지');
          if (tokenStatus.expired) {
            console.error('⛔ 토큰 만료로 인한 로그아웃 가능성 높음');
          }
        }
      }
    };
    
    // 초기 경로 설정
    if (!sessionStorage.getItem('currentPath')) {
      sessionStorage.setItem('currentPath', window.location.pathname);
    }
    
    // popstate 이벤트 리스너 추가
    window.addEventListener('popstate', handleRouteChange);
    
    return () => {
      window.removeEventListener('popstate', handleRouteChange);
    };
  }, []);

  // 로그인 처리
  const login = async ({ loginId, password, rememberMe }, isAdmin = false) => {
    try {
      setLoading(true);
      setError(null);
      
      const response = isAdmin 
        ? await apiLoginAdmin(loginId, password, rememberMe)
        : await apiLoginUser(loginId, password, rememberMe);
      
      // API 응답 구조에 따라 처리 (response.success 또는 response.data.success)
      const isSuccess = response.success || (response.data && response.data.success);
      
      if (isSuccess) {
        // 인증 상태 업데이트
        const userData = getCurrentUser();
        setUser(userData);
        setIsAuthenticated(true);
        setIsAdminMode(userData?.isAdmin || false);
        
        // 토큰 상태 확인
        checkTokenExpiry();
        
        // 적절한 페이지로 리다이렉트
        if (userData?.isAdmin) {
          navigate('/admin');
        } else {
          navigate('/user');
        }
        
        return { success: true };
      } else {
        const errorMessage = response.message || (response.data && response.data.message) || '로그인에 실패했습니다.';
        setError(errorMessage);
        return { success: false, message: errorMessage };
      }
    } catch (err) {
      const errorMessage = err.response?.data?.message || '로그인 중 오류가 발생했습니다.';
      setError(errorMessage);
      return { success: false, message: errorMessage };
    } finally {
      setLoading(false);
    }
  };

  // 로그아웃 처리
  const logout = async (isForced = false) => {
    try {
      if (isForced) {
        console.log('⚠️ 강제 로그아웃 진행 중...');
      } else {
        await apiLogoutUser();
      }
      
      // 로그아웃 전 상태 로깅
      console.log(`🚪 로그아웃 처리 (${isForced ? '강제' : '정상'}):
      - 현재 페이지: ${window.location.pathname}
      - 인증 상태: ${isAuthenticated}
      - 사용자 정보: ${user ? JSON.stringify(user) : 'null'}
      - 토큰 존재: ${!!localStorage.getItem('accessToken')}`);
      
      // 토큰 상태 마지막 확인
      checkTokenExpiry();
      
      // 상태 초기화
      setUser(null);
      setIsAuthenticated(false);
      setIsAdminMode(false);
      localStorage.removeItem('accessToken');
      localStorage.removeItem('refreshToken');
      
      // 로그인 페이지로 리다이렉트
      console.log('🔀 로그인 페이지로 리다이렉트');
      navigate('/');
    } catch (error) {
      console.error('❌ 로그아웃 중 오류가 발생했습니다:', error);
      // 오류가 발생해도 로그아웃으로 처리
      setUser(null);
      setIsAuthenticated(false);
      setIsAdminMode(false);
      localStorage.removeItem('accessToken');
      localStorage.removeItem('refreshToken');
      navigate('/');
    }
  };

  // 관리자 모드 전환
  const switchToAdminMode = (adminKey) => {
    if (adminKey === '1234') {
      setIsAdminMode(true);
      return { success: true };
    }
    
    return { 
      success: false, 
      message: '관리자 키가 유효하지 않습니다.'
    };
  };

  // 일반 사용자 모드 전환
  const switchToUserMode = () => {
    setIsAdminMode(false);
  };

  // 시스템 상태 가져오기
  const fetchSystemStatus = async () => {
    try {
      const statusResponse = await getSystemStatus();
      const status = statusResponse.data || statusResponse;
      
      setSystemStatus(status);
      return status;
    } catch (err) {
      console.error('시스템 상태를 가져오는 중 오류가 발생했습니다:', err);
      return systemStatus;
    }
  };

  // 컴포넌트 마운트 시 인증 상태 확인
  useEffect(() => {
    // localStorage에서 사용자 정보 복구
    const checkAuthStatus = () => {
      // 토큰 유효성 확인
      const tokenStatus = checkTokenExpiry();
      console.log(`🔄 초기 인증 상태 확인: 토큰 유효=${tokenStatus.valid}`);
      
      if (checkAuthenticated() && tokenStatus.valid) {
        const currentUser = getCurrentUser();
        setUser(currentUser);
        setIsAuthenticated(true);
        setIsAdminMode(checkIsAdmin());
        
        console.log(`✅ 로그인 상태 복원 완료:
        - 사용자: ${currentUser?.name || 'Unknown'}
        - 역할: ${checkIsAdmin() ? 'admin' : 'user'}
        - 현재 페이지: ${window.location.pathname}`);
        
        // 이미 로그인되어 있고 루트 경로인 경우 적절한 페이지로 리다이렉트
        if (window.location.pathname === '/') {
          navigate(checkIsAdmin() ? '/admin' : '/user');
        }
      } else {
        // 토큰이 있지만 유효하지 않은 경우 (만료된 경우)
        if (localStorage.getItem('accessToken') && !tokenStatus.valid) {
          console.warn('⚠️ 저장된 토큰이 있으나 유효하지 않음 (만료됨) - 로그아웃 처리');
          localStorage.removeItem('accessToken');
          localStorage.removeItem('refreshToken');
        }
        
        // 로그인되어 있지 않은 경우, 보호된 경로에 접근하려고 하면 로그인 페이지로 리다이렉트
        if (window.location.pathname !== '/' && window.location.pathname !== '/login') {
          console.log(`🚫 보호된 경로 ${window.location.pathname} 접근 시도 - 로그인 페이지로 리다이렉트`);
          navigate('/');
        }
      }
    };
    
    checkAuthStatus();
  }, [navigate]);
  
  // 시스템 상태 정보 가져오기
  useEffect(() => {
    fetchSystemStatus();
    
    // 주기적 업데이트 (소켓 연결을 대체)
    const intervalId = setInterval(fetchSystemStatus, 30000);
    
    return () => clearInterval(intervalId);
  }, []);
  
  const authContextValue = {
    user,
    loading,
    error,
    isAuthenticated,
    isAdminMode,
    systemStatus,
    login,
    logout,
    switchToAdminMode,
    switchToUserMode,
    fetchSystemStatus,
    checkTokenExpiry,
    tokenExpiryTime
  };

  return (
    <AuthContext.Provider value={authContextValue}>
      {children}
    </AuthContext.Provider>
  );
}; 