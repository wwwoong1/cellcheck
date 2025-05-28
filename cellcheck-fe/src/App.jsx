// src/App.jsx
import React, { useEffect } from 'react';
import { BrowserRouter, Routes, Route, Navigate, useNavigate, useLocation } from 'react-router-dom';
import LoginPage from './pages/LoginPage';
import AdminPage from './pages/AdminPage';
import UserPage from './pages/UserPage';
import { AuthProvider, AuthContext } from './context/AuthContext';
import { WebSocketProvider } from './context/WebSocketContext';
import { useContext } from 'react';
import Footer from './components/layout/Footer';

// 페이지 전환 애니메이션 래퍼 컴포넌트
const AnimatedPage = ({ children }) => {
  return (
    <div className="page-wrapper">
      <div className="page-transition-enter-active">
        {children}
      </div>
    </div>
  );
};

// 인증 필요한 라우트를 위한 Protected 컴포넌트
const Protected = ({ component: Component, requiredRole = null, ...rest }) => {
  const { isAuthenticated, user } = useContext(AuthContext);
  const location = useLocation();

  // 로그인 상태 확인 및 로깅
  console.log(`🔐 Protected 라우트 접근 (${location.pathname}): 인증 상태 = ${isAuthenticated}, 역할 = ${user?.role || 'none'}`);
  
  // JWT 토큰 확인
  const checkToken = () => {
    const accessToken = localStorage.getItem('accessToken');
    const refreshToken = localStorage.getItem('refreshToken');
    
    console.log(`🔐 보호된 라우트 토큰 확인 (${location.pathname}):
    - 액세스 토큰 존재: ${!!accessToken}
    - 리프레시 토큰 존재: ${!!refreshToken}`);
    
    if (accessToken) {
      try {
        const payload = JSON.parse(atob(accessToken.split('.')[1]));
        const expiryDate = new Date(payload.exp * 1000);
        const now = new Date();
        const timeLeft = Math.floor((expiryDate - now) / 1000);
        
        console.log(`🕒 토큰 만료 정보 (${location.pathname}):
        - 만료 시간: ${expiryDate.toLocaleString()}
        - 현재 시간: ${now.toLocaleString()}
        - 남은 시간: ${timeLeft}초 (${Math.floor(timeLeft / 60)}분 ${timeLeft % 60}초)`);
        
        if (timeLeft < 0) {
          console.warn(`⚠️ 토큰이 만료되었습니다! 페이지: ${location.pathname}`);
        }
      } catch (e) {
        console.error('💥 토큰 확인 중 오류:', e);
      }
    }
  };
  
  // 토큰 확인 호출
  useEffect(() => {
    checkToken();
  }, [location.pathname]);

  // 로그인 상태 확인
  if (!isAuthenticated) {
    // 로그인되지 않은 경우 로그인 페이지로 리다이렉트
    console.warn(`⛔ 인증되지 않은 접근 감지: ${location.pathname} → / 리다이렉트`);
    return <Navigate to="/" replace state={{ from: location }} />;
  }
  
  // 역할 확인 (필요한 경우)
  if (requiredRole && user.role !== requiredRole) {
    // 필요한 역할이 없는 경우 적절한 페이지로 리다이렉트
    console.warn(`⛔ 권한 없는 접근 감지: ${user.role} 사용자가 ${requiredRole} 페이지 접근 시도`);
    return <Navigate to={user.role === 'admin' ? "/admin" : "/user"} replace />;
  }
  
  // 모든 조건을 통과하면 요청된 컴포넌트 렌더링
  return (
    <AnimatedPage>
      <Component {...rest} />
    </AnimatedPage>
  );
};

function App() {
  return (
    <BrowserRouter>
      <WebSocketProvider>
        <AuthProvider>
          <AppRoutes />
          {/* {process.env.NODE_ENV === 'development' && <WebSocketDebugger />} */}
        </AuthProvider>
      </WebSocketProvider>
    </BrowserRouter>
  );
}

// 라우트 컴포넌트를 분리하여 useNavigate 훅을 사용할 수 있게 함
const AppRoutes = () => {
  const navigate = useNavigate();
  const location = useLocation();
  const { isAuthenticated, user } = useContext(AuthContext);
  
  // 현재 경로가 루트 경로('/')인지 확인
  const isRootPath = location.pathname === '/';
  
  // 관리자 모드인지 확인
  const isAdminMode = isAuthenticated && user?.role === 'admin';

  // 페이지 로드 시 경로 확인 및 로깅
  useEffect(() => {
    const previousPath = sessionStorage.getItem('previousPath') || 'none';
    console.log(`🧭 페이지 이동 감지: ${previousPath} → ${location.pathname}`);
    
    // 특별한 관심 경로 전환 (관리자 → 홈)
    if (previousPath === '/admin' && location.pathname === '/') {
      console.log(`⚠️ 관리자 페이지에서 홈으로 이동 감지 - 인증 상태 확인`);
      
      // 토큰 상태 확인
      const accessToken = localStorage.getItem('accessToken');
      const refreshToken = localStorage.getItem('refreshToken');
      
      console.log(`🔐 페이지 전환 시 토큰 상태:
      - 액세스 토큰 존재: ${!!accessToken}
      - 리프레시 토큰 존재: ${!!refreshToken}`);
      
      if (accessToken) {
        try {
          const payload = JSON.parse(atob(accessToken.split('.')[1]));
          const expiryDate = new Date(payload.exp * 1000);
          const now = new Date();
          const timeLeft = Math.floor((expiryDate - now) / 1000);
          
          console.log(`🕒 페이지 전환 시 토큰 만료 정보:
          - 만료 시간: ${expiryDate.toLocaleString()}
          - 현재 시간: ${now.toLocaleString()}
          - 남은 시간: ${timeLeft}초`);
          
          if (timeLeft < 0) {
            console.warn('⚠️ 페이지 전환 시 토큰이 이미 만료된 상태 감지');
          }
        } catch (e) {
          console.error('💥 페이지 전환 시 토큰 확인 오류:', e);
        }
      } else {
        console.warn('⚠️ 페이지 전환 시 액세스 토큰이 없음 - 세션 종료 가능성 있음');
      }
    }
    
    // 현재 경로 저장
    sessionStorage.setItem('previousPath', location.pathname);
    
  }, [location]);

  // 로그인 페이지에서는 푸터를 표시하지 않음
  const showFooter = !isRootPath;

  return (
    <div className="app-container">
      <Routes>
        <Route path="/" element={
          <AnimatedPage>
            <LoginPage />
          </AnimatedPage>
        } />
        <Route path="/admin" element={<Protected component={AdminPage} requiredRole="admin" />} />
        <Route path="/user" element={<Protected component={UserPage} requiredRole="user" />} />
        <Route path="*" element={<Navigate to="/" replace />} />
      </Routes>
      {showFooter && <Footer isAdminMode={isAdminMode} />}
    </div>
  );
};

export default App;