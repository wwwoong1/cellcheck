import React, { useState, useEffect } from 'react';
import { 
  Battery, User, Key, Eye, EyeOff, LogIn, 
  Clock, Shield, Info, HelpCircle, AlertTriangle,
  RefreshCw, ChevronRight, Zap, Smartphone, Cpu,
  Lock, ShieldAlert, X
} from 'lucide-react';

const AdminModeTransition = () => {
  // 상태 관리
  const [showAdminModal, setShowAdminModal] = useState(false);
  const [adminKey, setAdminKey] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [adminKeyError, setAdminKeyError] = useState('');
  const [isAdminMode, setIsAdminMode] = useState(false);
  const [adminAnimating, setAdminAnimating] = useState(false);
  const [adminUIReady, setAdminUIReady] = useState(false);
  const [showCircleAnimation, setShowCircleAnimation] = useState(false);
  
  // 시스템 상태
  const [systemStatus] = useState({
    serverStatus: 'ONLINE',
    deviceCount: 42
  });
  
  // 관리자 키 입력 핸들러
  const handleAdminKeyChange = (e) => {
    setAdminKey(e.target.value);
    setAdminKeyError('');
  };
  
  // 키 입력 시 엔터키 처리
  const handleAdminKeyPress = (e) => {
    if (e.key === 'Enter') {
      handleAdminKeySubmit();
    }
  };
  
  // 관리자 키 검증 및 모달 닫기
  const handleAdminKeySubmit = () => {
    if (adminKey === '1234') {
      // 모달 닫기
      setShowAdminModal(false);
      
      // 즉시 관리자 모드로 전환하고 UI 준비
      setIsAdminMode(true);
      setAdminUIReady(true);
      
      // 화면 가리기
      document.body.style.overflow = 'hidden';
      
      // 약간의 딜레이 후 애니메이션 시작
      setTimeout(() => {
        setShowCircleAnimation(true);
        
        // 애니메이션 완료 후 정리
        setTimeout(() => {
          setShowCircleAnimation(false);
          document.body.style.overflow = '';
        }, 2000);
      }, 100);
    } else {
      setAdminKeyError('관리자 키가 올바르지 않습니다.');
    }
  };
  
  // 관리자 모드 종료
  const handleExitAdmin = () => {
    setIsAdminMode(false);
    setAdminUIReady(false);
    setAdminKey('');
  };
  
  useEffect(() => {
    // 컴포넌트 마운트 후 애니메이션을 보여주기 위한 코드
    const timer = setTimeout(() => {
      setShowCircleAnimation(true);
      setTimeout(() => {
        setShowCircleAnimation(false);
      }, 2000);
    }, 500);
    
    return () => clearTimeout(timer);
  }, []);

  // 관리자 페이지 렌더링
  if (isAdminMode) {
    return (
      <div className="min-h-screen bg-gray-900 py-6 flex flex-col justify-center sm:py-12 relative overflow-hidden">
        {/* 원형 애니메이션 - 전환 중일 때만 표시 */}
        {adminAnimating && (
          <div 
            className="circle-animation absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 bg-gray-900 rounded-full z-30"
            style={{
              width: '0px',
              height: '0px',
              animation: 'circle-expand 1s ease-out forwards'
            }}
          />
        )}
        
        {/* 배경 패턴 - 다크 모드 */}
        <div className="absolute inset-0 bg-gray-900 z-0"
          style={{
            backgroundImage: `url("data:image/svg+xml,%3Csvg width='60' height='60' viewBox='0 0 60 60' xmlns='http://www.w3.org/2000/svg'%3E%3Cg fill='none' fill-rule='evenodd'%3E%3Cg fill='%23292f3b' fill-opacity='0.4'%3E%3Cpath d='M36 34v-4h-2v4h-4v2h4v4h2v-4h4v-2h-4zm0-30V0h-2v4h-4v2h4v4h2V6h4V4h-4zM6 34v-4H4v4H0v2h4v4h2v-4h4v-2H6zM6 4V0H4v4H0v2h4v4h2V6h4V4H6z'/%3E%3C/g%3E%3C/g%3E%3C/svg%3E")`
          }}
        />
        
        {/* 헤더 - 다크 모드 */}
        <div className="fixed top-0 left-0 right-0 bg-gray-800 bg-opacity-90 backdrop-blur-sm border-b border-gray-700 py-2 px-4 z-10">
          <div className="max-w-7xl mx-auto flex justify-between items-center">
            <div className="flex items-center">
              <Battery className="h-5 w-5 text-red-500 mr-2" />
              <span className="text-lg font-medium text-gray-100">스마트 배터리 시스템</span>
              <span className="ml-2 px-2 py-0.5 bg-red-900 text-red-100 text-xs rounded-md">관리자 접속</span>
            </div>
            <div className="flex items-center text-sm text-gray-400">
              <Clock className="h-4 w-4 mr-1" />
              <span>{new Date().toLocaleString()}</span>
            </div>
          </div>
        </div>
        
        {/* 로그인 카드 - 다크 모드 */}
        <div className="relative py-3 sm:max-w-xl sm:mx-auto z-10">
          <div className="absolute inset-0 bg-gradient-to-r from-red-500 to-purple-600 shadow-lg transform -skew-y-6 sm:skew-y-0 sm:-rotate-6 sm:rounded-3xl"></div>
          <div className="relative px-4 py-10 bg-gray-800 shadow-lg sm:rounded-3xl sm:p-10" style={{width: '448px', maxWidth: '100%'}}>
            <div className="max-w-md mx-auto">
              <div className="flex items-center justify-center mb-6">
                <ShieldAlert className="h-8 w-8 text-red-500 mr-2" />
                <h1 className="text-2xl font-semibold text-gray-100">IoT 관리자 시스템</h1>
              </div>
              
              <div className="mb-2 flex items-center justify-between">
                <h2 className="text-lg font-medium text-gray-100">관리자 로그인</h2>
                <div className="flex items-center">
                  <div className="w-2 h-2 rounded-full mr-1 bg-red-500"></div>
                  <span className="text-xs font-mono text-gray-400">ADMIN MODE</span>
                </div>
              </div>
              
              <div>
                <div className="mb-4">
                  <label htmlFor="username" className="block text-sm font-medium text-gray-300 mb-1 flex items-center">
                    <User className="h-4 w-4 mr-1" />
                    관리자 ID
                  </label>
                  <div className="relative">
                    <input
                      id="username"
                      name="username"
                      type="text"
                      autoComplete="username"
                      required
                      className="px-3 py-2 mt-1 block w-full border border-gray-600 bg-gray-700 text-white rounded-md text-sm focus:outline-none focus:ring-red-500 focus:border-red-500"
                      placeholder="관리자 ID를 입력하세요"
                    />
                  </div>
                </div>
                
                <div className="mb-6">
                  <label htmlFor="password" className="block text-sm font-medium text-gray-300 mb-1 flex items-center">
                    <Key className="h-4 w-4 mr-1" />
                    비밀번호
                  </label>
                  <div className="relative">
                    <input
                      id="password"
                      name="password"
                      type="password"
                      autoComplete="current-password"
                      required
                      className="px-3 py-2 mt-1 block w-full border border-gray-600 bg-gray-700 text-white rounded-md text-sm focus:outline-none focus:ring-red-500 focus:border-red-500"
                      placeholder="비밀번호를 입력하세요"
                    />
                    <button
                      type="button"
                      className="absolute right-2 top-1/2 transform -translate-y-1/2 text-gray-400 hover:text-gray-300"
                    >
                      <Eye className="h-4 w-4" />
                    </button>
                  </div>
                </div>
                
                <div className="flex items-center justify-between mb-6">
                  <div className="flex items-center">
                    <input
                      id="remember-me"
                      name="remember-me"
                      type="checkbox"
                      className="h-4 w-4 text-red-600 focus:ring-red-500 border-gray-700 rounded"
                    />
                    <label htmlFor="remember-me" className="ml-2 block text-sm text-gray-300">
                      로그인 정보 저장
                    </label>
                  </div>
                  
                  <div className="text-sm font-medium text-red-400 hover:text-red-300 cursor-pointer">
                    비밀번호 초기화
                  </div>
                </div>
                
                <div>
                  <button
                    className="w-full flex justify-center items-center px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-red-600 hover:bg-red-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-red-500"
                  >
                    <LogIn className="h-4 w-4 mr-2" />
                    <span>관리자 로그인</span>
                  </button>
                </div>
                
                <div className="mt-4 text-center">
                  <button 
                    onClick={handleExitAdmin}
                    className="text-sm font-medium text-gray-400 hover:text-gray-300 inline-flex items-center"
                  >
                    <X className="h-3 w-3 mr-1" />
                    <span>일반 사용자 모드로 돌아가기</span>
                  </button>
                </div>
              </div>
              
              <div className="mt-8 pt-6 border-t border-gray-700">
                <h3 className="text-sm font-medium text-gray-300 mb-3 flex items-center">
                  <ShieldAlert className="h-4 w-4 text-red-500 mr-1" />
                  관리자 공지사항
                </h3>
                
                <div className="space-y-2 max-h-32 overflow-y-auto">
                  <div className="flex items-start p-2 bg-gray-700 rounded-md text-sm">
                    <AlertTriangle className="h-4 w-4 text-red-400 mr-2 mt-0.5" />
                    <div className="flex-1">
                      <p className="text-gray-300">시스템 보안 업데이트 적용 예정 (05/20)</p>
                      <p className="text-xs text-gray-500 mt-1">2025-05-15</p>
                    </div>
                    <ChevronRight className="h-4 w-4 text-gray-500" />
                  </div>
                  
                  <div className="flex items-start p-2 bg-gray-700 rounded-md text-sm">
                    <RefreshCw className="h-4 w-4 text-purple-400 mr-2 mt-0.5" />
                    <div className="flex-1">
                      <p className="text-gray-300">백업 서버 점검 완료</p>
                      <p className="text-xs text-gray-500 mt-1">2025-05-13</p>
                    </div>
                    <ChevronRight className="h-4 w-4 text-gray-500" />
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
        
        {/* 푸터 - 다크 모드 */}
        <div className="fixed bottom-0 left-0 right-0 bg-gray-800 bg-opacity-90 backdrop-blur-sm border-t border-gray-700 py-2 px-4 text-xs text-gray-400 z-10">
          <div className="max-w-7xl mx-auto flex justify-between items-center">
            <div className="flex items-center space-x-4">
              <div className="flex items-center">
                <Zap className="h-3 w-3 text-red-500 mr-1" />
                <span>관리자 모드: 활성화</span>
              </div>
              <div className="flex items-center">
                <Smartphone className="h-3 w-3 text-blue-400 mr-1" />
                <span>연결된 디바이스: {systemStatus.deviceCount}</span>
              </div>
              <div className="flex items-center">
                <Cpu className="h-3 w-3 text-purple-400 mr-1" />
                <span>시스템 버전: v2.3.7</span>
              </div>
            </div>
            
            <div className="flex items-center space-x-4">
              <div className="text-red-400 hover:text-red-300 flex items-center cursor-pointer">
                <HelpCircle className="h-3 w-3 mr-1" />
                <span>관리자 매뉴얼</span>
              </div>
              <span>© 2025 스마트 배터리 시스템</span>
            </div>
          </div>
        </div>
      </div>
    );
  }
  
  return (
    <div className="min-h-screen bg-gray-100 py-6 flex flex-col justify-center items-center sm:py-12 relative overflow-hidden">
      {/* 원형 애니메이션 - 점점 줄어들면서 페이지 드러남 */}
      {showCircleAnimation && (
        <div className="circle-animation fixed top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 rounded-full" 
          style={{zIndex: 9999}}
        />
      )}
      
      {/* 배경 패턴 */}
      <div className="absolute inset-0 bg-gray-100 z-0"
        style={{
          backgroundImage: `url("data:image/svg+xml,%3Csvg width='60' height='60' viewBox='0 0 60 60' xmlns='http://www.w3.org/2000/svg'%3E%3Cg fill='none' fill-rule='evenodd'%3E%3Cg fill='%23e5e7eb' fill-opacity='0.4'%3E%3Cpath d='M36 34v-4h-2v4h-4v2h4v4h2v-4h4v-2h-4zm0-30V0h-2v4h-4v2h4v4h2V6h4V4h-4zM6 34v-4H4v4H0v2h4v4h2v-4h4v-2H6zM6 4V0H4v4H0v2h4v4h2V6h4V4H6z'/%3E%3C/g%3E%3C/g%3E%3C/svg%3E")`
        }}
      />
      
      {/* 헤더 */}
      <div className="fixed top-0 left-0 right-0 bg-white bg-opacity-90 backdrop-blur-sm border-b border-gray-200 py-2 px-4 z-10">
        <div className="max-w-7xl mx-auto flex justify-between items-center">
          <div className="flex items-center">
            <Battery className="h-5 w-5 text-blue-500 mr-2" />
            <span className="text-lg font-medium text-gray-800">스마트 배터리 시스템</span>
            <span className="ml-2 px-2 py-0.5 bg-blue-100 text-blue-700 text-xs rounded-md">사용자 접속</span>
          </div>
          <div className="flex items-center text-sm text-gray-600">
            <Clock className="h-4 w-4 mr-1" />
            <span>{new Date().toLocaleString()}</span>
          </div>
        </div>
      </div>
      
      <div className="relative py-3 sm:max-w-xl sm:mx-auto z-10">
        <div className="absolute inset-0 bg-gradient-to-r from-blue-400 to-indigo-500 shadow-lg transform -skew-y-6 sm:skew-y-0 sm:-rotate-6 sm:rounded-3xl"></div>
        <div className="relative px-4 py-10 bg-white shadow-lg sm:rounded-3xl sm:p-10" style={{width: '448px', maxWidth: '100%'}}>
          <div className="max-w-md mx-auto">
            <div className="flex items-center justify-center mb-6">
              <Shield className="h-8 w-8 text-blue-500 mr-2" />
              <h1 className="text-2xl font-semibold text-gray-900">IoT 스마트 시스템</h1>
            </div>
            
            <div className="mb-2 flex items-center justify-between">
              <h2 className="text-lg font-medium text-gray-800">사용자 로그인</h2>
              <div className="flex items-center">
                <div className={`w-2 h-2 rounded-full mr-1 ${systemStatus.serverStatus === 'ONLINE' ? 'bg-green-500' : 'bg-red-500'}`}></div>
                <span className="text-xs font-mono text-gray-500">SYSTEM {systemStatus.serverStatus}</span>
              </div>
            </div>
            
            <div className="mb-6 flex justify-center">
              <button 
                onClick={() => setShowAdminModal(true)}
                className="px-4 py-2 bg-blue-100 text-blue-700 rounded-md hover:bg-blue-200 flex items-center"
              >
                <Lock className="h-4 w-4 mr-2" />
                관리자 모드 접근 (Ctrl+Shift+F)
              </button>
            </div>
            
            <div>
              <div className="mb-4">
                <label htmlFor="username" className="block text-sm font-medium text-gray-700 mb-1 flex items-center">
                  <User className="h-4 w-4 mr-1" />
                  아이디
                </label>
                <div className="relative">
                  <input
                    id="username"
                    name="username"
                    type="text"
                    autoComplete="username"
                    required
                    className="px-3 py-2 mt-1 block w-full border border-gray-300 rounded-md text-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                    placeholder="아이디를 입력하세요"
                  />
                </div>
              </div>
              
              <div className="mb-6">
                <label htmlFor="password" className="block text-sm font-medium text-gray-700 mb-1 flex items-center">
                  <Key className="h-4 w-4 mr-1" />
                  비밀번호
                </label>
                <div className="relative">
                  <input
                    id="password"
                    name="password"
                    type="password"
                    autoComplete="current-password"
                    required
                    className="px-3 py-2 mt-1 block w-full border border-gray-300 rounded-md text-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                    placeholder="비밀번호를 입력하세요"
                  />
                  <button
                    type="button"
                    className="absolute right-2 top-1/2 transform -translate-y-1/2 text-gray-500 hover:text-gray-700"
                  >
                    <Eye className="h-4 w-4" />
                  </button>
                </div>
              </div>
              
              <div className="flex items-center justify-between mb-6">
                <div className="flex items-center">
                  <input
                    id="remember-me"
                    name="remember-me"
                    type="checkbox"
                    className="h-4 w-4 text-blue-500 focus:ring-blue-500 border-gray-300 rounded"
                  />
                  <label htmlFor="remember-me" className="ml-2 block text-sm text-gray-700">
                    로그인 정보 저장
                  </label>
                </div>
                
                <div className="text-sm font-medium text-blue-600 hover:text-blue-500 cursor-pointer">
                  비밀번호 찾기
                </div>
              </div>
              
              <div>
                <button
                  className="w-full flex justify-center items-center px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
                >
                  <LogIn className="h-4 w-4 mr-2" />
                  <span>로그인</span>
                </button>
              </div>
              
              <div className="mt-4 text-center">
                <span className="text-sm text-gray-600">계정이 없으신가요?</span>
                <span className="ml-1 text-sm font-medium text-blue-600 hover:text-blue-500 cursor-pointer">
                  회원가입
                </span>
              </div>
            </div>
          </div>
        </div>
      </div>
      
      {/* 푸터 */}
      <div className="fixed bottom-0 left-0 right-0 bg-white bg-opacity-90 backdrop-blur-sm border-t border-gray-200 py-2 px-4 text-xs text-gray-600 z-10">
        <div className="max-w-7xl mx-auto flex justify-between items-center">
          <div className="flex items-center space-x-4">
            <div className="flex items-center">
              <Zap className="h-3 w-3 text-green-500 mr-1" />
              <span>서버 상태: 정상</span>
            </div>
            <div className="flex items-center">
              <Smartphone className="h-3 w-3 text-blue-500 mr-1" />
              <span>연결된 디바이스: {systemStatus.deviceCount}</span>
            </div>
            <div className="flex items-center">
              <Cpu className="h-3 w-3 text-purple-500 mr-1" />
              <span>시스템 버전: v2.3.7</span>
            </div>
          </div>
          
          <div className="flex items-center space-x-4">
            <div className="text-blue-600 hover:text-blue-500 flex items-center cursor-pointer">
              <HelpCircle className="h-3 w-3 mr-1" />
              <span>도움말</span>
            </div>
            <span>© 2025 스마트 배터리 시스템</span>
          </div>
        </div>
      </div>
      
      {/* 관리자 키 입력 모달 */}
      {showAdminModal && (
        <div className="fixed inset-0 flex items-center justify-center z-50">
          <div 
            className="absolute inset-0 bg-black bg-opacity-50 backdrop-blur-sm"
          ></div>
          
          <div className="bg-white rounded-lg shadow-xl w-full max-w-md z-10 overflow-hidden">
            <div className="bg-blue-600 px-4 py-3 flex items-center justify-between">
              <div className="flex items-center text-white">
                <ShieldAlert className="h-5 w-5 mr-2" />
                <h3 className="text-lg font-medium">관리자 접근</h3>
              </div>
              <button 
                onClick={() => setShowAdminModal(false)}
                className="text-white hover:text-gray-200"
              >
                <X className="h-5 w-5" />
              </button>
            </div>
            
            <div className="p-6">
              <div className="mb-4 bg-yellow-50 border-l-4 border-yellow-400 p-3 text-sm text-yellow-700">
                <p className="flex items-center">
                  <AlertTriangle className="h-4 w-4 mr-2 flex-shrink-0" />
                  <span>관리자 접근은 허가된 사용자만 가능합니다.</span>
                </p>
              </div>
              
              <div className="mb-4">
                <label htmlFor="adminKey" className="block text-sm font-medium text-gray-700 mb-1 flex items-center">
                  <Lock className="h-4 w-4 mr-1" />
                  관리자 키
                </label>
                <div className="relative">
                  <input
                    id="adminKey"
                    type={showPassword ? "text" : "password"}
                    value={adminKey}
                    onChange={handleAdminKeyChange}
                    onKeyPress={handleAdminKeyPress}
                    className="px-3 py-2 block w-full border border-gray-300 rounded-md text-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
                    placeholder="관리자 키를 입력하세요"
                  />
                  <button
                    type="button"
                    onClick={() => setShowPassword(!showPassword)}
                    className="absolute right-2 top-1/2 transform -translate-y-1/2 text-gray-500 hover:text-gray-700"
                  >
                    {showPassword ? (
                      <EyeOff className="h-4 w-4" />
                    ) : (
                      <Eye className="h-4 w-4" />
                    )}
                  </button>
                </div>
                {adminKeyError && (
                  <p className="mt-1 text-sm text-red-600">{adminKeyError}</p>
                )}
              </div>
              
              <div className="flex justify-end">
                <button
                  onClick={() => setShowAdminModal(false)}
                  className="mr-2 px-4 py-2 border border-gray-300 rounded-md shadow-sm text-sm font-medium text-gray-700 bg-white hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
                >
                  취소
                </button>
                <button
                  onClick={handleAdminKeySubmit}
                  className="px-4 py-2 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
                >
                  접근
                </button>
              </div>
            </div>
            
            <div className="bg-gray-50 px-4 py-3 text-xs text-gray-500 flex items-center">
              <Key className="h-3 w-3 mr-1" />
              <span>단축키: Ctrl+Shift+F (개발자 및 관리자 전용)</span>
            </div>
          </div>
        </div>
      )}
      
      {/* CSS 애니메이션 스타일 */}
      <style jsx>{`
        /* 전체를 가리는 애니메이션에서 점점 드러나게 */
        @keyframes reveal-circle {
          0% {
            width: 4000px;
            height: 4000px;
            opacity: 1;
          }
          100% {
            width: 0;
            height: 0;
            opacity: 0;
          }
        }
        
        .circle-animation {
          width: 4000px;
          height: 4000px;
          background-color: #111827; /* 다크 그레이 */
          animation: reveal-circle 1.8s ease-in-out forwards;
          box-shadow: 0 0 200px 100px rgba(255, 0, 0, 0.4);
        }
      `}</style>
    </div>
  );
};

export default AdminModeTransition;