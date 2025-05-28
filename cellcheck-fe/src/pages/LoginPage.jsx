// src/pages/LoginPage.jsx
import React, { useState, useEffect } from 'react';
import { Lock, Shield, Battery, BatteryCharging, Info, ChevronRight, UserPlus } from 'lucide-react';
import Header from '../components/layout/Header';
import Footer from '../components/layout/Footer';
import BackgroundPattern from '../components/layout/BackgroundPattern';
import LoginForm from '../components/auth/LoginForm';
import AdminKeyModal from '../components/auth/AdminKeyModal';
import AdminSignupModal from '../components/auth/AdminSignupModal';
import Button from '../components/common/Button';
import useAuth from '../hooks/useAuth';
import useModal from '../hooks/useModal';
import FadeAnimation from '../components/animations/FadeAnimation';

const LoginPage = () => {
  // 인증 및 모달 관련 훅 사용
  const { 
    isAdminMode, 
    switchToAdminMode, 
    switchToUserMode, 
    login, 
    systemStatus,
    loading,
    error 
  } = useAuth();
  
  const { 
    isOpen: showAdminModal, 
    openModal: openAdminModal, 
    closeModal: closeAdminModal 
  } = useModal();
  
  // 관리자 회원가입 모달 상태
  const { 
    isOpen: showAdminSignupModal, 
    openModal: openAdminSignupModal, 
    closeModal: closeAdminSignupModal 
  } = useModal();
  
  // 애니메이션 상태
  const [showFadeAnimation, setShowFadeAnimation] = useState(false);
  const [loginError, setLoginError] = useState(null);
  
  // 관리자 키 입력 처리
  const handleAdminKeySubmit = (adminKey) => {
    const result = switchToAdminMode(adminKey);
    
    if (result.success) {
      // 모달 닫기
      closeAdminModal();
      
      // 애니메이션 시작
      setShowFadeAnimation(true);
      
      // 애니메이션 완료 후 정리
      setTimeout(() => {
        setShowFadeAnimation(false);
      }, 800);
    }
  };
  
  // 일반 사용자 모드로 전환
  const handleExitAdmin = () => {
    switchToUserMode();
    
    // 애니메이션 시작
    setShowFadeAnimation(true);
    
    // 애니메이션 완료 후 정리
    setTimeout(() => {
      setShowFadeAnimation(false);
    }, 800);
  };
  
  // 로그인 폼 제출 처리
  const handleLoginSubmit = async (formData, isAdmin) => {
    setLoginError(null);
    const result = await login(formData, isAdmin);
    
    if (!result.success) {
      setLoginError(result.message || '로그인에 실패했습니다.');
    }
  };
  
  // 관리자 회원가입 성공 처리
  const handleAdminSignupSuccess = () => {
    // 성공 메시지 표시 또는 다른 처리
    alert('관리자 계정이 성공적으로 등록되었습니다. 로그인해주세요.');
  };
  
  // 에러 업데이트
  useEffect(() => {
    if (error) {
      setLoginError(error);
    }
  }, [error]);
  
  useEffect(() => {
    // 컴포넌트 마운트 후 애니메이션을 보여주기 위한 코드
    const timer = setTimeout(() => {
      setShowFadeAnimation(true);
      setTimeout(() => {
        setShowFadeAnimation(false);
      }, 800);
    }, 300);
    
    return () => clearTimeout(timer);
  }, []);

  // 단축키 처리 추가
  useEffect(() => {
    const handleKeyDown = (e) => {
      // Ctrl + Shift + F
      if (e.ctrlKey && e.shiftKey && e.key === 'F') {
        e.preventDefault();
        if (!isAdminMode) {
          openAdminModal();
        }
      }
    };
    
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isAdminMode, openAdminModal]);

  return (
    <div className="min-h-screen w-full h-full flex flex-col justify-center items-center relative overflow-hidden login-page">
      {/* 페이드 애니메이션 */}
      <FadeAnimation 
        show={showFadeAnimation} 
        isEntering={!isAdminMode}
      />
      
      {/* 배경 패턴 */}
      <BackgroundPattern isAdminMode={isAdminMode} />
      
      {/* 헤더 */}
      <Header 
        isAdminMode={isAdminMode} 
        systemStatus={systemStatus} 
      />
      
      <div className="relative py-3 w-full max-w-4xl mx-auto z-10 login-container flex flex-col md:flex-row items-center gap-6 px-4">
        {/* 로고 및 설명 섹션 */}
        <div className="flex flex-col items-center md:items-start w-full md:w-1/2 mb-8 md:mb-0">
          <div className="flex items-center justify-center mb-4">
            <div className={`p-4 rounded-full ${isAdminMode ? 'bg-red-500/20' : 'bg-blue-500/20'} mb-2`}>
              {isAdminMode ? (
                <Shield className="h-16 w-16 text-red-500" />
              ) : (
                <BatteryCharging className="h-16 w-16 text-blue-500" />
              )}
            </div>
          </div>
          
          <h1 className={`text-3xl md:text-4xl font-bold mb-6 text-center md:text-left ${
            isAdminMode ? 'text-white' : 'text-gray-900'
          }`}>
            {isAdminMode ? '배터리 관리 시스템' : '스마트 배터리 시스템'}
          </h1>
          
          <p className={`text-lg mb-6 text-center md:text-left ${
            isAdminMode ? 'text-gray-300' : 'text-gray-600'
          }`}>
            {isAdminMode 
              ? '안전하고 효율적인 배터리 모니터링 및 제어 시스템입니다.' 
              : '배터리 상태를 확인하고 최적의 성능을 유지하세요.'}
          </p>
          
          <div className={`hidden md:flex flex-col space-y-4 ${
            isAdminMode ? 'text-gray-300' : 'text-gray-600'
          }`}>
            <div className="flex items-center">
              <div className={`rounded-full p-1 mr-3 ${
                isAdminMode ? 'bg-red-500/20 text-red-400' : 'bg-blue-500/20 text-blue-500'
              }`}>
                <ChevronRight className="h-4 w-4" />
              </div>
              <span>실시간 배터리 상태 모니터링</span>
            </div>
            <div className="flex items-center">
              <div className={`rounded-full p-1 mr-3 ${
                isAdminMode ? 'bg-red-500/20 text-red-400' : 'bg-blue-500/20 text-blue-500'
              }`}>
                <ChevronRight className="h-4 w-4" />
              </div>
              <span>성능 최적화 및 문제 감지</span>
            </div>
            <div className="flex items-center">
              <div className={`rounded-full p-1 mr-3 ${
                isAdminMode ? 'bg-red-500/20 text-red-400' : 'bg-blue-500/20 text-blue-500'
              }`}>
                <ChevronRight className="h-4 w-4" />
              </div>
              <span>안전한 데이터 관리</span>
            </div>
          </div>
        </div>
        
        {/* 로그인 폼 섹션 */}
        <div className="w-full md:w-1/2">
          <div className={`absolute inset-0 ${isAdminMode ? 'bg-gradient-to-r-admin' : 'bg-gradient-to-r-user'} shadow-lg transform -skew-y-6 sm:-rotate-6 sm:rounded-3xl login-form-bg`} style={{ maxWidth: '448px', left: '50%', transform: 'translateX(-50%) skew(-6deg)' }}></div>
          
          <div className={`relative px-6 py-10 ${
            isAdminMode ? 'bg-gray-800' : 'bg-white'
          } shadow-lg sm:rounded-3xl sm:p-10 login-form-content`} style={{width: '448px', maxWidth: '100%', margin: '0 auto'}}>
            <div className="max-w-md mx-auto">
              <div className="flex items-center justify-center mb-6">
                {isAdminMode ? (
                  <Shield className="h-8 w-8 text-red-500 mr-2" />
                ) : (
                  <Shield className="h-8 w-8 text-blue-500 mr-2" />
                )}
                <h1 className={`text-2xl font-semibold ${
                  isAdminMode ? 'text-gray-100' : 'text-gray-900'
                }`}>
                  {isAdminMode ? '관리자 로그인' : '사용자 로그인'}
                </h1>
              </div>
              
              <div className="mb-2 flex items-center justify-between">
                <h2 className={`text-lg font-medium ${
                  isAdminMode ? 'text-gray-100' : 'text-gray-800'
                }`}>
                  {isAdminMode ? '관리자 액세스' : '사용자 액세스'}
                </h2>
                <div className="flex items-center">
                  <div className={`w-2 h-2 rounded-full mr-1 ${
                    isAdminMode 
                      ? 'bg-red-500' 
                      : systemStatus.serverStatus === 'ONLINE' ? 'bg-green-500' : 'bg-red-500'
                  }`}></div>
                  <span className={`text-xs font-mono ${
                    isAdminMode ? 'text-gray-400' : 'text-gray-500'
                  }`}>
                    {isAdminMode ? 'ADMIN MODE' : `SYSTEM ${systemStatus.serverStatus}`}
                  </span>
                </div>
              </div>
              
              {loginError && (
                <div className="mb-4 p-2 bg-red-100 border-l-4 border-red-500 text-red-700 text-sm">
                  <p>{loginError}</p>
                </div>
              )}
              
              <LoginForm 
                isAdminMode={isAdminMode}
                onSwitchToUserMode={handleExitAdmin}
                onSubmit={handleLoginSubmit}
              />
              
              {/* 관리자 회원가입 버튼 추가 */}
              {isAdminMode && (
                <div className="mt-6">
                  <button
                    type="button"
                    onClick={openAdminSignupModal}
                    className="w-full flex items-center justify-center text-sm font-medium text-gray-400 hover:text-gray-300 border border-gray-700 rounded-md p-2 hover:bg-gray-700 transition-colors"
                  >
                    <UserPlus className="h-4 w-4 mr-2" />
                    <span>관리자 계정 등록</span>
                  </button>
                </div>
              )}
              
              <div className="mt-6 text-center">
                <p className={`text-xs ${isAdminMode ? 'text-gray-500' : 'text-gray-600'}`}>
                  © 2025 배터리 관리 시스템.
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>
      
      {/* 푸터 */}
      <Footer 
        isAdminMode={isAdminMode} 
        systemStatus={systemStatus} 
      />
      
      {/* 관리자 키 입력 모달 */}
      <AdminKeyModal 
        isOpen={showAdminModal} 
        onClose={closeAdminModal}
        onSubmit={handleAdminKeySubmit}
      />
      
      {/* 관리자 회원가입 모달 */}
      <AdminSignupModal
        isOpen={showAdminSignupModal}
        onClose={closeAdminSignupModal}
        onSuccess={handleAdminSignupSuccess}
      />
    </div>
  );
};

export default LoginPage;