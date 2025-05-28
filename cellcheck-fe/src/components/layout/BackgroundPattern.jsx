// src/components/layout/BackgroundPattern.jsx
import React from 'react';

const BackgroundPattern = ({ isAdminMode = false }) => {
  return (
    <div className="fixed inset-0 z-[-1] overflow-hidden pointer-events-none">
      {/* 배경 그라데이션 */}
      <div 
        className={`absolute inset-0 ${
          isAdminMode 
            ? 'bg-gradient-to-br from-gray-900 via-gray-800 to-gray-900' 
            : 'bg-gradient-to-br from-blue-50 via-indigo-50 to-blue-100'
        }`}
      ></div>
      
      {/* 패턴 오버레이 */}
      <div 
        className={`absolute inset-0 ${
          isAdminMode ? 'bg-pattern-admin opacity-10' : 'bg-pattern opacity-20'
        }`}
      ></div>
      
      {/* 원형 그라데이션 - 왼쪽 상단 */}
      <div 
        className={`absolute -top-40 -left-40 w-96 h-96 rounded-full ${
          isAdminMode 
            ? 'bg-gradient-to-r from-red-500/30 to-purple-500/20 blur-3xl' 
            : 'bg-gradient-to-r from-blue-400/30 to-indigo-400/20 blur-3xl'
        }`}
      ></div>
      
      {/* 원형 그라데이션 - 오른쪽 하단 */}
      <div 
        className={`absolute -bottom-32 -right-32 w-96 h-96 rounded-full ${
          isAdminMode 
            ? 'bg-gradient-to-r from-purple-600/20 to-red-600/30 blur-3xl' 
            : 'bg-gradient-to-r from-cyan-400/20 to-blue-500/30 blur-3xl'
        }`}
      ></div>
      
      {/* 물결 패턴 - 하단 */}
      <div className="absolute bottom-0 left-0 right-0 h-32 overflow-hidden">
        <svg 
          className={`absolute bottom-0 w-full h-48 ${
            isAdminMode ? 'text-gray-800/30' : 'text-white/30'
          }`} 
          viewBox="0 0 1200 120" 
          preserveAspectRatio="none"
        >
          <path 
            d="M0,0V46.29c47.79,22.2,103.59,32.17,158,28,70.36-5.37,136.33-33.31,206.8-37.5C438.64,32.43,512.34,53.67,583,72.05c69.27,18,138.3,24.88,209.4,13.08,36.15-6,69.85-17.84,104.45-29.34C989.49,25,1113-14.29,1200,52.47V0Z" 
            fill="currentColor"
          />
        </svg>
      </div>
      
      {/* 도트 그리드 */}
      <div className="absolute inset-0" style={{
        backgroundImage: `radial-gradient(${isAdminMode ? 'rgba(255, 255, 255, 0.08)' : 'rgba(0, 0, 0, 0.05)'} 1px, transparent 1px)`,
        backgroundSize: '30px 30px'
      }}></div>
    </div>
  );
};

export default BackgroundPattern;