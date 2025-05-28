// src/components/layout/Footer.jsx
import React from 'react';
import { Zap, Smartphone, Cpu, HelpCircle, Shield, Battery } from 'lucide-react';

const Footer = ({ 
  isAdminMode = false,
  systemStatus = { 
    serverStatus: 'ONLINE', 
    deviceCount: 42, 
    version: 'v1.0.0' 
  } 
}) => {
  const bgColorClass = isAdminMode 
    ? 'bg-gray-800 bg-opacity-90' 
    : 'bg-white bg-opacity-90';
    
  const borderColorClass = isAdminMode 
    ? 'border-gray-700' 
    : 'border-gray-200';
    
  const textColorClass = isAdminMode 
    ? 'text-gray-400' 
    : 'text-gray-600';
  
  const serverStatusColorClass = isAdminMode 
    ? 'text-red-500' 
    : 'text-green-500';
    
  const deviceColorClass = isAdminMode 
    ? 'text-blue-400' 
    : 'text-blue-500';
    
  const versionColorClass = isAdminMode 
    ? 'text-purple-400' 
    : 'text-purple-500';
    
  const helpColorClass = isAdminMode 
    ? 'text-red-400 hover:text-red-300' 
    : 'text-blue-600 hover:text-blue-500';

  return (
    <div className={`fixed bottom-0 left-0 right-0 ${bgColorClass} backdrop-blur-sm border-t ${borderColorClass} py-2 px-4 text-xs ${textColorClass} z-10 footer`}>
      <div className="w-full flex flex-col sm:flex-row justify-between items-center gap-2 sm:gap-0">
        <div className="flex flex-wrap items-center justify-center sm:justify-start gap-2 sm:gap-6">
          <div className="flex items-center gap-2">
            <Zap className={`h-3 w-3 ${serverStatusColorClass}`} />
            <span>
              {isAdminMode ? '관리자 모드: 활성화' : `서버 상태: ${systemStatus.serverStatus === 'ONLINE' ? '정상' : '점검중'}`}
            </span>
          </div>
          <div className="flex items-center gap-2">
            <Cpu className={`h-3 w-3 ${versionColorClass}`} />
            <span>시스템 버전: {systemStatus.version}</span>
          </div>
        </div>

        <div className="flex items-center gap-6 mt-1 sm:mt-0">
          <span>© 2025 스마트 배터리 시스템</span>
        </div>
      </div>
    </div>
  );
};

export default Footer;