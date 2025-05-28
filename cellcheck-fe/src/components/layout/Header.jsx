// src/components/layout/Header.jsx
import React, { useState, useEffect } from 'react';
import { Battery, Clock, Shield, Menu } from 'lucide-react';
import { useLocation } from 'react-router-dom';

const Header = ({ 
  isAdminMode = false,
  systemStatus = { serverStatus: 'ONLINE' },
  onToggleSidebar = () => {} 
}) => {
  const [currentTime, setCurrentTime] = useState(new Date().toLocaleString());
  const location = useLocation();
  const isRootPath = location.pathname === '/';
  
  useEffect(() => {
    // 1초마다 현재 시간 업데이트
    const interval = setInterval(() => {
      setCurrentTime(new Date().toLocaleString());
    }, 1000);
    
    return () => clearInterval(interval);
  }, []);
  
  const bgColorClass = isAdminMode 
    ? 'bg-gray-800 bg-opacity-90' 
    : 'bg-white bg-opacity-90';
    
  const borderColorClass = isAdminMode 
    ? 'border-gray-700' 
    : 'border-gray-200';
    
  const textColorClass = isAdminMode 
    ? 'text-gray-100' 
    : 'text-gray-800';
    
  const accentColorClass = isAdminMode 
    ? 'text-red-500' 
    : 'text-blue-500';
    
  const badgeBgClass = isAdminMode
    ? 'bg-red-900 text-red-100'
    : 'bg-blue-100 text-blue-700';

  return (
    <div className={`fixed top-0 left-0 right-0 ${bgColorClass} backdrop-blur-sm border-b ${borderColorClass} py-2 px-4 z-10 header`}>
      <div className="w-full flex justify-between items-center">
        <div className="flex items-center">
          {onToggleSidebar !== undefined && !isRootPath && (
          <button 
            onClick={onToggleSidebar}
            className={`${accentColorClass} mr-3 p-1 rounded hover:bg-opacity-20 hover:bg-gray-400 hidden md:block`}
          >
            <Menu className="h-5 w-5" />
          </button>
          )}
          
          <Battery className={`h-5 w-5 ${accentColorClass} mr-2`} />
          <span className={`text-lg font-medium ${textColorClass}`}>스마트 배터리 시스템</span>
          <span className={`ml-2 px-2 py-0.5 ${badgeBgClass} rounded-full text-xs`}>
            {isAdminMode ? '관리자 접속' : '사용자 접속'}
          </span>
        </div>
        <div className={`flex items-center text-sm ${isAdminMode ? 'text-gray-400' : 'text-gray-600'}`}>
          <Clock className="h-4 w-4 mr-1" />
          <span>{currentTime}</span>
        </div>
      </div>
    </div>
  );
};

export default Header;