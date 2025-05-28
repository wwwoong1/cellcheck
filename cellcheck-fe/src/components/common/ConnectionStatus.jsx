// src/components/common/ConnectionStatus.jsx
import React from 'react';
import { Wifi, WifiOff } from 'lucide-react';

const ConnectionStatus = ({ connected, isAdminMode, compact = false }) => {
  // 색상 설정
  const bgColor = connected 
    ? isAdminMode ? 'bg-green-900 bg-opacity-30' : 'bg-green-100' 
    : isAdminMode ? 'bg-red-900 bg-opacity-30' : 'bg-red-100';
  
  const textColor = connected 
    ? isAdminMode ? 'text-green-400' : 'text-green-800' 
    : isAdminMode ? 'text-red-400' : 'text-red-800';
  
  const indicatorColor = connected ? 'bg-green-500' : 'bg-red-500';
  
  return (
    <div className={`${bgColor} ${textColor} px-2 py-1 rounded-full flex items-center text-xs`}>
      {compact ? (
        <div className={`w-2 h-2 rounded-full ${indicatorColor}`}></div>
      ) : (
        <>
          {connected ? (
            <Wifi className="w-3 h-3 mr-1" />
          ) : (
            <WifiOff className="w-3 h-3 mr-1" />
          )}
          <span>{connected ? '실시간 연결됨' : '연결 끊김'}</span>
        </>
      )}
    </div>
  );
};

export default ConnectionStatus;