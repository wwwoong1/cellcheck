// src/context/WebSocketContext.jsx
import React, { createContext, useContext, useState, useEffect } from 'react';
import useWebSocket from '../hooks/useWebSocket';

const WebSocketContext = createContext(null);

// 애플리케이션 전체에서 사용할 글로벌 WebSocket 상태를 관리하는 Provider
export function WebSocketProvider({ children }) {
  // WebSocket 서버 URL 설정
  const getWebSocketUrl = () => {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.host;
    return `${protocol}//${host}/api/ws/mqtt`;
  };
  
  // 개발 환경에서는 지정된 URL 사용
  const devWsUrl = 'wss://k12d106.p.ssafy.io/api/ws/mqtt';
  
  // 환경에 따라 URL 선택
  const wsUrl = process.env.NODE_ENV === 'production' ? getWebSocketUrl() : devWsUrl;
  
  // useWebSocket 훅 사용 - 서버 실행 시점부터 연결 유지
  const wsData = useWebSocket(wsUrl);
  
  // 현재 진행 중인 배터리 ID 추적
  const [currentBatteryId, setCurrentBatteryId] = useState(null);
  
  // 시계열 데이터 상태 구성 - useWebSocket에서 제공하는 accumulatedData 사용
  // 이 데이터는 localStorage에 저장되어 새로고침해도 유지됨
  const timeSeriesData = wsData.accumulatedData || {
    environment: [],
    system: [],
    discharge: [],
    appearance: []
  };

  // 현재 활성화된 프로세스 추적 (검사 또는 방전)
  const [activeProcess, setActiveProcess] = useState(null); // 'inspection' 또는 'discharge'
  
  // 최신 검사 데이터와 방전 데이터를 별도로 관리
  const [currentInspectionData, setCurrentInspectionData] = useState(null);
  const [currentDischargeData, setCurrentDischargeData] = useState(null);
  
  // 데이터 업데이트 시 현재 상태 처리
  useEffect(() => {
    if (wsData.data) {
      Object.entries(wsData.data).forEach(([type, data]) => {
        if (data && Object.keys(data).length > 0) {
          // 배터리 ID 추출
          const batteryId = data.battery_id || data.batteryId;
          
          // 외관검사 데이터 처리
          if (type === 'appearance') {
            // 현재 검사 데이터 업데이트
            setCurrentInspectionData(data);
            
            // 항상 외관검사 데이터가 들어오면 방전 데이터 초기화
            setCurrentDischargeData(null);
            
            // 활성 프로세스를 검사로 변경
            setActiveProcess('inspection');
            
            // 새로운 배터리 ID면 방전 데이터 초기화
            if (batteryId && batteryId !== currentBatteryId) {
              setCurrentBatteryId(batteryId);
            }
          }
          
          // 방전 데이터 처리
          else if (type === 'discharge') {
            // 검사 데이터가 있고, 검사 결과가 정상(false)인 경우에만 방전 데이터 처리
            if (currentInspectionData && currentInspectionData.appearanceInspection === false) {
              // 현재 방전 데이터 업데이트
              setCurrentDischargeData(data);
              
              // 활성 프로세스를 방전으로 변경
              setActiveProcess('discharge');
              
              // SoC가 5% 이하면 방전 완료로 간주, 다음 검사를 위해 상태 재설정
              const soc = data.SoC !== undefined ? data.SoC : 
                         (data.soc !== undefined ? data.soc : null);
              
              if (soc !== null && soc <= 5) {
                // 1초 후 방전 데이터 초기화 (UI에서 방전 완료 메시지를 잠시 보여주기 위해)
                setTimeout(() => {
                  setCurrentDischargeData(null);
                  setActiveProcess('inspection'); // 검사 프로세스로 전환하여 다음 배터리 대기
                }, 1000);
              }
            }
          }
        }
      });
    }
  }, [wsData.data, currentBatteryId, currentInspectionData, activeProcess]);

  // 확장된 컨텍스트 값 제공
  const contextValue = {
    ...wsData,
    timeSeriesData,
    currentBatteryId,
    currentInspectionData,
    currentDischargeData,
    activeProcess
  };
  
  return (
    <WebSocketContext.Provider value={contextValue}>
      {children}
    </WebSocketContext.Provider>
  );
}

export function useWebSocketContext() {
  const context = useContext(WebSocketContext);
  if (!context) {
    throw new Error('useWebSocketContext must be used within a WebSocketProvider');
  }
  return context;
}