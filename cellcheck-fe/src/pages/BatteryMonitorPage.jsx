import React, { useEffect } from 'react';
import { useWebSocketContext } from '../context/WebSocketContext';
import BatteryInspectionCard from '../components/dashboard/BatteryInspectionCard';
import BatteryDischargeCard from '../components/dashboard/BatteryDischargeCard';
import BackgroundPattern from '../components/layout/BackgroundPattern';
import ConnectionStatus from '../components/common/ConnectionStatus';

const BatteryMonitorPage = () => {
  const { connected, timeSeriesData, activeProcess, currentInspectionData, currentDischargeData } = useWebSocketContext();
  
  useEffect(() => {
    // 모니터링 페이지 접속 시 메시지 표시
    document.title = "배터리 모니터링 - 셀체크";
  }, []);
  
  return (
    <div className="min-h-screen bg-gray-100 p-4">
      {/* 배경 패턴 */}
      <BackgroundPattern isAdminMode={false} />
      
      {/* 상단 영역 - 제목과 연결 상태 */}
      <div className="flex justify-between items-center mb-6">
        <h1 className="text-2xl font-semibold text-gray-800">배터리 모니터링</h1>
        <ConnectionStatus connected={connected} isAdminMode={false} />
      </div>
      
      {/* 배터리 카드 영역 */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mb-6">
        <BatteryInspectionCard 
          inspectionData={currentInspectionData} 
          dischargeData={currentDischargeData} 
          isAdminMode={false} 
        />
        
        <BatteryDischargeCard 
          batteryData={currentDischargeData} 
          inspectionData={currentInspectionData} 
          isAdminMode={false} 
        />
      </div>
      
      {/* 안내 메시지 */}
      <div className="bg-white p-4 rounded-lg shadow-sm border border-gray-200 text-center">
        <p className="text-gray-600">
          {connected 
            ? "WebSocket 연결됨 - 실시간 데이터를 표시합니다" 
            : "WebSocket 연결 시도 중... 잠시만 기다려주세요"}
        </p>
        {activeProcess && (
          <p className="text-green-600 mt-2">
            현재 활성 프로세스: {activeProcess}
          </p>
        )}
      </div>
    </div>
  );
};

export default BatteryMonitorPage; 