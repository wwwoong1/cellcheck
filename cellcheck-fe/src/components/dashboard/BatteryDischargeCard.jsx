import React from 'react';
import { Battery, Activity, Badge, CircuitBoard } from 'lucide-react';

const BatteryDischargeCard = ({ batteryData = null, isAdminMode = false, inspectionData = null }) => {
  // 관리자 모드에서는 표시하지 않음
  if (isAdminMode) {
    return null;
  }
  
  // 스타일 설정
  const bgColor = isAdminMode ? 'bg-gray-800' : 'bg-white';
  const textColor = isAdminMode ? 'text-gray-300' : 'text-gray-800';
  const titleColor = isAdminMode ? 'text-gray-200' : 'text-gray-800';
  const borderColor = isAdminMode ? 'border-gray-700' : 'border-gray-200';
  const cardBgColor = isAdminMode ? 'bg-gray-700' : 'bg-gray-50';
  
  // 배터리 타입 매핑
  const batteryTypeMap = {
    'AAA': '초소형 (AAA)',
    'AA': '소형 (AA)',
    'C': '중형 (C)',
    'D': '대형 (D)',
  };
  
  // 데이터 필드 안전하게 접근
  const getBatteryType = () => {
    if (!batteryData) return '';
    return batteryData.batteryType || batteryData.battery_type || '';
  };
  
  const getSoC = () => {
    if (!batteryData) return 0;
    // SoC 또는 soc로 접근 시도
    const socValue = batteryData.SoC !== undefined ? batteryData.SoC : 
                     (batteryData.soc !== undefined ? batteryData.soc : 0);
    return Number(socValue);
  };
  
  const getCircuitType = () => {
    if (!batteryData) return '';
    return batteryData.circuitType || batteryData.circuit_type || '';
  };
  
  // 데이터가 유효한지 확인
  const isValidData = () => {
    if (!batteryData) return false;
    
    // 데이터가 있고 타입이 discharge인지 확인
    const hasType = batteryData.type === 'discharge';
    
    // SoC 값이 있는지 확인 
    const hasSoC = getSoC() !== undefined && getSoC() !== null;
    
    return hasType || hasSoC;  // 둘 중 하나라도 충족하면 유효
  };
  
  // SoC 값에 따른 배터리 색상 및 채움
  const getBatteryFillColor = (soc) => {
    if (soc > 70) return isAdminMode ? 'bg-green-500' : 'bg-green-500';
    if (soc > 30) return isAdminMode ? 'bg-yellow-500' : 'bg-yellow-500';
    if (soc > 5) return isAdminMode ? 'bg-orange-500' : 'bg-orange-500';
    return isAdminMode ? 'bg-red-500' : 'bg-red-500';
  };

  // 방전 상태 텍스트
  const getDischargeStatus = (soc) => {
    if (soc <= 5) return '방전 완료';
    if (soc <= 20) return '방전 종료 단계';
    if (soc <= 50) return '방전 중';
    return '방전 시작';
  };
  
  // SoC 표시 가중치 적용 (5% 이하는 0%로 표시)
  const getDisplaySoC = (soc) => {
    if (soc <= 5) return 0;
    // 5%~100% 범위를 좀 더 자연스럽게 분포
    return Math.round(soc);
  };
  
  // 진행 바에 표시할 SoC (시각적 효과)
  const getProgressBarSoC = (soc) => {
    if (soc <= 5) return 0;
    // 더 자연스러운 시각효과를 위한 비선형 변환 (로그 스케일)
    return Math.max(5, Math.round(soc));
  };
  
  // 회로 타입 아이콘 색상
  const getCircuitColor = (circuitType) => {
    switch (circuitType) {
      case 'A':
        return isAdminMode ? 'text-blue-400' : 'text-blue-600';
      case 'B':
        return isAdminMode ? 'text-green-400' : 'text-green-600';
      default:
        return isAdminMode ? 'text-gray-400' : 'text-gray-600';
    }
  };
  
  // 배터리 타입 표시 색상
  const getBatteryTypeColor = (type) => {
    switch (type) {
      case 'AAA':
        return isAdminMode ? 'bg-blue-900 bg-opacity-50 text-blue-300' : 'bg-blue-100 text-blue-800';
      case 'AA':
        return isAdminMode ? 'bg-green-900 bg-opacity-50 text-green-300' : 'bg-green-100 text-green-800';
      case 'C':
        return isAdminMode ? 'bg-purple-900 bg-opacity-50 text-purple-300' : 'bg-purple-100 text-purple-800';
      case 'D':
        return isAdminMode ? 'bg-yellow-900 bg-opacity-50 text-yellow-300' : 'bg-yellow-100 text-yellow-800';
      default:
        return isAdminMode ? 'bg-gray-600 text-gray-300' : 'bg-gray-200 text-gray-800';
    }
  };
  
  // 외관 검사 결과에 따라 방전 카드 표시 결정
  const shouldShowDischargeData = () => {
    // 외관 검사 데이터가 있고 불량이 아닌 경우에만 방전 데이터 표시
    if (inspectionData && inspectionData.appearanceInspection === false && batteryData) {
      // 방전 데이터가 없거나 방전 데이터가 검사 데이터보다 이후에 생성된 경우에만 표시
      // 검사 데이터가 방전 데이터보다 더 최신인 경우에는 방전 데이터를 표시하지 않음
      if (batteryData.timestamp && inspectionData.timestamp) {
        const dischargeTime = new Date(batteryData.timestamp).getTime();
        const inspectionTime = new Date(inspectionData.timestamp).getTime();
        return dischargeTime >= inspectionTime;
      }
      return true;
    }
    return false;
  };
  
  // inspectionData가 있고 방전 데이터가 없거나 검사 결과가 불량이면 대기 메시지 표시
  // 또는 검사 데이터가 방전 데이터보다 더 최신인 경우에도 대기 메시지 표시
  const showWaitingMessage = () => {
    // 기본 조건: 방전 데이터가 없거나, 유효하지 않거나, 검사 결과가 불량인 경우
    if (!batteryData || !isValidData() || (inspectionData && inspectionData.appearanceInspection === true)) {
      return true;
    }
    
    // 추가 조건: 검사 데이터와 방전 데이터가 모두 있는 경우, 타임스탬프 비교
    if (inspectionData && batteryData && 
        inspectionData.timestamp && batteryData.timestamp) {
      const dischargeTime = new Date(batteryData.timestamp).getTime();
      const inspectionTime = new Date(inspectionData.timestamp).getTime();
      // 검사 데이터가 방전 데이터보다 더 최신이면 대기 메시지 표시
      return inspectionTime > dischargeTime;
    }
    
    return false;
  };
  
  // 현재 SoC 값
  const currentSoC = getSoC();
  const batteryType = getBatteryType();
  const circuitType = getCircuitType();
  
  return (
    <div className={`${bgColor} p-6 rounded-lg shadow-md ${isAdminMode ? '' : 'border'} ${borderColor}`}>
      <div className="flex items-center mb-4">
        <Activity className={`h-5 w-5 ${isAdminMode ? 'text-green-400' : 'text-green-500'} mr-2`} />
        <h3 className={`text-lg font-medium ${titleColor}`}>실시간 배터리 방전 상태</h3>
      </div>
      
      {!showWaitingMessage() ? (
        <div className="flex flex-col items-center">
  
          
          {/* 방전 상태 */}
          <div className="mb-3 flex items-center justify-center">
            <span className={`${currentSoC <= 5 ? 'text-red-500 font-bold' : textColor}`}>
              {getDischargeStatus(currentSoC)}
            </span>
            {currentSoC <= 5 && (
              <span className="ml-2 px-2 py-0.5 bg-red-100 text-red-800 text-xs rounded-full animate-pulse">
                폐건전지함 이동 대기
              </span>
            )}
          </div>
          
          {/* 잔량 */}
          <div className="mb-4 w-full">
            <div className="flex justify-between items-center mb-1">
              <span className={textColor}>배터리 잔량 (SoC)</span>
              <span className={`font-semibold ${currentSoC <= 5 ? 'text-red-500' : textColor}`}>
                {getDisplaySoC(currentSoC)}%
              </span>
            </div>
            
            {/* 배터리 시각화 */}
            <div className="relative h-6 w-full bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
              <div 
                className={`h-full ${getBatteryFillColor(currentSoC)} transition-all duration-500 ease-out`} 
                style={{ width: `${getProgressBarSoC(currentSoC)}%` }}
              ></div>
            </div>
          </div>
          
          {/* 회로 정보 */}
          <div className={`${cardBgColor} p-4 rounded-lg w-full flex justify-between items-center`}>
            <div className="flex items-center">
              <CircuitBoard className={`h-5 w-5 mr-2 ${getCircuitColor(circuitType)}`} />
              <span className={textColor}>작동 회로:</span>
            </div>
            <span className={`font-semibold ${getCircuitColor(circuitType)}`}>
              {circuitType === 'A' && '회로 A'}
              {circuitType === 'B' && '회로 B'}
              {(!circuitType || circuitType === 'O') && '대기 중'}
            </span>
          </div>
        </div>
      ) : (
        <div className={`${cardBgColor} p-6 rounded-lg text-center`}>
          <p className={isAdminMode ? "text-gray-400" : "text-gray-500"}>
            방전 대기 중... 외관 검사 진행 후 이상 없을 시 방전이 시작됩니다.
          </p>
        </div>
      )}
    </div>
  );
};

export default BatteryDischargeCard;