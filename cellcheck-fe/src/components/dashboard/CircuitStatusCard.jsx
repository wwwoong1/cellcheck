import React, { useEffect, useState } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import { Thermometer, Fan, AlertTriangle, Battery, Bell } from 'lucide-react';

const CircuitStatusCard = ({ environmentData = [], selectedDevice, isAdminMode = true }) => {
  // 차트 데이터 상태 추가
  const [chartData, setChartData] = useState([]);

  // 온도 경고 기준
  const TEMP_WARNING = 45; // 45도 이상이면 경고
  const TEMP_DANGER = 55;  // 55도 이상이면 위험
  
  // 스타일 설정
  const bgColor = isAdminMode ? 'bg-gray-800' : 'bg-white';
  const textColor = isAdminMode ? 'text-gray-300' : 'text-gray-800';
  const titleColor = isAdminMode ? 'text-gray-200' : 'text-gray-800';
  const borderColor = isAdminMode ? 'border-gray-700' : 'border-gray-200';
  const cardBgColor = isAdminMode ? 'bg-gray-700' : 'bg-gray-50';
  
  // 최신 데이터 추출
  const latestData = environmentData.length > 0 ? environmentData[environmentData.length - 1] : null;
  
  // 시간 포맷팅 함수
  const formatTime = (timestamp) => {
    if (!timestamp) return '시간 정보 없음';
    
    try {
      // 문자열이 아니면 문자열로 변환
      const timestampStr = typeof timestamp === 'string' ? timestamp : String(timestamp);
      
      // ISO 형식의 문자열이면 Date 객체로 변환
      let date;
      if (timestampStr.includes('T') || timestampStr.includes('-')) {
        date = new Date(timestampStr);
      } else {
        // 숫자로된 timestamp면 숫자로 변환 후 Date 객체 생성
        date = new Date(Number(timestampStr));
      }
      
      // UTC 시간에 9시간을 더해 KST로 변환
      date = new Date(date.getTime() + 9 * 60 * 60 * 1000);
      
      // 한국 시간 형식으로 포맷팅
      return date.toLocaleTimeString('ko-KR', {
        hour: '2-digit', 
        minute: '2-digit', 
        second: '2-digit', 
        hour12: false
      });
    } catch (error) {
      console.error("시간 변환 오류:", error);
      return String(timestamp).slice(0, 8);
    }
  };

  // 안전한 데이터 접근 함수들
  const getIsFullError = () => {
    if (!latestData) return 0;
    // 다양한 속성명 확인
    if (latestData.isFullError !== undefined) return Number(latestData.isFullError) === 0 ? 1 : 0;
    if (latestData.is_fullerror !== undefined) return Number(latestData.is_fullerror) === 0 ? 1 : 0;
    return 0;
  };
  
  const getIsFullNormal = () => {
    if (!latestData) return 0;
    if (latestData.isFullNormal !== undefined) return Number(latestData.isFullNormal) === 0 ? 1 : 0;
    if (latestData.is_fullnormal !== undefined) return Number(latestData.is_fullnormal) === 0 ? 1 : 0;
    return 0;
  };
  
  const getResistanceTempA = () => {
    if (!latestData) return "N/A";
    if (latestData.resistanceTemperatureA !== undefined) return latestData.resistanceTemperatureA;
    if (latestData.resistance_tempA !== undefined) return latestData.resistance_tempA;
    return "N/A";
  };
  
  const getResistanceTempB = () => {
    if (!latestData) return "N/A";
    if (latestData.resistanceTemperatureB !== undefined) return latestData.resistanceTemperatureB;
    if (latestData.resistance_tempB !== undefined) return latestData.resistance_tempB;
    return "N/A";
  };
  
  const getIndoorTemp = () => {
    if (!latestData) return "N/A";
    if (latestData.ambientTemp !== undefined) return latestData.ambientTemp;
    if (latestData.ambient_temp !== undefined) return latestData.ambient_temp;
    if (latestData.indoorTemperature !== undefined) return latestData.indoorTemperature;
    if (latestData.indoor_temp !== undefined) return latestData.indoor_temp;
    if (latestData.room_temp !== undefined) return latestData.room_temp;
    return "N/A";
  };
  
  const getCoolingFanA = () => {
    if (!latestData) return 0;
    if (latestData.coolingFanA !== undefined) return Number(latestData.coolingFanA) === 1;
    if (latestData.cooling_fanA !== undefined) return Number(latestData.cooling_fanA) === 1;
    return false;
  };
  
  const getCoolingFanB = () => {
    if (!latestData) return 0;
    if (latestData.coolingFanB !== undefined) return Number(latestData.coolingFanB) === 1;
    if (latestData.cooling_fanB !== undefined) return Number(latestData.cooling_fanB) === 1;
    return false;
  };
  
  // 차트용 데이터 처리
  const processDataForChart = (data) => {
    if (!Array.isArray(data) || data.length === 0) return [];
    
    // 최대 20개의 최신 데이터만 사용 (그래프가 너무 복잡해지지 않도록)
    const recentData = data.slice(-20);
    
    return recentData.map(item => ({
      timestamp: item.sent_at || item.recorded_at || item.timestamp || new Date().toISOString(),
      resistance_tempA: parseFloat(Number(item.resistanceTemperatureA || item.resistance_tempA || 0).toFixed(1)),
      resistance_tempB: parseFloat(Number(item.resistanceTemperatureB || item.resistance_tempB || 0).toFixed(1))
    }));
  };
  
  // environmentData가 변경될 때마다 차트 데이터 업데이트
  useEffect(() => {
    setChartData(processDataForChart(environmentData));
  }, [environmentData]);
  
  // 온도 상태에 따른 스타일 반환
  const getTempStatusStyle = (temp) => {
    const tempValue = parseFloat(temp);
    if (isNaN(tempValue)) return isAdminMode ? "text-gray-400" : "text-gray-600";
    if (tempValue >= TEMP_DANGER) {
      return isAdminMode ? "text-red-400 font-bold" : "text-red-600 font-bold";
    } else if (tempValue >= TEMP_WARNING) {
      return isAdminMode ? "text-yellow-400 font-bold" : "text-yellow-600 font-bold";
    }
    return isAdminMode ? "text-green-400" : "text-green-600";
  };

  // 온도 값 포맷팅
  const formatTemp = (temp) => {
    if (temp === "N/A") return "N/A";
    const tempValue = parseFloat(temp);
    if (isNaN(tempValue)) return "0.0";
    return tempValue.toFixed(1);
  };
  
  // 차트 공통 스타일 (관리자/사용자 동일하게)
  const chartStyle = {
    backgroundColor: isAdminMode ? '#374151' : '#f9fafb', // gray-700 vs gray-50
    borderRadius: '0.5rem', 
    padding: '1rem'
  };
  
  return (
    <div className={`${bgColor} p-6 rounded-xl shadow-md mt-6 ${isAdminMode ? '' : 'border'} ${borderColor}`}>
      {!isAdminMode && (
        <>
          <div className="flex items-center justify-between mb-4">
            <h2 className={`text-lg font-semibold ${titleColor}`}>
              배터리 통 상태 확인
            </h2>
            {(getIsFullError() === 1 || getIsFullNormal() === 1) && (
              <div className="flex items-center">
                <Bell className="h-5 w-5 text-red-500 mr-1 animate-pulse" />
                <span className="text-red-500 font-semibold">배터리 통 상태 확인 필요!</span>
              </div>
            )}
          </div>

          <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mb-6">
            {/* 배터리 통 상태 섹션 */}
            <div className={`${isAdminMode ? 'bg-gray-700' : 'bg-gray-100'} rounded-xl p-4 border ${
              (getIsFullError() === 1 || getIsFullNormal() === 1) 
                ? 'border-red-500' 
                : isAdminMode ? 'border-gray-600' : 'border-gray-300'
            }`}>
              <h3 className={`font-semibold mb-3 ${textColor} flex items-center`}>
                <Battery className={`h-5 w-5 mr-2 ${
                  (getIsFullError() === 1 || getIsFullNormal() === 1)
                    ? 'text-red-500'
                    : isAdminMode ? 'text-blue-400' : 'text-blue-600'
                }`} />
                배터리 통 상태
              </h3>
              
              <div className="grid grid-cols-1 gap-4">
                {/* 불량통 상태 */}
                <div className={`p-3 rounded-lg ${
                  getIsFullError() === 1 
                    ? isAdminMode ? 'bg-red-900 bg-opacity-50' : 'bg-red-100' 
                    : isAdminMode ? 'bg-gray-800' : 'bg-white'
                }`}>
                  <div className="flex justify-between items-center">
                    <span className={textColor}>불량통 상태:</span>
                    <span className={`font-semibold ${
                      getIsFullError() === 1
                        ? "text-red-500 animate-pulse"
                        : isAdminMode ? "text-green-400" : "text-green-600"
                    }`}>
                      {getIsFullError() === 1 ? '가득 참 ⚠️' : '여유 있음'}
                    </span>
                  </div>
                  {getIsFullError() === 1 && (
                    <div className="mt-2 text-sm text-red-500 font-medium">
                      불량통을 비워주세요! 처리가 중단될 수 있습니다.
                    </div>
                  )}
                </div>
                
                {/* 정상통 상태 */}
                <div className={`p-3 rounded-lg ${
                  getIsFullNormal() === 1
                    ? isAdminMode ? 'bg-red-900 bg-opacity-50' : 'bg-red-100' 
                    : isAdminMode ? 'bg-gray-800' : 'bg-white'
                }`}>
                  <div className="flex justify-between items-center">
                    <span className={textColor}>방전완료통 상태:</span>
                    <span className={`font-semibold ${
                      getIsFullNormal() === 1
                        ? "text-red-500 animate-pulse"
                        : isAdminMode ? "text-green-400" : "text-green-600"
                    }`}>
                      {getIsFullNormal() === 1 ? '가득 참 ⚠️' : '여유 있음'}
                    </span>
                  </div>
                  {getIsFullNormal() === 1 && (
                    <div className="mt-2 text-sm text-red-500 font-medium">
                      방전완료통을 비워주세요! 처리가 중단될 수 있습니다.
                    </div>
                  )}
                </div>
              </div>
            </div>
            
            {/* 회로 상태 섹션 */}
            <div className={`${cardBgColor} rounded-xl p-4 border ${isAdminMode ? 'border-gray-600' : 'border-gray-300'}`}>
              <h3 className={`font-semibold mb-3 ${textColor} flex items-center`}>
                <Thermometer className={`h-5 w-5 mr-2 ${isAdminMode ? 'text-red-400' : 'text-red-600'}`} />
                회로 상태
              </h3>
              
              <div className="grid grid-cols-1 gap-3">
                {/* 실내 온도 추가 */}
                <div className="text-center p-2 rounded-lg bg-opacity-50 mb-2">
                  <div className="flex justify-center items-center mb-1">
                    <Thermometer className={`h-4 w-4 mr-1 ${isAdminMode ? 'text-purple-400' : 'text-purple-600'}`} />
                    <span className={textColor}>실내 온도:</span>
                  </div>
                  <span className={`font-medium ${getTempStatusStyle(getIndoorTemp())}`}>
                    {formatTemp(getIndoorTemp())}°C
                  </span>
                </div>
                
                {/* 회로 A 상태 */}
                <div className="text-center p-2 rounded-lg">
                  <div className="flex justify-center items-center mb-1">
                    <div className="w-2 h-2 rounded-full bg-red-500 mr-1"></div>
                    <span className={textColor}>회로 A 온도:</span>
                    <span className={`ml-2 ${getTempStatusStyle(getResistanceTempA())}`}>
                      {formatTemp(getResistanceTempA())}°C
                    </span>
                  </div>
                  <div className="flex justify-center items-center">
                    <Fan className={`h-4 w-4 mr-1 ${getCoolingFanA() ? 
                      isAdminMode ? 'text-green-400' : 'text-green-600' : 
                      isAdminMode ? 'text-gray-500' : 'text-gray-500'}`} />
                    <span className={`${textColor} mr-1`}>쿨링팬:</span>
                    <span className={getCoolingFanA() ? 
                      isAdminMode ? "text-green-400" : "text-green-600" : 
                      isAdminMode ? "text-gray-500" : "text-gray-500"}>
                      {getCoolingFanA() ? '작동 중' : '꺼짐'}
                    </span>
                  </div>
                </div>
                
                {/* 회로 B 상태 */}
                <div className="text-center p-2 rounded-lg">
                  <div className="flex justify-center items-center mb-1">
                    <div className="w-2 h-2 rounded-full bg-blue-500 mr-1"></div>
                    <span className={textColor}>회로 B 온도:</span>
                    <span className={`ml-2 ${getTempStatusStyle(getResistanceTempB())}`}>
                      {formatTemp(getResistanceTempB())}°C
                    </span>
                  </div>
                  <div className="flex justify-center items-center">
                    <Fan className={`h-4 w-4 mr-1 ${getCoolingFanB() ? 
                      isAdminMode ? 'text-green-400' : 'text-green-600' : 
                      isAdminMode ? 'text-gray-500' : 'text-gray-500'}`} />
                    <span className={`${textColor} mr-1`}>쿨링팬:</span>
                    <span className={getCoolingFanB() ? 
                      isAdminMode ? "text-green-400" : "text-green-600" : 
                      isAdminMode ? "text-gray-500" : "text-gray-500"}>
                      {getCoolingFanB() ? '작동 중' : '꺼짐'}
                    </span>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </>
      )}

      {/* 회로 온도 차트 - 관리자/사용자 모드 공통 */}
      <div className="mt-6">
        <h3 className={`font-semibold mb-3 ${titleColor} flex items-center`}>
          <Thermometer className={`h-5 w-5 mr-2 ${isAdminMode ? 'text-red-400' : 'text-red-600'}`} />
          회로 온도 추이
        </h3>
        
        <div style={chartStyle}>
          <ResponsiveContainer width="100%" height={250}>
            <LineChart data={chartData} margin={{top: 5, right: 30, left: 20, bottom: 5}}>
              <CartesianGrid strokeDasharray="3 3" stroke={isAdminMode ? "#4B5563" : "#E5E7EB"} />
              <XAxis
                dataKey="timestamp"
                tickFormatter={formatTime}
                stroke={isAdminMode ? "#9CA3AF" : "#6B7280"}
              />
              <YAxis
                stroke={isAdminMode ? "#9CA3AF" : "#6B7280"}
                domain={['auto', 'auto']}
                allowDataOverflow={false}
              />
              <Tooltip
                formatter={(value) => [`${value}°C`, '온도']}
                labelFormatter={(label) => `시간: ${formatTime(label)}`}
                contentStyle={{
                  backgroundColor: isAdminMode ? '#1F2937' : '#FFFFFF',
                  borderColor: isAdminMode ? '#374151' : '#E5E7EB'
                }}
              />
              <Legend />
              <Line
                type="monotone"
                dataKey="resistance_tempA"
                name="회로 A 온도"
                stroke="#EF4444"
                activeDot={{r: 8}}
                connectNulls={true}
                isAnimationActive={true}
              />
              <Line
                type="monotone"
                dataKey="resistance_tempB"
                name="회로 B 온도"
                stroke="#3B82F6"
                connectNulls={true}
                isAnimationActive={true}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>

      {/* 이전 표현 방식 (리스트형 표시는 유지) */}
      {isAdminMode && (
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mt-4">
          <div className={`${cardBgColor} p-4 rounded-lg`}>
            <div className="flex justify-between items-center">
              <div className="flex items-center">
                <div className="w-3 h-3 rounded-full bg-red-500 mr-2"></div>
                <span className={textColor}>회로 A 온도:</span>
              </div>
              <span className={getTempStatusStyle(getResistanceTempA())}>
                {formatTemp(getResistanceTempA())}°C
              </span>
            </div>
            <div className="flex justify-between items-center mt-2">
              <span className={textColor}>쿨링팬 상태:</span>
              <span className={getCoolingFanA() ? "text-green-400" : "text-gray-500"}>
                {getCoolingFanA() ? '작동 중' : '꺼짐'}
              </span>
            </div>
          </div>
          
          <div className={`${cardBgColor} p-4 rounded-lg`}>
            <div className="flex justify-between items-center">
              <div className="flex items-center">
                <div className="w-3 h-3 rounded-full bg-blue-500 mr-2"></div>
                <span className={textColor}>회로 B 온도:</span>
              </div>
              <span className={getTempStatusStyle(getResistanceTempB())}>
                {formatTemp(getResistanceTempB())}°C
              </span>
            </div>
            <div className="flex justify-between items-center mt-2">
              <span className={textColor}>쿨링팬 상태:</span>
              <span className={getCoolingFanB() ? "text-green-400" : "text-gray-500"}>
                {getCoolingFanB() ? '작동 중' : '꺼짐'}
              </span>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default CircuitStatusCard;