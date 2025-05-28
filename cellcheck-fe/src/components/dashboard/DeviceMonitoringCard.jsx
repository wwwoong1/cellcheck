import React, { useState, useEffect } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import { Cpu, HardDrive, Thermometer } from 'lucide-react';

const DeviceMonitoringCard = ({ systemData = [], isAdminMode = true }) => {
  
  // 누적 데이터 상태 관리
  const [timeSeriesData, setTimeSeriesData] = useState([]);
  
  // 데이터가 배열이 아닌 경우 처리 (객체를 배열로 변환)
  const dataArray = Array.isArray(systemData) ? systemData : [systemData];
  
  // 현재 디바이스 상태 (객체인 경우 바로 사용)
  const latestData = Array.isArray(systemData) ? 
    (dataArray.length > 0 ? dataArray[dataArray.length - 1] : null) : 
    systemData;
  
  // 새 데이터가 들어올 때마다 시계열 데이터 업데이트
  useEffect(() => {
    if (latestData) {
      // 현재 시간 추가
      const dataWithTimestamp = {
        ...latestData,
        recorded_at: latestData.recorded_at || new Date().toISOString(),
        // 소수점 처리 추가
        cpu_percent: parseFloat(parseFloat(latestData.cpu_percent || 0).toFixed(1)),
        mem_used: parseFloat(parseFloat(latestData.mem_used || 0).toFixed(1)),
        cpu_temp: parseFloat(parseFloat(latestData.cpu_temp || 0).toFixed(1)),
        soc_temp: parseFloat(parseFloat(latestData.soc_temp || 0).toFixed(1))
      };
      
      // 새로운 데이터를 배열에 추가
      setTimeSeriesData(prevData => {
        // 최대 30개 데이터 포인트만 유지
        const newData = [...prevData, dataWithTimestamp];
        if (newData.length > 30) {
          return newData.slice(newData.length - 30);
        }
        return newData;
      });
    }
  }, [latestData]);
  
  // 시간 포맷팅 함수
  const formatTime = (timestamp) => {
    if (!timestamp) return '현재';
    return new Date(timestamp).toLocaleTimeString('ko-KR', {
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit',
      hour12: false
    });
  };
  
  // 값 포맷팅 함수
  const formatValue = (value, unit = '') => {
    if (value === undefined || value === null) return `0${unit}`;
    const parsedValue = parseFloat(value);
    if (isNaN(parsedValue)) return `0${unit}`;
    return `${parsedValue.toFixed(1)}${unit}`;
  };
  
  // 상태 색상 결정 함수
  const getStatusClass = (value, low, high) => {
    if (value === undefined || value === null) return isAdminMode ? 'text-gray-400' : 'text-gray-400';
    const parsedValue = parseFloat(value);
    if (isNaN(parsedValue)) return isAdminMode ? 'text-gray-400' : 'text-gray-400';
    if (parsedValue < low) return isAdminMode ? 'text-blue-400' : 'text-blue-500';
    if (parsedValue > high) return isAdminMode ? 'text-red-400' : 'text-red-500';
    return isAdminMode ? 'text-green-400' : 'text-green-500';
  };
  
  // 스타일 설정
  const bgColor = isAdminMode ? 'bg-gray-800' : 'bg-white';
  const textColor = isAdminMode ? 'text-gray-300' : 'text-gray-800';
  const titleColor = isAdminMode ? 'text-gray-200' : 'text-gray-800';
  const borderColor = isAdminMode ? 'border-gray-700' : 'border-gray-200';
  const cardBgColor = isAdminMode ? 'bg-gray-700' : 'bg-gray-50';
  const gridColor = isAdminMode ? '#4b5563' : '#e5e7eb';
  
  // 차트 공통 스타일
  const chartStyle = {
    backgroundColor: isAdminMode ? '#374151' : '#f9fafb', // gray-700 vs gray-50
    borderRadius: '0.5rem', 
    padding: '1rem'
  };
  
  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
      {/* CPU & 메모리 사용률 차트 */}
      <div className={`${bgColor} rounded-xl shadow-md p-4 ${isAdminMode ? '' : 'border'} ${borderColor}`}>
        <div className="flex items-center mb-3">
          <Cpu className={`h-5 w-5 ${isAdminMode ? 'text-blue-400' : 'text-blue-500'} mr-2`} />
          <h3 className={`text-lg font-medium ${titleColor}`}>CPU & 메모리 사용률</h3>
        </div>
        
        <div style={chartStyle}>
          <ResponsiveContainer width="100%" height={250}>
            <LineChart data={systemData}>
              <XAxis 
                dataKey="recorded_at" 
                tick={{ fontSize: 10 }}
                stroke={isAdminMode ? '#6b7280' : '#9ca3af'}
                tickFormatter={formatTime}
              />
              <YAxis 
                unit="%" 
                domain={[0, 100]}
                stroke={isAdminMode ? '#6b7280' : '#9ca3af'}
              />
              <CartesianGrid stroke={gridColor} strokeDasharray="3 3" />
              <Tooltip 
                labelFormatter={(label) => `시간: ${formatTime(label)}`}
                formatter={(value) => [`${parseFloat(value).toFixed(1)}%`, '']}
                contentStyle={{
                  backgroundColor: isAdminMode ? '#1f2937' : '#fff',
                  border: `1px solid ${isAdminMode ? '#374151' : '#e5e7eb'}`,
                  color: isAdminMode ? '#e5e7eb' : '#111827'
                }}
              />
              <Legend />
              <Line 
                type="monotone" 
                dataKey="cpu_percent" 
                name="CPU 사용률" 
                stroke="#3b82f6" 
                activeDot={{ r: 8 }} 
                connectNulls={true}
                isAnimationActive={true}
              />
              <Line 
                type="monotone" 
                dataKey="mem_used" 
                name="메모리 사용률" 
                stroke="#10b981" 
                connectNulls={true}
                isAnimationActive={true}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
        
        {latestData && (
          <div className="mt-4 grid grid-cols-2 gap-4 text-center">
            <div className={`${cardBgColor} p-3 rounded shadow`}>
              <h4 className="text-sm text-gray-500">CPU 현재 사용률</h4>
              <p className={`text-lg font-semibold ${getStatusClass(latestData.cpu_percent, 30, 75)}`}>
                {formatValue(latestData.cpu_percent, '%')}
              </p>
            </div>
            <div className={`${cardBgColor} p-3 rounded shadow`}>
              <h4 className="text-sm text-gray-500">메모리 현재 사용률</h4>
              <p className={`text-lg font-semibold ${getStatusClass(latestData.mem_used, 30, 75)}`}>
                {formatValue(latestData.mem_used, '%')}
              </p>
            </div>
          </div>
        )}
      </div>
      
      {/* 온도 모니터링 차트 */}
      <div className={`${bgColor} rounded-xl shadow-md p-4 ${isAdminMode ? '' : 'border'} ${borderColor}`}>
        <div className="flex items-center mb-3">
          <Thermometer className={`h-5 w-5 ${isAdminMode ? 'text-red-400' : 'text-red-500'} mr-2`} />
          <h3 className={`text-lg font-medium ${titleColor}`}>CPU & SOC 온도</h3>
        </div>
        
        <div style={chartStyle}>
          <ResponsiveContainer width="100%" height={250}>
            <LineChart data={systemData}>
              <XAxis 
                dataKey="recorded_at" 
                tick={{ fontSize: 10 }}
                stroke={isAdminMode ? '#6b7280' : '#9ca3af'}
                tickFormatter={formatTime}
              />
              <YAxis 
                unit="°C"
                stroke={isAdminMode ? '#6b7280' : '#9ca3af'}
                domain={['auto', 'auto']}
              />
              <CartesianGrid stroke={gridColor} strokeDasharray="3 3" />
              <Tooltip 
                labelFormatter={(label) => `시간: ${formatTime(label)}`}
                formatter={(value) => [`${parseFloat(value).toFixed(1)}°C`, '']}
                contentStyle={{
                  backgroundColor: isAdminMode ? '#1f2937' : '#fff',
                  border: `1px solid ${isAdminMode ? '#374151' : '#e5e7eb'}`,
                  color: isAdminMode ? '#e5e7eb' : '#111827'
                }}
              />
              <Legend />
              <Line 
                type="monotone" 
                dataKey="cpu_temp" 
                name="CPU 온도" 
                stroke="#f87171" 
                activeDot={{ r: 8 }} 
                connectNulls={true}
                isAnimationActive={true}
              />
              <Line 
                type="monotone" 
                dataKey="soc_temp" 
                name="SOC 온도" 
                stroke="#8b5cf6" 
                connectNulls={true}
                isAnimationActive={true}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
        
        {latestData && (
          <div className="mt-4 grid grid-cols-2 gap-4 text-center">
            <div className={`${cardBgColor} p-3 rounded shadow`}>
              <h4 className="text-sm text-gray-500">CPU 현재 온도</h4>
              <p className={`text-lg font-semibold ${getStatusClass(latestData.cpu_temp, 40, 70)}`}>
                {formatValue(latestData.cpu_temp, '°C')}
              </p>
            </div>
            <div className={`${cardBgColor} p-3 rounded shadow`}>
              <h4 className="text-sm text-gray-500">SOC 현재 온도</h4>
              <p className={`text-lg font-semibold ${getStatusClass(latestData.soc_temp, 40, 70)}`}>
                {formatValue(latestData.soc_temp, '°C')}
              </p>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default DeviceMonitoringCard; 