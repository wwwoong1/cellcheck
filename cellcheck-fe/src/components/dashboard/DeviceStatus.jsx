import React from 'react';
import { Cpu, Server, Thermometer, Fan } from 'lucide-react';

const DeviceStatus = ({ deviceData = [], selectedDevice = 1, onDeviceChange, isAdminMode = true }) => {
  // 선택된 장치의 최신 데이터 가져오기
  const deviceDataFiltered = deviceData.filter(d => d.device_id === selectedDevice);
  const latestData = deviceDataFiltered.length > 0 
    ? deviceDataFiltered[deviceDataFiltered.length - 1] 
    : null;
  
  // 고유 장치 ID 목록 생성
  const deviceIds = Array.from(new Set(deviceData.map(d => d.device_id))).sort();

  // 상태에 따른 색상 클래스 반환 함수
  const getStatusClass = (value, low, high) => {
    if (value < low) return { textColor: 'text-blue-500', bgColor: 'bg-blue-100 dark:bg-blue-900' };
    if (value > high) return { textColor: 'text-red-500', bgColor: 'bg-red-100 dark:bg-red-900' };
    return { textColor: 'text-green-500', bgColor: 'bg-green-100 dark:bg-green-900' };
  };

  return (
    <div className={`${isAdminMode ? 'bg-gray-800 text-gray-300' : 'bg-white text-gray-800'} rounded-xl shadow-md overflow-hidden`}>
      <div className="p-4 border-b border-gray-700 flex justify-between items-center">
        <div className="flex items-center">
          <Server className={`h-5 w-5 ${isAdminMode ? 'text-red-400' : 'text-blue-500'} mr-2`} />
          <h2 className="text-lg font-semibold">장치 상태 모니터링</h2>
        </div>
        
        <div className="flex items-center space-x-2">
          <span className="text-sm">장치 선택:</span>
          <select
            value={selectedDevice}
            onChange={(e) => onDeviceChange(Number(e.target.value))}
            className={`px-3 py-1 rounded border ${isAdminMode ? 'bg-gray-700 border-gray-600 text-gray-200' : 'bg-white border-gray-300 text-gray-800'}`}
          >
            {deviceIds.map((id) => (
              <option key={id} value={id}>
                Device {id}
              </option>
            ))}
          </select>
        </div>
      </div>
      
      {latestData ? (
        <div className="p-4">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
            {/* CPU 사용률 */}
            <div className={`p-4 rounded-xl ${isAdminMode ? 'bg-gray-700' : 'bg-gray-50'}`}>
              <div className="flex items-center justify-between mb-2">
                <h3 className="text-sm text-gray-500 dark:text-gray-400">CPU 사용률</h3>
                <Cpu className="h-4 w-4 text-blue-500" />
              </div>
              <div className="flex items-center">
                <div className={`w-full bg-gray-200 dark:bg-gray-600 rounded-full h-2.5 mr-2`}>
                  <div 
                    className={`h-2.5 rounded-full ${
                      latestData.cpu_percent > 75 ? 'bg-red-500' : 
                      latestData.cpu_percent > 50 ? 'bg-yellow-500' : 
                      'bg-green-500'
                    }`}
                    style={{ width: `${latestData.cpu_percent}%` }}
                  ></div>
                </div>
                <span className="text-lg font-bold">{latestData.cpu_percent}%</span>
              </div>
            </div>
            
            {/* 메모리 사용률 */}
            <div className={`p-4 rounded-xl ${isAdminMode ? 'bg-gray-700' : 'bg-gray-50'}`}>
              <div className="flex items-center justify-between mb-2">
                <h3 className="text-sm text-gray-500 dark:text-gray-400">메모리 사용률</h3>
                <Server className="h-4 w-4 text-purple-500" />
              </div>
              <div className="flex items-center">
                <div className={`w-full bg-gray-200 dark:bg-gray-600 rounded-full h-2.5 mr-2`}>
                  <div 
                    className={`h-2.5 rounded-full ${
                      latestData.mem_usage > 75 ? 'bg-red-500' : 
                      latestData.mem_usage > 50 ? 'bg-yellow-500' : 
                      'bg-green-500'
                    }`}
                    style={{ width: `${latestData.mem_usage}%` }}
                  ></div>
                </div>
                <span className="text-lg font-bold">{latestData.mem_usage}%</span>
              </div>
            </div>
            
            {/* CPU 온도 */}
            <div className={`p-4 rounded-xl ${isAdminMode ? 'bg-gray-700' : 'bg-gray-50'}`}>
              <div className="flex items-center justify-between mb-2">
                <h3 className="text-sm text-gray-500 dark:text-gray-400">CPU 온도</h3>
                <Thermometer className="h-4 w-4 text-red-500" />
              </div>
              <p className={`text-lg font-bold ${
                getStatusClass(latestData.cpu_temp, 50, 70).textColor
              }`}>{latestData.cpu_temp}°C</p>
            </div>
            
            {/* 냉각팬 상태 */}
            <div className={`p-4 rounded-xl ${isAdminMode ? 'bg-gray-700' : 'bg-gray-50'}`}>
              <div className="flex items-center justify-between mb-2">
                <h3 className="text-sm text-gray-500 dark:text-gray-400">냉각팬 상태</h3>
                <Fan className="h-4 w-4 text-cyan-500" />
              </div>
              <div className="grid grid-cols-2 gap-2">
                <div className={`text-xs p-1 text-center rounded ${
                  latestData.cooling_fanA 
                    ? 'bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-300' 
                    : 'bg-gray-100 text-gray-800 dark:bg-gray-800 dark:text-gray-400'
                }`}>
                  팬 A: {latestData.cooling_fanA ? '작동중' : '정지'}
                </div>
                <div className={`text-xs p-1 text-center rounded ${
                  latestData.cooling_fanB
                    ? 'bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-300' 
                    : 'bg-gray-100 text-gray-800 dark:bg-gray-800 dark:text-gray-400'
                }`}>
                  팬 B: {latestData.cooling_fanB ? '작동중' : '정지'}
                </div>
              </div>
            </div>
          </div>
          
          {/* 추가 상태 정보 */}
          <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
            {/* 저항 온도 */}
            <div className={`p-4 rounded-xl ${isAdminMode ? 'bg-gray-700' : 'bg-gray-50'}`}>
              <h3 className="text-sm text-gray-500 dark:text-gray-400 mb-3">저항 온도</h3>
              <div className="space-y-2">
                <div className="flex justify-between items-center">
                  <span className="text-sm">저항 A:</span>
                  <span className={`font-semibold ${
                    getStatusClass(latestData.resistance_tempA, 40, 60).textColor
                  }`}>{latestData.resistance_tempA}°C</span>
                </div>
                <div className="flex justify-between items-center">
                  <span className="text-sm">저항 B:</span>
                  <span className={`font-semibold ${
                    getStatusClass(latestData.resistance_tempB, 40, 60).textColor
                  }`}>{latestData.resistance_tempB}°C</span>
                </div>
                <div className="flex justify-between items-center">
                  <span className="text-sm">주변 온도:</span>
                  <span className={`font-semibold ${
                    getStatusClass(latestData.ambient_temp, 10, 30).textColor
                  }`}>{latestData.ambient_temp}°C</span>
                </div>
              </div>
            </div>
            
            {/* 배출통 상태 */}
            <div className={`p-4 rounded-xl ${isAdminMode ? 'bg-gray-700' : 'bg-gray-50'}`}>
              <h3 className="text-sm text-gray-500 dark:text-gray-400 mb-3">배출통 상태</h3>
              <div className="space-y-2">
                <div className="flex justify-between items-center">
                  <span className="text-sm">불량품 배출통:</span>
                  <span className={`font-semibold px-2 py-1 rounded-full text-xs ${
                    latestData.is_fullerror 
                      ? 'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-300' 
                      : 'bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-300'
                  }`}>
                    {latestData.is_fullerror ? '가득참' : '정상'}
                  </span>
                </div>
                <div className="flex justify-between items-center">
                  <span className="text-sm">정상품 배출통:</span>
                  <span className={`font-semibold px-2 py-1 rounded-full text-xs ${
                    latestData.is_fullnormal 
                      ? 'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-300' 
                      : 'bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-300'
                  }`}>
                    {latestData.is_fullnormal ? '가득참' : '정상'}
                  </span>
                </div>
              </div>
            </div>
            
            {/* 기기 정보 */}
            <div className={`p-4 rounded-xl ${isAdminMode ? 'bg-gray-700' : 'bg-gray-50'}`}>
              <h3 className="text-sm text-gray-500 dark:text-gray-400 mb-3">기기 정보</h3>
              <div className="space-y-2">
                <div className="flex justify-between items-center">
                  <span className="text-sm">장치 ID:</span>
                  <span className="font-mono font-semibold">Device {latestData.device_id}</span>
                </div>
                <div className="flex justify-between items-center">
                  <span className="text-sm">마지막 업데이트:</span>
                  <span className="font-mono text-xs">
                    {new Date(latestData.recorded_at).toLocaleString()}
                  </span>
                </div>
                <div className="flex justify-between items-center">
                  <span className="text-sm">상태:</span>
                  <span className={`font-semibold px-2 py-1 rounded-full text-xs bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-300`}>
                    활성
                  </span>
                </div>
              </div>
            </div>
          </div>
        </div>
      ) : (
        <div className="p-6 text-center text-gray-500">
          선택된 장치에 대한 데이터가 없습니다.
        </div>
      )}
    </div>
  );
};

export default DeviceStatus; 