// src/data/mockData.js

// 배터리 타입 목록
const batteryTypes = ['리튬이온', '리튬폴리머', 'LFP', 'NMC'];

// 배터리 로그 데이터 생성
export const generateBatteryLogs = (count = 50) => {
  const logs = [];
  const now = new Date();
  
  for (let i = 0; i < count; i++) {
    const timestamp = new Date(now - Math.random() * 86400000 * 2); // 최근 2일 내
    logs.push({
      battery_id: `BAT-${String(i + 1).padStart(4, '0')}`,
      battery_type: batteryTypes[Math.floor(Math.random() * batteryTypes.length)],
      soc: Math.floor(Math.random() * 100), // 충전 상태 (0-100%)
      inspection_result: Math.random() > 0.2, // 80%는 양호, 20%는 불량
      circuit_type: Math.random() > 0.5, // A 또는 B 회로
      temperature: Math.floor(Math.random() * 20) + 20, // 20-40°C
      voltage: (Math.random() * 2 + 3).toFixed(2), // 3-5V
      current: (Math.random() * 2).toFixed(2), // 0-2A
      health: Math.floor(Math.random() * 20) + 80, // 80-100%
      charge_cycles: Math.floor(Math.random() * 500), // 충전 사이클 수
      timestamp: timestamp.toISOString()
    });
  }
  
  // 시간순으로 정렬
  return logs.sort((a, b) => new Date(a.timestamp) - new Date(b.timestamp));
};

// 시스템 모니터링 데이터 생성
export const generateSystemData = () => {
  const deviceData = [];
  const deviceCount = 8;
  
  for (let deviceId = 1; deviceId <= deviceCount; deviceId++) {
    // 기기별 데이터 생성
    const now = new Date();
    const timePoints = 24; // 24시간 데이터
    const deviceStatus = [];
    
    for (let i = 0; i < timePoints; i++) {
      const timestamp = new Date(now - (timePoints - i) * 3600000);
      
      deviceStatus.push({
        device_id: deviceId,
        cpu_percent: Math.floor(Math.random() * 40) + 20, // 20-60%
        mem_usage: Math.floor(Math.random() * 30) + 40, // 40-70%
        cpu_temp: Math.floor(Math.random() * 15) + 45, // 45-60°C
        soc_temp: Math.floor(Math.random() * 10) + 35, // 35-45°C
        resistance_tempA: Math.floor(Math.random() * 15) + 35, // 35-50°C
        resistance_tempB: Math.floor(Math.random() * 15) + 30, // 30-45°C
        ambient_temp: Math.floor(Math.random() * 5) + 22, // 22-27°C
        cooling_fanA: Math.random() > 0.3, // 70% 확률로 켜짐
        cooling_fanB: Math.random() > 0.4, // 60% 확률로 켜짐
        is_fullerror: Math.random() > 0.7, // 30% 확률로 가득참
        is_fullnormal: Math.random() > 0.5, // 50% 확률로 가득참
        recorded_at: timestamp.toISOString()
      });
    }
    
    deviceData.push(...deviceStatus);
  }
  
  return deviceData;
};

// 시스템 상태 요약 생성
export const generateSystemSummary = () => {
  return {
    totalBatteries: Math.floor(Math.random() * 300) + 200, // 200-500개 배터리
    passRate: (Math.random() * 10 + 88).toFixed(1), // 88-98% 통과율
    failRate: (Math.random() * 10 + 2).toFixed(1), // 2-12% 불량률
    avgTemperature: (Math.random() * 5 + 35).toFixed(1), // 평균 35-40°C
    avgSOC: Math.floor(Math.random() * 20 + 70), // 평균 70-90% 충전
    todayInspected: Math.floor(Math.random() * 50) + 50, // 오늘 검사된 배터리 수
    alertCount: Math.floor(Math.random() * 5), // 0-4개 경고
    deviceCount: 8, // 장치 수
    activeDevices: Math.floor(Math.random() * 3) + 5 // 5-8개 활성 장치
  };
};

// 미리 생성된 데이터
export const batteryLogs = generateBatteryLogs();
export const systemData = generateSystemData();
export const systemSummary = generateSystemSummary(); 