import React from 'react';
// 삭제된 차트 컴포넌트들
// import CircuitTempChart from '../dashboard/CircuitTempChart';
// import CpuMemoryChart from '../dashboard/CpuMemoryChart';
// import CpuSocTempChart from '../dashboard/CpuSocTempChart';

const AdminDashboardCharts = () => {
  return (
    <div className="w-full space-y-6">
      <h2 className="text-xl font-semibold text-gray-200 mb-4">시스템 모니터링</h2>
      
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* 차트 컴포넌트들이 삭제되어 대체 메시지 표시 */}
        <div className="bg-gray-700 p-6 rounded-lg shadow-md">
          <h3 className="text-gray-300 font-medium mb-3">회로 온도 모니터링</h3>
          <p className="text-gray-400 text-center py-4">시스템 업데이트 중...</p>
        </div>
        
        <div className="bg-gray-700 p-6 rounded-lg shadow-md">
          <h3 className="text-gray-300 font-medium mb-3">CPU & 메모리 사용률</h3>
          <p className="text-gray-400 text-center py-4">시스템 업데이트 중...</p>
        </div>
        
        <div className="bg-gray-700 p-6 rounded-lg shadow-md">
          <h3 className="text-gray-300 font-medium mb-3">CPU & SOC 온도</h3>
          <p className="text-gray-400 text-center py-4">시스템 업데이트 중...</p>
        </div>
      </div>
    </div>
  );
};

export default AdminDashboardCharts; 