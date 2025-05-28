import React from 'react';
import { PieChart as RechartsSimplePie, Pie, Cell, Tooltip, ResponsiveContainer, Legend } from 'recharts';

const BatteryTypePie = ({ data = [], isAdminMode = false }) => {
  // 색상 설정
  const COLORS = ['#3b82f6', '#8b5cf6', '#10b981', '#f59e0b'];
  
  // 배터리 타입별 데이터 분석
  const typeCounts = data.reduce((acc, curr) => {
    if (!acc[curr.battery_type]) {
      acc[curr.battery_type] = {
        total: 0,
        failed: 0
      };
    }
    acc[curr.battery_type].total++;
    if (!curr.inspection_result) acc[curr.battery_type].failed++;
    return acc;
  }, {});
  
  // 차트 데이터 변환
  const chartData = Object.entries(typeCounts).map(([type, { total, failed }], idx) => ({
    name: type,
    value: total,
    failed,
    fill: COLORS[idx % COLORS.length]
  }));
  
  // 스타일 설정
  const bgColor = isAdminMode ? 'bg-gray-800' : 'bg-white';
  const textColor = isAdminMode ? 'text-gray-300' : 'text-gray-800';
  const titleColor = isAdminMode ? 'text-gray-200' : 'text-gray-800';
  const borderColor = isAdminMode ? 'border-gray-700' : 'border-gray-200';
  
  // 커스텀 툴팁
  const CustomTooltip = ({ active, payload }) => {
    if (active && payload && payload.length) {
      const data = payload[0].payload;
      return (
        <div className={`${isAdminMode ? 'bg-gray-700' : 'bg-white'} p-2 rounded shadow ${isAdminMode ? 'text-gray-200' : 'text-gray-800'} border ${isAdminMode ? 'border-gray-600' : 'border-gray-300'}`}>
          <p className="font-bold">{data.name}</p>
          <p>총 개수: {data.value}개</p>
          <p>불량품: {data.failed}개 ({Math.round((data.failed / data.value) * 100)}%)</p>
        </div>
      );
    }
    return null;
  };
  
  return (
    <div className={`${bgColor} ${textColor} rounded-xl shadow-md p-4 ${isAdminMode ? '' : 'border'} ${borderColor}`}>
      <h3 className={`text-lg font-medium ${titleColor} mb-4`}>배터리 타입별 비중</h3>
      <div className="h-64 flex items-center justify-center">
        <ResponsiveContainer width="100%" height="100%">
          <RechartsSimplePie>
            <Pie
              data={chartData}
              cx="50%"
              cy="50%"
              labelLine={false}
              outerRadius={80}
              dataKey="value"
              label={({ name, percent }) => `${name} ${(percent * 100).toFixed(0)}%`}
            >
              {chartData.map((entry, index) => (
                <Cell key={`cell-${index}`} fill={entry.fill} />
              ))}
            </Pie>
            <Tooltip content={<CustomTooltip />} />
            <Legend />
          </RechartsSimplePie>
        </ResponsiveContainer>
      </div>
      
      {/* 통계 박스 */}
      <div className="grid grid-cols-2 gap-2 mt-3">
        {chartData.map((item, idx) => (
          <div key={idx} className={`p-2 rounded ${isAdminMode ? 'bg-gray-700' : 'bg-gray-50'} text-sm flex items-center`}>
            <div className="w-3 h-3 rounded-full mr-2" style={{ backgroundColor: item.fill }}></div>
            <div>
              <div className="font-medium">{item.name}</div>
              <div className="text-xs opacity-80">
                {item.value}개 / 불량 {item.failed}개
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default BatteryTypePie; 