import React, { useState } from 'react';
import { Battery, Clock, Activity, ThermometerSnowflake, Zap } from 'lucide-react';

const BatteryLogTable = ({ data = [], isAdminMode = false }) => {
  const [sortConfig, setSortConfig] = useState({
    key: 'timestamp',
    direction: 'desc'
  });
  
  const formatDateTime = (timestamp) => {
    const date = new Date(timestamp);
    return date.toLocaleString('ko-KR', {
      month: '2-digit',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit',
      hour12: false
    });
  };
  
  const sortedData = [...data].sort((a, b) => {
    if (sortConfig.key === 'timestamp') {
      const dateA = new Date(a[sortConfig.key]).getTime();
      const dateB = new Date(b[sortConfig.key]).getTime();
      return sortConfig.direction === 'asc' ? dateA - dateB : dateB - dateA;
    } else if (sortConfig.key === 'soc' || sortConfig.key === 'temperature' || sortConfig.key === 'voltage') {
      return sortConfig.direction === 'asc' 
        ? a[sortConfig.key] - b[sortConfig.key] 
        : b[sortConfig.key] - a[sortConfig.key];
    } else {
      if (a[sortConfig.key] < b[sortConfig.key]) {
        return sortConfig.direction === 'asc' ? -1 : 1;
      }
      if (a[sortConfig.key] > b[sortConfig.key]) {
        return sortConfig.direction === 'asc' ? 1 : -1;
      }
      return 0;
    }
  });
  
  const requestSort = (key) => {
    let direction = 'asc';
    if (sortConfig.key === key && sortConfig.direction === 'asc') {
      direction = 'desc';
    }
    setSortConfig({ key, direction });
  };
  
  const getSortIcon = (key) => {
    if (sortConfig.key !== key) {
      return <span className="ml-1 text-gray-400">↕</span>;
    }
    return sortConfig.direction === 'asc' 
      ? <span className="ml-1 text-blue-500">↑</span> 
      : <span className="ml-1 text-blue-500">↓</span>;
  };
  
  const bgColor = isAdminMode ? 'bg-gray-800' : 'bg-white';
  const textColor = isAdminMode ? 'text-gray-300' : 'text-gray-800';
  const titleColor = isAdminMode ? 'text-gray-200' : 'text-gray-800';
  const borderColor = isAdminMode ? 'border-gray-700' : 'border-gray-200';
  const headerBgColor = isAdminMode ? 'bg-gray-700' : 'bg-gray-100';
  const rowHoverBgColor = isAdminMode ? 'hover:bg-gray-700' : 'hover:bg-gray-50';
  const rowBorderColor = isAdminMode ? 'border-gray-700' : 'border-gray-200';
  
  return (
    <div className={`${bgColor} ${textColor} rounded-xl shadow-md p-4 ${isAdminMode ? '' : 'border'} ${borderColor}`}>
      <div className="p-4 border-b border-gray-200 dark:border-gray-700 flex justify-between items-center">
        <div className="flex items-center">
          <Battery className={`h-5 w-5 ${isAdminMode ? 'text-red-400' : 'text-blue-500'} mr-2`} />
          <h2 className="text-lg font-semibold">배터리 로그</h2>
        </div>
        <span className="text-xs font-mono flex items-center">
          <Clock className="h-3 w-3 mr-1" />
          최근 업데이트: {formatDateTime(new Date().toISOString())}
        </span>
      </div>
      
      <div className="overflow-x-auto">
        <table className="min-w-full table-auto text-sm">
          <thead className={`${headerBgColor}`}>
            <tr>
              <th className="px-4 py-2 text-left font-medium">
                <button 
                  className="flex items-center focus:outline-none"
                  onClick={() => requestSort('battery_id')}
                >
                  ID {getSortIcon('battery_id')}
                </button>
              </th>
              <th className="px-4 py-2 text-left font-medium">
                <button 
                  className="flex items-center focus:outline-none"
                  onClick={() => requestSort('battery_type')}
                >
                  종류 {getSortIcon('battery_type')}
                </button>
              </th>
              <th className="px-4 py-2 text-left font-medium">
                <button 
                  className="flex items-center focus:outline-none"
                  onClick={() => requestSort('soc')}
                >
                  SOC {getSortIcon('soc')}
                </button>
              </th>
              <th className="px-4 py-2 text-left font-medium">
                <button 
                  className="flex items-center focus:outline-none"
                  onClick={() => requestSort('inspection_result')}
                >
                  검사결과 {getSortIcon('inspection_result')}
                </button>
              </th>
              <th className="px-4 py-2 text-left font-medium">
                <button 
                  className="flex items-center focus:outline-none"
                  onClick={() => requestSort('temperature')}
                >
                  온도 {getSortIcon('temperature')}
                </button>
              </th>
              <th className="px-4 py-2 text-left font-medium">
                <button 
                  className="flex items-center focus:outline-none"
                  onClick={() => requestSort('voltage')}
                >
                  전압 {getSortIcon('voltage')}
                </button>
              </th>
              <th className="px-4 py-2 text-left font-medium">회로</th>
              <th className="px-4 py-2 text-left font-medium">
                <button 
                  className="flex items-center focus:outline-none"
                  onClick={() => requestSort('timestamp')}
                >
                  시간 {getSortIcon('timestamp')}
                </button>
              </th>
            </tr>
          </thead>
          <tbody>
            {sortedData.map((battery, index) => (
              <tr 
                key={`${battery.battery_id}-${index}`} 
                className={`${rowHoverBgColor} border-b ${rowBorderColor}`}
              >
                <td className="px-4 py-2 font-mono">{battery.battery_id}</td>
                <td className="px-4 py-2">{battery.battery_type}</td>
                <td className="px-4 py-2">{battery.soc}%</td>
                <td className="px-4 py-2">
                  {battery.inspection_result ? 
                    <span className="inline-flex items-center bg-green-100 text-green-800 text-xs font-medium px-2 py-0.5 rounded-full">
                      ✓ 양호
                    </span> : 
                    <span className="inline-flex items-center bg-red-100 text-red-800 text-xs font-medium px-2 py-0.5 rounded-full">
                      ✗ 불량
                    </span>
                  }
                </td>
                <td className="px-4 py-2">{battery.temperature}°C</td>
                <td className="px-4 py-2">{battery.voltage}V</td>
                <td className="px-4 py-2">{battery.circuit_type ? 'A' : 'B'}</td>
                <td className="px-4 py-2">{formatDateTime(battery.timestamp)}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      
      {data.length === 0 && (
        <div className="p-6 text-center text-gray-500 dark:text-gray-400">
          데이터가 없습니다.
        </div>
      )}
    </div>
  );
};

export default BatteryLogTable; 