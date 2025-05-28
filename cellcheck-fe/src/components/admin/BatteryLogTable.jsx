import React, { useState, useEffect } from 'react';
import { Battery, Search, Filter, AlertTriangle, Check, X, ChevronLeft, ChevronRight } from 'lucide-react';
import { getBatteryLogs } from '../../api/batteryApi';
import Button from '../common/Button';

const BatteryLogTable = ({ isAdminMode = true }) => {
  const [logs, setLogs] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  
  // 필터링 및 페이징 상태
  const [dayFilter, setDayFilter] = useState(7); // 기본값: 7일
  const [currentPage, setCurrentPage] = useState(1);
  const [totalPages, setTotalPages] = useState(1);
  const [totalItems, setTotalItems] = useState(0);
  const itemsPerPage = 10;
  
  // 배터리 로그 데이터 불러오기
  const fetchBatteryLogs = async () => {
    setLoading(true);
    try {
      // 토큰 확인용 로그
      const token = localStorage.getItem('accessToken');
      console.log('API 호출 전 토큰 존재 여부:', !!token);
      console.log('API 호출 정보:', { dayFilter, currentPage, itemsPerPage });
      
      const response = await getBatteryLogs(dayFilter, currentPage, itemsPerPage);
      console.log('배터리 로그 API 응답:', response);
      
      if (response && response.success) {
        if (Array.isArray(response.data)) {
          console.log('배터리 로그 데이터 수:', response.data.length);
          
          // 데이터가 있는 경우
          if (response.data.length > 0) {
            console.log('첫 번째 로그 항목 샘플:', response.data[0]);
            setLogs(response.data);
            
            // 페이지 정보 설정
            if (response.meta) {
              setTotalItems(response.meta.totalItems);
              setTotalPages(Math.max(1, response.meta.totalPages));
            } else {
              // 페이지 정보가 없으면 데이터 길이로 계산
              setTotalItems(response.data.length);
              setTotalPages(Math.max(1, Math.ceil(response.data.length / itemsPerPage)));
            }
            
            setError(null);
          } else {
            // 데이터가 없는 경우
            console.log('배터리 로그 데이터가 없음');
            setLogs([]);
            setTotalItems(0);
            setTotalPages(1);
            setError('배터리 로그 데이터가 없습니다.');
          }
        } else {
          console.error('API 응답의 data 필드가 배열이 아님:', response.data);
          setLogs([]);
          setError('데이터 형식이 올바르지 않습니다.');
        }
      } else {
        console.error('배터리 로그 API 호출 실패:', response?.message || '알 수 없는 오류');
        setLogs([]);
        setError(response?.message || '배터리 로그를 불러오는데 실패했습니다.');
      }
    } catch (err) {
      console.error('배터리 로그 조회 오류:', err);
      setLogs([]);
      setError('배터리 로그를 불러오는데 실패했습니다: ' + (err.message || '알 수 없는 오류'));
    } finally {
      setLoading(false);
    }
  };

  // 컴포넌트 마운트 시와 필터/페이지 변경 시 데이터 로드
  useEffect(() => {
    fetchBatteryLogs();
  }, [currentPage, dayFilter]);

  // 필터 변경 핸들러
  const handleFilterChange = (days) => {
    setDayFilter(days);
    setCurrentPage(1); // 필터 변경 시 첫 페이지로 리셋
  };
  
  // 페이지 이동 핸들러
  const handlePageChange = (newPage) => {
    if (newPage > 0 && newPage <= totalPages) {
      setCurrentPage(newPage);
    }
  };

  // 수동으로 다시 불러오기
  const handleRetry = () => {
    fetchBatteryLogs();
  };
  
  // 날짜 포맷팅 함수
  const formatDate = (dateString) => {
    if (!dateString) return '날짜 정보 없음';
    try {
      const date = new Date(dateString);
      return date.toLocaleString('ko-KR', { 
        year: 'numeric', 
        month: '2-digit', 
        day: '2-digit',
        hour: '2-digit', 
        minute: '2-digit'
      });
    } catch (error) {
      console.error('날짜 포맷팅 오류:', error, dateString);
      return '날짜 포맷 오류';
    }
  };

  // 배터리 타입 표시 클래스
  const getBatteryTypeClass = (type) => {
    switch (type) {
      case 'AAA':
        return isAdminMode ? 'bg-blue-900 bg-opacity-40 text-blue-300' : 'bg-blue-100 text-blue-800';
      case 'AA':
        return isAdminMode ? 'bg-green-900 bg-opacity-40 text-green-300' : 'bg-green-100 text-green-800';
      case 'C':
        return isAdminMode ? 'bg-purple-900 bg-opacity-40 text-purple-300' : 'bg-purple-100 text-purple-800';
      case 'D':
        return isAdminMode ? 'bg-yellow-900 bg-opacity-40 text-yellow-300' : 'bg-yellow-100 text-yellow-800';
      default:
        return isAdminMode ? 'bg-gray-700 text-gray-300' : 'bg-gray-200 text-gray-800';
    }
  };

  return (
    <div className={`admin-card ${isAdminMode ? 'bg-gray-700' : 'bg-white'} shadow-md`}>
      <div className={`card-header ${isAdminMode ? 'border-gray-600' : 'border-gray-200'}`}>
        <div className="flex justify-between items-center">
          <h2 className={`text-lg font-medium flex items-center ${isAdminMode ? 'text-white' : 'text-gray-800'}`}>
            <Battery className="h-5 w-5 mr-2" />
            배터리 이력
          </h2>
          
          <div className="flex items-center space-x-3">
            <div className={`flex space-x-2 rounded-md shadow-sm ${isAdminMode ? 'bg-gray-800' : 'bg-gray-100'}`}>
              <Button 
                variant={dayFilter === 1 ? (isAdminMode ? "primary" : "default") : "ghost"} 
                size="sm"
                onClick={() => handleFilterChange(1)}
              >
                1일
              </Button>
              <Button 
                variant={dayFilter === 7 ? (isAdminMode ? "primary" : "default") : "ghost"} 
                size="sm"
                onClick={() => handleFilterChange(7)}
              >
                7일
              </Button>
              <Button 
                variant={dayFilter === 30 ? (isAdminMode ? "primary" : "default") : "ghost"} 
                size="sm"
                onClick={() => handleFilterChange(30)}
              >
                30일
              </Button>
            </div>
          </div>
        </div>
      </div>
      
      <div className="overflow-x-auto">
        {loading ? (
          <div className={`card-body flex flex-col items-center justify-center py-12 ${isAdminMode ? 'bg-gray-800' : 'bg-gray-50'}`}>
            <div className="inline-block animate-spin rounded-full h-10 w-10 border-t-2 border-b-2 border-blue-500 mb-4"></div>
            <p className={isAdminMode ? "text-gray-400" : "text-gray-500"}>로그 데이터를 불러오는 중...</p>
          </div>
        ) : error ? (
          <div className={`card-body flex flex-col items-center justify-center py-12 ${isAdminMode ? 'bg-gray-800' : 'bg-gray-50'}`}>
            <AlertTriangle className={`h-14 w-14 ${isAdminMode ? 'text-red-500' : 'text-red-600'} mb-4`} />
            <p className={`${isAdminMode ? "text-gray-300" : "text-gray-700"} text-center max-w-md mb-4`}>{error}</p>
            <button 
              className={`px-4 py-2 rounded-md ${isAdminMode ? 'bg-blue-600 text-white hover:bg-blue-700' : 'bg-blue-100 text-blue-700 hover:bg-blue-200'}`}
              onClick={handleRetry}
            >
              다시 시도하기
            </button>
          </div>
        ) : logs.length === 0 ? (
          <div className={`card-body flex flex-col items-center justify-center py-12 ${isAdminMode ? 'bg-gray-800' : 'bg-gray-50'}`}>
            <Battery className={`h-14 w-14 ${isAdminMode ? 'text-gray-500' : 'text-gray-400'} mb-4`} />
            <p className={isAdminMode ? "text-gray-400" : "text-gray-500"}>
              최근 {dayFilter}일 동안의 배터리 기록이 없습니다.
            </p>
            <button 
              className={`mt-4 px-4 py-2 rounded-md ${isAdminMode ? 'bg-blue-600 text-white hover:bg-blue-700' : 'bg-blue-100 text-blue-700 hover:bg-blue-200'}`}
              onClick={handleRetry}
            >
              다시 시도하기
            </button>
          </div>
        ) : (
          <>
            <table className="min-w-full table-auto border-collapse">
              <thead className={isAdminMode ? 'bg-gray-800' : 'bg-gray-50'}>
                <tr>
                  <th className={`p-3 text-center w-[10%] ${
                    isAdminMode ? 'text-gray-400' : 'text-gray-500'
                  }`}>
                    ID
                  </th>
                  <th className={`p-3 text-center w-[20%] ${
                    isAdminMode ? 'text-gray-400' : 'text-gray-500'
                  }`}>
                    배터리 타입
                  </th>
                  <th className={`p-3 text-center w-[20%] ${
                    isAdminMode ? 'text-gray-400' : 'text-gray-500'
                  }`}>
                    검사 결과
                  </th>
                  <th className={`p-3 text-center w-[50%] ${
                    isAdminMode ? 'text-gray-400' : 'text-gray-500'
                  }`}>
                    처리 일시
                  </th>
                </tr>
              </thead>
              <tbody className={`divide-y ${
                isAdminMode ? 'bg-gray-800 divide-gray-700' : 'bg-white divide-gray-200'
              }`}>
                {logs.slice(0, itemsPerPage).map((log) => (
                  <tr key={log.id} className={
                    isAdminMode ? 'hover:bg-gray-700' : 'hover:bg-gray-50'
                  }>
                    <td className={`p-3 text-center ${
                      isAdminMode ? 'text-gray-300' : 'text-gray-800'
                    }`}>
                      {log.id}
                    </td>
                    <td className={`p-3 text-center ${
                      isAdminMode ? 'text-gray-300' : 'text-gray-800'
                    }`}>
                      <span className={`inline-block px-3 py-1 rounded-full text-xs font-medium ${getBatteryTypeClass(log.batteryType)}`}>
                        {log.batteryType || '알 수 없음'}
                      </span>
                    </td>
                    <td className={`p-3 text-center ${
                      isAdminMode ? 'text-gray-300' : 'text-gray-800'
                    }`}>
                      {log.inspectionResult === true ? (
                        <span className={`inline-flex items-center justify-center ${isAdminMode ? 'text-green-400' : 'text-green-500'}`}>
                          <Check className="h-4 w-4 mr-1" />
                          방전 처리
                        </span>
                      ) : (
                        <span className={`inline-flex items-center justify-center ${isAdminMode ? 'text-red-400' : 'text-red-500'}`}>
                          <X className="h-4 w-4 mr-1" />
                          외관 불량
                        </span>
                      )}
                    </td>
                    <td className={`p-3 text-center ${
                      isAdminMode ? 'text-gray-400' : 'text-gray-500'
                    }`}>
                      {formatDate(log.createdAt)}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
            
            {/* 페이지네이션 */}
            <div className={`card-footer flex items-center justify-between ${
              isAdminMode ? 'bg-gray-800 border-gray-700' : 'bg-white border-gray-200'
            }`}>
              <div className={isAdminMode ? "text-gray-400" : "text-gray-500"}>
                <p className="text-sm">
                  총 <span className={isAdminMode ? "text-white" : "text-gray-900"}>
                    {totalItems}
                  </span> 항목 중 {(currentPage - 1) * itemsPerPage + 1}-
                  {Math.min(currentPage * itemsPerPage, totalItems)}
                </p>
              </div>
              <div className="flex-1 flex justify-center sm:justify-end">
                <nav className="relative z-0 inline-flex shadow-sm -space-x-px" aria-label="Pagination">
                  <Button 
                    variant="ghost"
                    onClick={() => handlePageChange(currentPage - 1)}
                    disabled={currentPage === 1}
                    className={`${
                      isAdminMode ? 'bg-gray-800 text-gray-400 hover:bg-gray-700' : 'bg-white text-gray-500 hover:bg-gray-50'
                    } ${
                      currentPage === 1 ? 'cursor-not-allowed opacity-50' : ''
                    }`}
                  >
                    <ChevronLeft className="h-4 w-4" />
                  </Button>
                  <div className={`px-4 py-2 ${
                    isAdminMode ? 'bg-gray-800 text-white' : 'bg-white text-gray-800'
                  }`}>
                    {currentPage} / {totalPages}
                  </div>
                  <Button 
                    variant="ghost"
                    onClick={() => handlePageChange(currentPage + 1)}
                    disabled={currentPage === totalPages}
                    className={`${
                      isAdminMode ? 'bg-gray-800 text-gray-400 hover:bg-gray-700' : 'bg-white text-gray-500 hover:bg-gray-50'
                    } ${
                      currentPage === totalPages ? 'cursor-not-allowed opacity-50' : ''
                    }`}
                  >
                    <ChevronRight className="h-4 w-4" />
                  </Button>
                </nav>
              </div>
            </div>
          </>
        )}
      </div>
    </div>
  );
};

export default BatteryLogTable; 