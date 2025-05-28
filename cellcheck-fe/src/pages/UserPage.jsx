import React, { useState, useEffect } from 'react';
import { Battery, User, List, BarChart, Settings, Bell, Clock, ChevronRight, AlertTriangle, Info, Wrench, RefreshCw } from 'lucide-react';
import Header from '../components/layout/Header';
import Footer from '../components/layout/Footer';
import BackgroundPattern from '../components/layout/BackgroundPattern';
import Button from '../components/common/Button';
import useAuth from '../hooks/useAuth';
import BatteryInspectionCard from '../components/dashboard/BatteryInspectionCard';
import BatteryDischargeCard from '../components/dashboard/BatteryDischargeCard';
import CircuitStatusCard from '../components/dashboard/CircuitStatusCard';
import { useWebSocketContext } from '../context/WebSocketContext';
import ConnectionStatus from '../components/common/ConnectionStatus';
import { getNotices, getNoticeTypeLabel, getNoticeTypeColorClass, formatRelativeTime } from '../api/noticeApi';
import NoticeDetailModal from '../components/common/NoticeDetailModal';
import { useNavigate, useLocation } from 'react-router-dom';

const UserPage = () => {
  const { user, systemStatus, logout } = useAuth();
  const { connected, timeSeriesData } = useWebSocketContext();
  const navigate = useNavigate();
  const location = useLocation();
  
  // URL 쿼리 파라미터에서 선택된 탭 가져오기
  const queryParams = new URLSearchParams(location.search);
  const tabParam = queryParams.get('tab');
  
  const [currentTab, setCurrentTab] = useState(tabParam || 'dashboard');
  
  // 반응형을 위한 사이드바 상태 관리
  const [sidebarOpen, setSidebarOpen] = useState(false);
  
  // 공지사항 상태 관리
  const [notices, setNotices] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  
  // 미확인 공지사항 수 계산
  const [unreadNotifications, setUnreadNotifications] = useState(0);
  
  // 공지사항 상세 모달 상태
  const [selectedNotice, setSelectedNotice] = useState(null);
  const [showNoticeModal, setShowNoticeModal] = useState(false);
  
  // 선택된 탭 변경 시 URL 업데이트
  const handleTabChange = (tab) => {
    setCurrentTab(tab);
    setSidebarOpen(false); // 모바일에서 탭 선택 후 사이드바 닫기
    navigate(`/user?tab=${tab}`, { replace: true });
  };
  
  // 페이지 새로고침 함수
  const handleRefreshPage = () => {
    window.location.reload();
  };
  
  // 공지사항 상세 보기
  const handleNoticeClick = (notice) => {
    setSelectedNotice(notice);
    setShowNoticeModal(true);
  };
  
  // 공지사항 모달 닫기
  const handleCloseModal = () => {
    setShowNoticeModal(false);
  };
  
  // 공지사항 불러오기
  useEffect(() => {
    const fetchNotices = async () => {
      setIsLoading(true);
      setError(null);
      try {
        const response = await getNotices();
        
        if (response.success && Array.isArray(response.data)) {
          // 공지사항 데이터 설정
          setNotices(response.data);
          
          // 일주일 내의 공지사항 수를 미확인 공지로 간주
          const oneWeekAgo = new Date();
          oneWeekAgo.setDate(oneWeekAgo.getDate() - 7);
          
          const recentNotices = response.data.filter(notice => 
            new Date(notice.createdAt) > oneWeekAgo
          );
          
          setUnreadNotifications(recentNotices.length);
        } else {
          setError('공지사항 데이터 형식이 올바르지 않습니다.');
          setNotices([]); // 빈 배열로 초기화
        }
      } catch (err) {
        setError('공지사항을 불러오는데 실패했습니다.');
        setNotices([]); // 오류 시 빈 배열로 초기화
      } finally {
        setIsLoading(false);
      }
    };

    // 공지사항 탭으로 변경될 때 또는 컴포넌트 마운트 시 데이터 로드
    if (currentTab === 'notices') {
      fetchNotices();
    }
  }, [currentTab]);
  
  // 토글 사이드바 함수
  const toggleSidebar = () => {
    setSidebarOpen(!sidebarOpen);
  };
  
  // 공지사항 타입에 따른 아이콘 렌더링
  const renderNoticeTypeIcon = (type) => {
    switch (type) {
      case 0: // 점검
        return <Wrench className="h-4 w-4 text-yellow-600" />;
      case 1: // 긴급
        return <AlertTriangle className="h-4 w-4 text-red-600" />;
      case 2: // 일반
        return <Info className="h-4 w-4 text-blue-600" />;
      default:
        return <Info className="h-4 w-4 text-gray-600" />;
    }
  };
  
  return (
    <div className="user-page bg-gray-100 relative z-10">
      {/* 헤더 */}
      <Header 
        isAdminMode={false} 
        systemStatus={systemStatus}
        onToggleSidebar={toggleSidebar}
      />
      
      {/* 배경 패턴 */}
      <BackgroundPattern isAdminMode={false} />
      
      {/* 메인 콘텐츠 */}
      <div className="flex flex-1 pt-16">
        {/* 사이드바 - 모바일에서는 토글 가능 */}
        <div className={`w-64 bg-white shadow-md p-4 border-r border-gray-200 fixed h-full z-50 transition-transform duration-300 
          ${sidebarOpen ? 'translate-x-0' : '-translate-x-full'}`}
        >
          <div className="flex items-center space-x-2 px-2 py-3 mb-6">
            <User className="h-6 w-6 text-blue-500" />
            <div>
              <h2 className="text-gray-800 font-medium">사용자 패널</h2>
              <p className="text-gray-500 text-xs">{user?.name ? `${user.name} 님 환영합니다` : '사용자'}</p>
            </div>
          </div>
          
          <nav className="space-y-1">
            <button
              onClick={() => handleTabChange('dashboard')}
              className={`flex items-center space-x-2 w-full px-3 py-2 rounded-md ${
                currentTab === 'dashboard' ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'
              }`}
            >
              <BarChart className="h-5 w-5" />
              <span>대시보드</span>
            </button>
            
            <button
              onClick={() => handleTabChange('notices')}
              className={`flex items-center space-x-2 w-full px-3 py-2 rounded-md ${
                currentTab === 'notices' ? 'bg-blue-50 text-blue-600' : 'text-gray-700 hover:bg-gray-100'
              }`}
            >
              <Bell className="h-5 w-5" />
              <span>공지사항</span>
  
            </button>
            

          </nav>
          
          <div className="pt-6 mt-6 border-t border-gray-200">
            <button
              onClick={logout}
              className="flex items-center space-x-2 w-full px-3 py-2 rounded-md text-gray-700 hover:bg-gray-100"
            >
              <span className="ml-auto">로그아웃</span>
            </button>
          </div>
        </div>
        
        {/* 사이드바 오버레이 */}
        {sidebarOpen && (
          <div 
            className="fixed inset-0 bg-black bg-opacity-50 z-40" 
            onClick={toggleSidebar}
          ></div>
        )}
        
        {/* 모바일용 사이드바 토글 버튼 */}
        <button 
          className="fixed bottom-4 left-4 md:hidden z-30 bg-white text-blue-500 p-3 rounded-full shadow-lg"
          onClick={toggleSidebar}
        >
          <List className="h-6 w-6" />
        </button>
        
        {/* 메인 콘텐츠 영역 */}
        <div className="flex-1 p-4 md:p-6 bg-gray-50">
          {currentTab === 'dashboard' && (
            <div className="space-y-6">
              <div className="flex justify-between items-center mb-6">
                <h1 className="text-2xl font-semibold text-gray-800">폐배터리 모니터링 대시보드</h1>
                
                <div className="flex items-center">
   
                  {/* 연결 상태 */}
                  <ConnectionStatus connected={connected} isAdminMode={false} />
                </div>
              </div>
              

              {/* 실시간 배터리 상태 섹션 - 최신 데이터 전달 */}
              <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mb-6">
                <BatteryInspectionCard 
                  inspectionData={timeSeriesData.appearance.length > 0 ? 
                    timeSeriesData.appearance[timeSeriesData.appearance.length - 1] : null}
                  dischargeData={timeSeriesData.discharge?.length > 0 ? 
                    timeSeriesData.discharge[timeSeriesData.discharge.length - 1] : null}
                  isAdminMode={false} 
                />
                
                <BatteryDischargeCard 
                  batteryData={timeSeriesData.discharge?.length > 0 ? 
                    timeSeriesData.discharge[timeSeriesData.discharge.length - 1] : null}
                  inspectionData={timeSeriesData.appearance.length > 0 ? 
                    timeSeriesData.appearance[timeSeriesData.appearance.length - 1] : null}
                  isAdminMode={false} 
                />
              </div>
              
              {/* 시계열 그래프를 표시하는 컴포넌트 - 전체 배열 전달 */}
              <CircuitStatusCard 
                environmentData={timeSeriesData.environment}
                isAdminMode={false} 
              />
            </div>
          )}

          {currentTab === 'notices' && (
            <div className="space-y-6">
              <div className="flex justify-between items-center mb-6">
                <h1 className="text-2xl font-semibold text-gray-800">공지사항</h1>
                

              </div>
              
              <div className="user-card bg-white shadow rounded-lg overflow-hidden">
                <div className="card-header border-b border-gray-200 flex justify-between items-center p-4">
                  <div>
                    <h2 className="text-lg font-medium text-gray-800 flex items-center">
                      <Bell className="h-5 w-5 mr-2" />
                      시스템 공지사항
                    </h2>
                    <p className="text-sm text-gray-500">배터리 관리 시스템의 소식을 확인하세요.</p>
                  </div>
                </div>
                
                <div className="overflow-x-auto">
                  {isLoading ? (
                    <div className="card-body flex flex-col items-center justify-center py-12 bg-gray-50">
                      <div className="inline-block animate-spin rounded-full h-10 w-10 border-t-2 border-b-2 border-blue-500 mb-4"></div>
                      <p className="text-gray-500">공지사항을 불러오는 중...</p>
                    </div>
                  ) : error ? (
                    <div className="card-body flex flex-col items-center justify-center py-12 bg-gray-50">
                      <AlertTriangle className="h-14 w-14 text-red-600 mb-4" />
                      <p className="text-gray-700 text-center max-w-md mb-4">{error}</p>
                      <button 
                        onClick={() => handleTabChange('notices')} 
                        className="px-4 py-2 rounded-md bg-blue-100 text-blue-700 hover:bg-blue-200"
                      >
                        다시 시도하기
                      </button>
                    </div>
                  ) : notices.length === 0 ? (
                    <div className="card-body flex flex-col items-center justify-center py-12 bg-gray-50">
                      <Info className="h-14 w-14 text-gray-400 mb-4" />
                      <p className="text-gray-500">현재 공지사항이 없습니다.</p>
                    </div>
                  ) : (
                    <table className="min-w-full table-fixed border-collapse">
                      <thead className="bg-gray-50">
                        <tr>
                          <th className="p-3 text-center w-[20%] text-gray-500">유형</th>
                          <th className="p-3 text-center w-[50%] text-gray-500">제목</th>
                          <th className="p-3 text-center w-[30%] text-gray-500">등록일</th>
                        </tr>
                      </thead>
                      <tbody className="divide-y bg-white divide-gray-200">
                        {notices.map((notice, index) => (
                          <tr key={index} className="hover:bg-gray-50">
                            <td className="p-3 text-center">
                              <span className={`inline-flex items-center justify-center px-2 py-1 rounded-full text-xs font-medium ${getNoticeTypeColorClass(notice.noticeType)}`}>
                                {renderNoticeTypeIcon(notice.noticeType)}
                                <span className="ml-1">{getNoticeTypeLabel(notice.noticeType)}</span>
                              </span>
                            </td>
                            <td 
                              className="p-3 text-center font-medium text-gray-800 cursor-pointer hover:text-blue-600"
                              onClick={() => handleNoticeClick(notice)}
                            >
                              {notice.noticeTitle}
                            </td>
                            <td className="p-3 text-center text-gray-500 text-sm">
                              {formatRelativeTime(notice.createdAt)}
                            </td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  )}
                </div>
              </div>
            </div>
          )}
        </div>
      </div>
      
      {/* 공지사항 상세 모달 */}
      <NoticeDetailModal 
        isOpen={showNoticeModal}
        onClose={handleCloseModal}
        notice={selectedNotice}
        isAdminMode={false}
      />
    </div>
  
  );
};

export default UserPage; 