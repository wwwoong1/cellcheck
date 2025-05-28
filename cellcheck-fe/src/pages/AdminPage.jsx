import React, { useState, useEffect } from 'react';
import { Shield, Activity, List, AlertTriangle, UserPlus, Users, Bell, FileText, Send, Info, Wrench, Plus, Battery } from 'lucide-react';
import Header from '../components/layout/Header';
import Footer from '../components/layout/Footer';
import BackgroundPattern from '../components/layout/BackgroundPattern';
import Button from '../components/common/Button';
import useAuth from '../hooks/useAuth';
import UserSignupModal from '../components/auth/UserSignupModal';
import NoticeCreateModal from '../components/admin/NoticeCreateModal';
import useModal from '../hooks/useModal';
import { useWebSocketContext } from '../context/WebSocketContext';
import ConnectionStatus from '../components/common/ConnectionStatus';
import CircuitStatusCard from '../components/dashboard/CircuitStatusCard';
import DeviceMonitoringCard from '../components/dashboard/DeviceMonitoringCard';
import { getNotices, getNoticeTypeLabel, getNoticeTypeColorClass, formatRelativeTime } from '../api/noticeApi';
import UserManagement from '../components/admin/UserManagement';
import BatteryLogTable from '../components/admin/BatteryLogTable';
import { isAuthenticated, isAdmin } from '../api/authApi';
import NoticeDetailModal from '../components/common/NoticeDetailModal';
import { useNavigate, useLocation } from 'react-router-dom';

// 안전한 데이터 접근을 위한 유틸리티 함수
const getLatestData = (dataArray) => {
  return dataArray && dataArray.length > 0 ? dataArray[dataArray.length - 1] : null;
};

// 알림 상태 확인 함수
const hasAlerts = (environmentData) => {
  const latest = getLatestData(environmentData);
  return latest && (latest.is_fullerror === 1 || latest.is_fullnormal === 1);
};

const AdminPage = () => {
  const { user, systemStatus, logout } = useAuth();
  const { connected, timeSeriesData = { environment: [], system: [], discharge: [], appearance: [] } } = useWebSocketContext();
  const navigate = useNavigate();
  const location = useLocation();
  
  // URL 쿼리 파라미터에서 선택된 섹션 가져오기
  const queryParams = new URLSearchParams(location.search);
  const sectionParam = queryParams.get('section');
  
  const [selectedSection, setSelectedSection] = useState(sectionParam || 'dashboard');
  const [selectedDevice, setSelectedDevice] = useState(1); // 항상 1 (jetson_nano)
  
  // 인증 상태 확인
  const [authChecked, setAuthChecked] = useState(false);
  
  // 선택된 섹션 변경 시 URL 업데이트
  const handleSectionChange = (section) => {
    setSelectedSection(section);
    navigate(`/admin?section=${section}`, { replace: true });
  };
  
  // 인증 확인
  useEffect(() => {
    const checkAuth = async () => {
      const authenticated = isAuthenticated();
      const adminPermission = isAdmin();
      
      console.log('인증 상태 확인:', { authenticated, adminPermission });
      
      if (!authenticated || !adminPermission) {
        console.warn('관리자 권한이 없거나 로그인되지 않았습니다.');
        // 필요시 메인 페이지 또는 로그인 페이지로 리디렉션
        // window.location.href = '/';
      }
      
      setAuthChecked(true);
    };
    
    checkAuth();
  }, []);
  
  // 공지사항 상태 관리
  const [notices, setNotices] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  
  // 공지사항 상세 모달 상태
  const [selectedNotice, setSelectedNotice] = useState(null);
  const [showNoticeModal, setShowNoticeModal] = useState(false);
  
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
        console.log('공지사항 API 응답:', response); // 디버깅용 로그
        
        if (response.success) {
          setNotices(response.data || []);
        } else {
          setError('공지사항을 불러오는데 실패했습니다.');
        }
      } catch (err) {
        setError('공지사항을 불러오는데 실패했습니다.');
        console.error('공지사항 로딩 오류:', err);
      } finally {
        setIsLoading(false);
      }
    };

    // 공지사항 탭으로 변경될 때 데이터 로드
    if (selectedSection === 'notices') {
      fetchNotices();
    }
  }, [selectedSection]);
  
  // 사용자 등록 모달 상태 관리
  const { 
    isOpen: showUserSignupModal, 
    openModal: openUserSignupModal, 
    closeModal: closeUserSignupModal 
  } = useModal();
  
  // 공지사항 등록 모달 상태 관리
  const {
    isOpen: showNoticeCreateModal,
    openModal: openNoticeCreateModal,
    closeModal: closeNoticeCreateModal
  } = useModal();
  
  // 반응형을 위한 사이드바 상태 관리
  const [sidebarOpen, setSidebarOpen] = useState(false);
  
  // 토글 사이드바 함수
  const toggleSidebar = () => {
    setSidebarOpen(!sidebarOpen);
  };
  
  // 사용자 등록 성공 처리
  const handleUserSignupSuccess = () => {
    alert('사용자가 성공적으로 등록되었습니다.');
  };
  
  // 공지사항 등록 성공 처리
  const handleNoticeCreateSuccess = () => {
    // 공지사항 다시 불러오기
    if (selectedSection === 'notices') {
      const fetchNotices = async () => {
        setIsLoading(true);
        try {
          const response = await getNotices();
          if (response.success) {
            setNotices(response.data || []);
          }
        } catch (error) {
          console.error('공지사항 갱신 중 오류:', error);
        } finally {
          setIsLoading(false);
        }
      };
      
      fetchNotices();
    }
    
    alert('공지사항이 성공적으로 등록되었습니다.');
  };
  
  // 공지사항 타입에 따른 아이콘 렌더링
  const renderNoticeTypeIcon = (type) => {
    switch (type) {
      case 0: // 점검
        return <Wrench className="h-5 w-5 text-yellow-400" />;
      case 1: // 긴급
        return <AlertTriangle className="h-5 w-5 text-red-400" />;
      case 2: // 일반
        return <Info className="h-5 w-5 text-blue-400" />;
      default:
        return <Info className="h-5 w-5 text-gray-400" />;
    }
  };

  // 인증 확인 중일 때 로딩 표시
  if (!authChecked) {
    return (
      <div className="min-h-screen bg-gray-900 flex flex-col items-center justify-center">
        <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500 mb-4"></div>
        <h2 className="text-white text-xl font-medium">인증 확인 중...</h2>
      </div>
    );
  }

  return (
    <div className="admin-page bg-gray-900 relative z-10">
      {/* 헤더 */}
      <Header 
        isAdminMode={true} 
        systemStatus={systemStatus}
        onToggleSidebar={toggleSidebar}
      />
      
      {/* 배경 패턴 */}
      <BackgroundPattern isAdminMode={true} />
      
      {/* 메인 콘텐츠 */}
      <div className="flex flex-1 pt-16">
        {/* 사이드바 - 모바일에서는 토글 가능 */}
        <div className={`w-64 bg-gray-800 border-r border-gray-700 p-4 fixed h-full z-50 transition-transform duration-300 
          ${sidebarOpen ? 'translate-x-0' : '-translate-x-full'}`}
        >
          <div className="flex items-center space-x-2 px-2 py-3 mb-6">
            <Shield className="h-6 w-6 text-red-500" />
            <div>
              <h2 className="text-white font-medium">관리자 패널</h2>
              <p className="text-gray-400 text-xs">{user?.name ? `${user.name} 님 환영합니다` : '관리자'}</p>
            </div>
          </div>
          
          <nav className="space-y-1">
            <button
              onClick={() => handleSectionChange('dashboard')}
              className={`flex items-center space-x-2 w-full px-3 py-2 rounded-md ${
                selectedSection === 'dashboard' ? 'bg-gray-700 text-white' : 'text-gray-300 hover:bg-gray-700'
              }`}
            >
              <Activity className="h-5 w-5" />
              <span>대시보드</span>
            </button>
            <button
              onClick={() => handleSectionChange('notices')}
              className={`flex items-center space-x-2 w-full px-3 py-2 rounded-md ${
                selectedSection === 'notices' ? 'bg-gray-700 text-white' : 'text-gray-300 hover:bg-gray-700'
              }`}
            >
              <Bell className="h-5 w-5" />
              <span>공지사항</span>
              {hasAlerts(timeSeriesData.environment) && (
                <span className="ml-auto bg-red-500 text-white rounded-full text-xs px-2 py-0.5">
                  !
                </span>
              )}
            </button>
            <button
              onClick={() => handleSectionChange('batteries')}
              className={`flex items-center space-x-2 w-full px-3 py-2 rounded-md ${
                selectedSection === 'batteries' ? 'bg-gray-700 text-white' : 'text-gray-300 hover:bg-gray-700'
              }`}
            >
              <Battery className="h-5 w-5" />
              <span>배터리 로그</span>
            </button>
            
            <button
              onClick={() => handleSectionChange('users')}
              className={`flex items-center space-x-2 w-full px-3 py-2 rounded-md ${
                selectedSection === 'users' ? 'bg-gray-700 text-white' : 'text-gray-300 hover:bg-gray-700'
              }`}
            >
              <Users className="h-5 w-5" />
              <span>사용자 관리</span>
            </button>
            
            
          </nav>
          
          <div className="pt-6 mt-6 border-t border-gray-700">
            <button
              onClick={logout}
              className="flex items-center space-x-2 w-full px-3 py-2 rounded-md text-gray-300 hover:bg-gray-700"
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
          className="fixed bottom-4 left-4 md:hidden z-30 bg-gray-700 text-white p-3 rounded-full shadow-lg"
          onClick={toggleSidebar}
        >
          <List className="h-6 w-6" />
        </button>
        
        {/* 메인 콘텐츠 영역 */}
        <div className="flex-1 p-4 md:p-6 bg-gray-800">
          {selectedSection === 'dashboard' && (
            <div className="admin-content">
              <div className="flex justify-between items-center mb-4">
                <h1 className="text-2xl font-semibold text-white">폐배터리 관리 시스템 - 관리자 대시보드</h1>
                <ConnectionStatus connected={connected} isAdminMode={true} />
              </div>
              
              <div className="admin-card content-section bg-gray-700">
                <h2 className="text-xl font-medium text-white mb-4">관리자 시스템 개요</h2>
                <p className="text-gray-300">
                  이 페이지는 시스템 알림 및 사용자 관리를 위해 사용됩니다.
                  배터리 관련 상세 정보는 일반 사용자용 페이지에서 확인할 수 있습니다.
                </p>
              </div>
              
              {/* 배터리 통 및 회로 상태 */}
              <CircuitStatusCard 
                environmentData={timeSeriesData?.environment || []} 
                selectedDevice={1} 
                isAdminMode={true} 
              />
              
              {/* 디바이스 모니터링 섹션 */}
              <DeviceMonitoringCard 
                systemData={timeSeriesData?.system || []}
                isAdminMode={true} 
              />
            </div>
          )}

          {selectedSection === 'notices' && (
            <div className="admin-content">
              <div className="flex justify-between items-center mb-4">
                <h1 className="text-2xl font-semibold text-white">공지사항 관리</h1>
                {/* <Button onClick={openNoticeCreateModal} size="sm">
                  <Plus className="h-4 w-4 mr-1" /> 공지사항 등록
                </Button> */}
              </div>
              
              {/* 시스템 알림 섹션 */}
              {hasAlerts(timeSeriesData.environment) && (
                <div className="admin-card content-section bg-gray-700">
                  <h2 className="text-xl font-medium text-white mb-4">시스템 알림</h2>
                  
                  <div className="space-y-4">
                    {getLatestData(timeSeriesData.environment)?.is_fullerror === 1 && (
                      <div className="bg-red-900 bg-opacity-30 border border-red-500 rounded-lg p-4 flex items-start">
                        <AlertTriangle className="h-6 w-6 text-red-500 mr-3" />
                        <div>
                          <h3 className="text-red-400 font-semibold">불량통 가득 참</h3>
                          <p className="text-gray-300 text-sm mt-1">
                            불량통이 가득 찼습니다. 즉시 비워주세요. 시스템 작동이 중단될 수 있습니다.
                          </p>
                        </div>
                      </div>
                    )}
                    
                    {getLatestData(timeSeriesData.environment)?.is_fullnormal === 1 && (
                      <div className="bg-yellow-900 bg-opacity-30 border border-yellow-500 rounded-lg p-4 flex items-start">
                        <AlertTriangle className="h-6 w-6 text-yellow-500 mr-3" />
                        <div>
                          <h3 className="text-yellow-400 font-semibold">방전완료통 가득 참</h3>
                          <p className="text-gray-300 text-sm mt-1">
                            방전완료통이 가득 찼습니다. 비워주세요. 추가 배터리 처리가 불가능합니다.
                          </p>
                        </div>
                      </div>
                    )}
                  </div>
                </div>
              )}
              
              {/* 공지사항 목록 */}
              <div className="admin-card bg-gray-700 shadow-md">
                <div className="card-header border-gray-600">
                  <div className="flex justify-between items-center">
                    <div>
                      <h2 className="text-lg font-medium flex items-center text-white">
                        <Bell className="h-5 w-5 mr-2" />
                        공지사항 목록
                      </h2>
                      <p className="text-gray-400 text-sm mt-1">
                        전체 사용자에게 공지되는 메시지를 관리합니다.
                      </p>
                    </div>
                    <Button onClick={openNoticeCreateModal} size="sm">
                      <Plus className="h-6 w-6 mr-1" /> 공지사항 등록
                    </Button>
                  </div>
                </div>
                
                <div className="overflow-x-auto">
                  {isLoading ? (
                    <div className="card-body flex flex-col items-center justify-center py-12">
                      <div className="inline-block animate-spin rounded-full h-10 w-10 border-t-2 border-b-2 border-blue-500 mb-4"></div>
                      <p className="text-gray-400">공지사항을 불러오는 중...</p>
                    </div>
                  ) : error ? (
                    <div className="card-body flex flex-col items-center justify-center py-12">
                      <AlertTriangle className="h-14 w-14 text-red-500 mb-4" />
                      <p className="text-gray-300 text-center max-w-md mb-4">{error}</p>
                      <button 
                        onClick={() => setSelectedSection('notices')} 
                        className="px-4 py-2 rounded-md bg-blue-600 text-white hover:bg-blue-700"
                      >
                        다시 시도하기
                      </button>
                    </div>
                  ) : notices.length === 0 ? (
                    <div className="card-body flex flex-col items-center justify-center py-12">
                      <Info className="h-14 w-14 text-gray-500 mb-4" />
                      <p className="text-gray-400 mb-2">등록된 공지사항이 없습니다.</p>
                      <p className="text-gray-500 text-sm">위의 '공지사항 등록' 버튼을 클릭하여 공지사항을 작성하세요.</p>
                    </div>
                  ) : (
                    <table className="min-w-full table-fixed border-collapse">
                      <thead className="bg-gray-800">
                        <tr>
                          <th className="p-3 text-center w-[20%] text-gray-400">유형</th>
                          <th className="p-3 text-center w-[50%] text-gray-400">제목</th>
                          <th className="p-3 text-center w-[30%] text-gray-400">등록일</th>
                        </tr>
                      </thead>
                      <tbody className="divide-y bg-gray-800 divide-gray-700">
                        {notices.map((notice, index) => (
                          <tr key={index} className="hover:bg-gray-700">
                            <td className="p-3 text-center">
                              <span className={`inline-flex items-center justify-center px-2 py-1 rounded-full text-xs font-medium ${getNoticeTypeColorClass(notice.noticeType, true)}`}>
                                {renderNoticeTypeIcon(notice.noticeType)}
                                <span className="ml-1">{getNoticeTypeLabel(notice.noticeType)}</span>
                              </span>
                            </td>
                            <td 
                              className="p-3 text-center font-medium text-white cursor-pointer hover:text-blue-400"
                              onClick={() => handleNoticeClick(notice)}
                            >
                              {notice.noticeTitle}
                            </td>
                            <td className="p-3 text-center text-gray-400 text-sm">
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
          
          {selectedSection === 'users' && (
            <div className="admin-content">
              <div className="flex justify-between items-center mb-4">
                <h1 className="text-2xl font-semibold text-white">사용자 관리</h1>
              </div>
              
              {/* 사용자 관리 컴포넌트 */}
              <UserManagement 
                isAdminMode={true} 
                onAddUser={openUserSignupModal} 
              />
            </div>
          )}
          
          {selectedSection === 'batteries' && (
            <div className="admin-content">
              <div className="flex justify-between items-center mb-4">
                <h1 className="text-2xl font-semibold text-white">배터리 로그</h1>
              </div>
              
              {/* 배터리 로그 테이블 */}
              <BatteryLogTable isAdminMode={true} />
            </div>
          )}
        </div>
      </div>
      
      {/* 사용자 등록 모달 */}
      <UserSignupModal
        isOpen={showUserSignupModal}
        onClose={closeUserSignupModal}
        onSuccess={handleUserSignupSuccess}
      />
      
      {/* 공지사항 등록 모달 */}
      <NoticeCreateModal
        isOpen={showNoticeCreateModal}
        onClose={closeNoticeCreateModal}
        onSuccess={handleNoticeCreateSuccess}
      />
      
      {/* 공지사항 상세 모달 */}
      <NoticeDetailModal 
        isOpen={showNoticeModal}
        onClose={handleCloseModal}
        notice={selectedNotice}
        isAdminMode={true}
      />
    </div>
  );
};

export default AdminPage; 