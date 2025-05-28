import React, { useState, useEffect } from 'react';
import { Search, Users, UserPlus, UserCheck, AlertTriangle, RefreshCw } from 'lucide-react';
import { getUserInfo } from '../../api/userApi';
import Button from '../common/Button';

const UserManagement = ({ isAdminMode = true, onAddUser }) => {
  const [users, setUsers] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [searchTerm, setSearchTerm] = useState('');
  const [retryCount, setRetryCount] = useState(0);

  // 사용자 목록 불러오기
  const fetchUsers = async (isRetry = false) => {
    if (isRetry) {
      setRetryCount(prev => prev + 1);
    } else if (!loading) {
      setLoading(true);
    }

    try {
      // 토큰 확인
      const token = localStorage.getItem('accessToken');
      console.log('사용자 API 호출 전 토큰 존재 여부:', !!token);
      if (!token) {
        console.error('인증 토큰이 없습니다. 로그인이 필요합니다.');
        setError('인증 정보가 없습니다. 다시 로그인해 주세요.');
        setUsers([]);
        setLoading(false);
        return;
      }

      const response = await getUserInfo();
      console.log('사용자 API 응답:', response); // 디버깅용 로그 추가
      
      if (response.success && Array.isArray(response.data)) {
        // 성공 시 데이터 설정
        console.log('사용자 데이터 개수:', response.data.length);
        
        // 각 사용자 항목의 필드 구조 확인
        if (response.data.length > 0) {
          console.log('첫 번째 사용자 객체 구조:', response.data[0]);
          
          // 배터리 로그 데이터인지 확인 (배터리 로그로 사용 가능한지)
          const firstItem = response.data[0];
          if (firstItem.batteryType !== undefined && firstItem.inspectionResult !== undefined) {
            console.log('받은 데이터는 배터리 로그 형식입니다. 배터리 데이터로 사용합니다.');
            // 필요한 경우 데이터 변환 로직을 여기에 추가
          }
        }
        
        setUsers(response.data);
        setError(null);
      } else {
        console.error('사용자 정보 조회 실패:', response.message);
        setError(response.message || '사용자 정보를 불러오는데 실패했습니다.');
        setUsers([]);
        
        // API 응답이 오류를 의미하는지 확인
        if (response.status === 401 || (response.message && response.message.includes('인증'))) {
          console.warn('인증 관련 오류가 감지되었습니다.');
          setError('인증이 만료되었습니다. 다시 로그인해 주세요.');
          
          // 로그인 페이지로 리다이렉트하지 않고, 사용자에게 알림만 표시
          return;
        }
        
        // 실패 시 자동 재시도 (최대 2번까지만)
        if (retryCount < 2) {
          console.log(`자동 재시도 (${retryCount + 1}/2)...`);
          setTimeout(() => fetchUsers(true), 1500);
          return;
        }
      }
    } catch (err) {
      console.error('사용자 정보 조회 오류:', err);
      
      // 인증 오류 특별 처리
      if (err.response && err.response.status === 401) {
        setError('인증이 만료되었습니다. 다시 로그인해 주세요.');
      } else {
        setError('사용자 정보를 불러오는데 실패했습니다: ' + (err.message || '알 수 없는 오류'));
      }
      
      setUsers([]);
    } finally {
      setLoading(false);
    }
  };

  // 컴포넌트 마운트 시 데이터 로드
  useEffect(() => {
    fetchUsers();
  }, []);

  // 검색어로 필터링 - DTO 필드와 일치하도록 수정
  const filteredUsers = users.filter(user => 
    (user.userName || '').toLowerCase().includes(searchTerm.toLowerCase()) || 
    (user.region || '').toLowerCase().includes(searchTerm.toLowerCase())
  );

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

  // 수동으로 다시 불러오기
  const handleRefresh = () => {
    setRetryCount(0); // 재시도 카운트 초기화
    fetchUsers();
  };

  return (
    <div className={`admin-card ${isAdminMode ? 'bg-gray-700' : 'bg-white'} shadow-md`}>
      <div className={`card-header ${isAdminMode ? 'border-gray-600' : 'border-gray-200'} flex justify-between items-center`}>
        <h2 className={`text-lg font-medium ${isAdminMode ? 'text-white' : 'text-gray-800'}`}>
          <Users className="h-5 w-5 inline-block mr-2" />
          사용자 관리
        </h2>
        
        <div className="flex space-x-3 items-center">
          <button 
            className={`p-2 rounded-md ${isAdminMode ? 'bg-gray-600 text-gray-200 hover:bg-gray-500' : 'bg-gray-200 text-gray-700 hover:bg-gray-300'}`}
            onClick={handleRefresh}
            disabled={loading}
            title="새로고침"
          >
            <RefreshCw className={`h-4 w-4 ${loading ? 'animate-spin' : ''}`} />
          </button>
          
          <div className={`relative rounded-md shadow-sm ${isAdminMode ? 'bg-gray-800' : 'bg-gray-100'}`}>
            <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
              <Search className={`h-4 w-4 ${isAdminMode ? 'text-gray-400' : 'text-gray-500'}`} />
            </div>
            <input
              type="text"
              placeholder="이름 또는 지역 검색"
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className={`pl-10 pr-4 py-2 border-0 rounded-md focus:ring-2 focus:outline-none text-sm ${
                isAdminMode 
                  ? 'bg-gray-800 text-white focus:ring-blue-500' 
                  : 'bg-gray-100 text-gray-800 focus:ring-blue-400'
              }`}
            />
          </div>
          
          <Button 
            variant={isAdminMode ? "primary" : "default"} 
            size="sm" 
            onClick={onAddUser}
          >
            <UserPlus className="h-5 w-5 mr-1" /> 사용자 추가
          </Button>
        </div>
      </div>
      
      <div className="overflow-x-auto">
        {loading ? (
          <div className={`card-body flex flex-col items-center justify-center py-12 ${isAdminMode ? 'bg-gray-800' : 'bg-gray-50'}`}>
            <div className="inline-block animate-spin rounded-full h-10 w-10 border-t-2 border-b-2 border-blue-500 mb-4"></div>
            <p className={isAdminMode ? "text-gray-400" : "text-gray-500"}>사용자 정보를 불러오는 중...</p>
          </div>
        ) : error ? (
          <div className={`card-body flex flex-col items-center justify-center py-12 ${isAdminMode ? 'bg-gray-800' : 'bg-gray-50'}`}>
            <AlertTriangle className={`h-14 w-14 ${isAdminMode ? 'text-red-500' : 'text-red-600'} mb-4`} />
            <p className={`${isAdminMode ? "text-gray-300" : "text-gray-700"} text-center max-w-md mb-4`}>{error}</p>
            <button 
              className={`px-4 py-2 rounded-md ${isAdminMode ? 'bg-blue-600 text-white hover:bg-blue-700' : 'bg-blue-100 text-blue-700 hover:bg-blue-200'}`}
              onClick={handleRefresh}
            >
              다시 시도하기
            </button>
          </div>
        ) : filteredUsers.length === 0 ? (
          <div className={`card-body flex flex-col items-center justify-center py-12 ${isAdminMode ? 'bg-gray-800' : 'bg-gray-50'}`}>
            <Users className={`h-14 w-14 ${isAdminMode ? 'text-gray-500' : 'text-gray-400'} mb-4`} />
            {searchTerm ? (
              <p className={isAdminMode ? "text-gray-400" : "text-gray-500"}>검색 결과가 없습니다.</p>
            ) : (
              <p className={isAdminMode ? "text-gray-400" : "text-gray-500"}>등록된 사용자가 없습니다.</p>
            )}
          </div>
        ) : (
          <table className="min-w-full table-auto border-collapse">
            <thead className={isAdminMode ? 'bg-gray-800' : 'bg-gray-50'}>
              <tr>
                <th scope="col" className={`p-3 text-center w-[15%] ${
                  isAdminMode ? 'text-gray-400' : 'text-gray-500'
                }`}>
                  ID
                </th>
                <th scope="col" className={`p-3 text-center w-[30%] ${
                  isAdminMode ? 'text-gray-400' : 'text-gray-500'
                }`}>
                  이름
                </th>
                <th scope="col" className={`p-3 text-center w-[25%] ${
                  isAdminMode ? 'text-gray-400' : 'text-gray-500'
                }`}>
                  지역
                </th>
                <th scope="col" className={`p-3 text-center w-[30%] ${
                  isAdminMode ? 'text-gray-400' : 'text-gray-500'
                }`}>
                  등록일
                </th>
              </tr>
            </thead>
            <tbody className={`divide-y ${
              isAdminMode ? 'bg-gray-800 divide-gray-700' : 'bg-white divide-gray-200'
            }`}>
              {filteredUsers.map((user) => (
                <tr key={user.userId || user.id} className={
                  isAdminMode ? 'hover:bg-gray-700' : 'hover:bg-gray-50'
                }>
                  <td className={`p-3 text-center ${
                    isAdminMode ? 'text-gray-300' : 'text-gray-800'
                  }`}>
                    {user.userId || user.id || '-'}
                  </td>
                  <td className={`p-3 text-center ${
                    isAdminMode ? 'text-white' : 'text-gray-900'
                  }`}>
                    <div className="flex items-center justify-center">
                      <UserCheck className={`h-5 w-5 mr-2 ${
                        isAdminMode ? 'text-blue-400' : 'text-blue-500'
                      }`} />
                      {user.userName || user.name || '이름 없음'}
                    </div>
                  </td>
                  <td className={`p-3 text-center ${
                    isAdminMode ? 'text-gray-300' : 'text-gray-800'
                  }`}>
                    {user.region || '지역 정보 없음'}
                  </td>
                  <td className={`p-3 text-center ${
                    isAdminMode ? 'text-gray-400' : 'text-gray-500'
                  }`}>
                    {formatDate(user.createdAt)}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </div>
    </div>
  );
};

export default UserManagement; 