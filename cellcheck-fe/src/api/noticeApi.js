import { authAxios } from '../utils/AxiosInstance';
import { handleApiResponse, handleApiError } from '../utils/helpers';
import { API_ENDPOINTS } from '../utils/constants';

/**
 * 공지사항 관련 API 함수들
 */

// 공지사항 목록 조회 (사용자는 자신의 공지만, 관리자는 전체 공지)
export const getNotices = async () => {
  try {
    const response = await authAxios.get(API_ENDPOINTS.NOTICES);
    console.log("공지사항 API 응답:", response);
    
    // 백엔드 응답 형식에 맞게 처리
    if (response.data) {
      console.log("공지사항 데이터:", response.data);
      return {
        success: true,
        message: '공지사항을 성공적으로 조회했습니다.',
        data: response.data
      };
    }
    
    return handleApiResponse(response);
  } catch (error) {
    console.error('공지사항 조회 실패:', error.message);
    return handleApiError(error);
  }
};

// 공지사항 등록 (관리자 전용)
export const createNotice = async (noticeData) => {
  try {
    const { noticeTitle, noticeContent, noticeType } = noticeData;
    
    const response = await authAxios.post(API_ENDPOINTS.CREATE_NOTICE, {
      noticeTitle,
      noticeContent,
      noticeType
    });
    
    return handleApiResponse(response);
  } catch (error) {
    console.error('공지사항 등록 실패:', error.message);
    return handleApiError(error);
  }
};

// 공지사항 타입에 따른 레이블 반환
export const getNoticeTypeLabel = (type) => {
  switch (type) {
    case 0:
      return '점검';
    case 1:
      return '긴급';
    case 2:
      return '일반';
    default:
      return '기타';
  }
};

// 공지사항 타입에 따른 색상 클래스 반환
export const getNoticeTypeColorClass = (type, isDarkMode = false) => {
  switch (type) {
    case 0: // 점검
      return isDarkMode ? 'bg-yellow-900 text-yellow-300' : 'bg-yellow-100 text-yellow-800';
    case 1: // 긴급
      return isDarkMode ? 'bg-red-900 text-red-300' : 'bg-red-100 text-red-800';
    case 2: // 일반
      return isDarkMode ? 'bg-blue-900 text-blue-300' : 'bg-blue-100 text-blue-800';
    default:
      return isDarkMode ? 'bg-gray-700 text-gray-300' : 'bg-gray-200 text-gray-800';
  }
};

// 상대 시간 포맷팅 유틸리티
export const formatRelativeTime = (dateString) => {
  const date = new Date(dateString);
  const now = new Date();
  const diffMs = now - date;
  const diffSec = Math.floor(diffMs / 1000);
  const diffMin = Math.floor(diffSec / 60);
  const diffHour = Math.floor(diffMin / 60);
  const diffDay = Math.floor(diffHour / 24);
  
  if (diffSec < 60) {
    return '방금 전';
  } else if (diffMin < 60) {
    return `${diffMin}분 전`;
  } else if (diffHour < 24) {
    return `${diffHour}시간 전`;
  } else if (diffDay < 7) {
    return `${diffDay}일 전`;
  } else {
    return `${date.getFullYear()}.${String(date.getMonth() + 1).padStart(2, '0')}.${String(date.getDate()).padStart(2, '0')}`;
  }
}; 