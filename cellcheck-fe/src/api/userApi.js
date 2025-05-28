// src/api/userApi.js
import { authAxios } from '../utils/AxiosInstance';
import { handleApiResponse, handleApiError } from '../utils/helpers';
import { API_ENDPOINTS } from '../utils/constants';

/**
 * 전체 사용자 정보 조회 (관리자 전용)
 * @returns {Promise<Object>} 사용자 목록과 상태 정보
 */
export const getUserInfo = async () => {
  try {
    console.log('사용자 API 호출 시작:', API_ENDPOINTS.USER_INFO);
    
    // 요청 헤더 확인을 위한 로깅
    const token = localStorage.getItem('accessToken');
    console.log('인증 토큰 존재 여부:', !!token);
    
    // 관리자 API는 404 오류를 반환하므로 바로 일반 사용자 정보 API만 호출
    const response = await authAxios.get(API_ENDPOINTS.USER_INFO);
    console.log('사용자 API 원본 응답:', response);
    
    if (response && response.data) {
      return processUserResponse(response);
    }
    
    console.error('사용자 API 응답이 유효하지 않음');
    return {
      success: false,
      message: '사용자 정보를 조회하는데 실패했습니다. (유효하지 않은 응답)',
      data: []
    };
  } catch (error) {
    console.error('사용자 정보 조회 실패 (예외 발생):', error);
    console.error('에러 상세 정보:', error.response || error.message);
    
    if (error.response && error.response.status === 401) {
      console.warn('인증 만료 또는 인증 정보가 유효하지 않습니다. 다시 로그인이 필요합니다.');
    }
    
    return handleApiError(error);
  }
};

/**
 * 사용자 API 응답 처리
 * @param {Object} response Axios 응답 객체
 * @returns {Object} 처리된 응답
 */
const processUserResponse = (response) => {
  console.log('사용자 API 데이터 타입:', typeof response.data);
  console.log('사용자 API 데이터:', response.data);
  
  // 1. 직접 배열인 경우
  if (Array.isArray(response.data)) {
    return {
      success: true,
      message: '사용자 정보를 성공적으로 조회했습니다.',
      data: response.data
    };
  }
  
  // 2. {data: [...]} 형식인 경우
  if (response.data.data && Array.isArray(response.data.data)) {
    return {
      success: true,
      message: '사용자 정보를 성공적으로 조회했습니다.',
      data: response.data.data
    };
  }
  
  // 3. {content: [...]} 형식인 경우 (페이지네이션)
  if (response.data.content && Array.isArray(response.data.content)) {
    return {
      success: true,
      message: '사용자 정보를 성공적으로 조회했습니다.',
      data: response.data.content
    };
  }
  
  // 4. 단일 객체인 경우 배열로 변환
  if (typeof response.data === 'object' && !Array.isArray(response.data) && response.data !== null) {
    // userId 필드가 있으면 사용자 객체로 간주
    if ('userId' in response.data) {
      return {
        success: true,
        message: '사용자 정보를 성공적으로 조회했습니다.',
        data: [response.data]
      };
    }
  }
  
  // 그 외의 경우는 원본 응답 처리를 통해 반환
  return handleApiResponse(response);
};

export default {
  getUserInfo
}; 