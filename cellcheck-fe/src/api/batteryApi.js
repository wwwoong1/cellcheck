import { authAxios } from '../utils/AxiosInstance';
import { handleApiResponse, handleApiError } from '../utils/helpers';
import { API_ENDPOINTS } from '../utils/constants';

/**
 * 배터리 관련 API 함수들
 */

/**
 * 배터리 로그 조회
 * @param {number} days - 조회할 일수 (기본값: 0, 전체)
 * @param {number} page - 페이지 번호 (기본값: 1)
 * @param {number} size - 페이지당 항목 수 (기본값: 10)
 * @returns {Promise<Object>} 배터리 로그 데이터
 */
export const getBatteryLogs = async (days = 0, page = 1, size = 10) => {
  try {
    console.log(`배터리 로그 API 호출 - days: ${days}, page: ${page}, size: ${size}`);
    
    const response = await authAxios.get(API_ENDPOINTS.BATTERY_LOGS, {
      params: {
        days,
        page: page - 1, // 백엔드 페이지 인덱스가 0부터 시작하는 경우
        size
      }
    });

    // 원본 응답 데이터 로깅 (디버깅용)
    console.log("배터리 로그 API 원본 응답:", response);
    console.log("배터리 로그 데이터:", response.data);

    // 성공적인 응답이지만 데이터가 없는 경우
    if (!response.data) {
      console.log("응답에 데이터가 없음");
      return {
        success: true,
        message: '배터리 로그가 없습니다.',
        data: [],
        meta: {
          totalItems: 0,
          currentPage: page,
          totalPages: 1,
          pageSize: size
        }
      };
    }

    // 직접 배열인 경우 (사용자가 공유한 예시 데이터와 같은 형태)
    if (Array.isArray(response.data)) {
      console.log("배열 형태의 데이터 발견:", response.data.length, "개의 로그");
      return {
        success: true,
        message: '배터리 로그를 성공적으로 조회했습니다.',
        data: response.data,
        meta: {
          totalItems: response.data.length,
          currentPage: page,
          totalPages: Math.ceil(response.data.length / size),
          pageSize: size
        }
      };
    }
    
    // 페이지네이션 객체인 경우 (Spring Data JPA의 Page 객체)
    if (response.data.content && Array.isArray(response.data.content)) {
      return {
        success: true,
        message: '배터리 로그를 성공적으로 조회했습니다.',
        data: response.data.content,
        meta: {
          totalItems: response.data.totalElements || 0,
          currentPage: response.data.number + 1 || page, // 백엔드는 0부터 시작하는 인덱스를 사용할 수 있음
          totalPages: response.data.totalPages || 1,
          pageSize: response.data.size || size
        }
      };
    }

    // 그 외의 객체 구조인 경우 (데이터 속성이 다른 이름일 수 있음)
    const possibleArrayFields = ['data', 'logs', 'items', 'list', 'results', 'batteries'];
    for (const field of possibleArrayFields) {
      if (response.data[field] && Array.isArray(response.data[field])) {
        return {
          success: true,
          message: '배터리 로그를 성공적으로 조회했습니다.',
          data: response.data[field],
          meta: {
            totalItems: response.data.totalItems || response.data.count || response.data[field].length,
            currentPage: page,
            totalPages: response.data.totalPages || Math.ceil((response.data.totalItems || response.data[field].length) / size),
            pageSize: size
          }
        };
      }
    }
    
    // 기타 응답 형식 - 단순히 응답 전체를 데이터로 사용
    console.log("표준 형식이 아닌 응답, 전체 응답을 데이터로 사용");
    return {
      success: true,
      message: '배터리 로그를 성공적으로 조회했습니다.',
      data: Array.isArray(response.data) ? response.data : [response.data],
      meta: {
        totalItems: Array.isArray(response.data) ? response.data.length : 1,
        currentPage: page,
        totalPages: 1,
        pageSize: size
      }
    };

  } catch (error) {
    console.error('배터리 로그 조회 실패:', error);
    console.error('에러 상세:', error.response || error.message);
    
    if (error.response && error.response.status === 401) {
      console.warn('인증 만료 또는 인증 정보가 유효하지 않습니다.');
    }
    
    return handleApiError(error);
  }
};

export default {
  getBatteryLogs
}; 