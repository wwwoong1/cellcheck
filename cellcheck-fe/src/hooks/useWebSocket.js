// src/hooks/useWebSocket.js
import { useState, useEffect, useRef, useCallback } from 'react';

// 로컬 스토리지 키
const STORAGE_KEY_PREFIX = 'cellcheck_ws_data_';
const STORAGE_KEYS = {
  ENVIRONMENT: `${STORAGE_KEY_PREFIX}environment`,
  APPEARANCE: `${STORAGE_KEY_PREFIX}appearance`,
  DISCHARGE: `${STORAGE_KEY_PREFIX}discharge`,
  SYSTEM: `${STORAGE_KEY_PREFIX}system`,
  TIMESTAMP: `${STORAGE_KEY_PREFIX}last_update`
};

// UTC 시간을 한국 시간(KST)으로 변환하는 함수
const convertToKST = (utcTimeString) => {
  try {
    if (!utcTimeString) return new Date().toISOString();
    
    const utcDate = new Date(utcTimeString);
    
    // 유효한 날짜인지 확인
    if (isNaN(utcDate.getTime())) return new Date().toISOString();
    
    // 한국 시간으로 변환 (UTC+9)
    const kstDate = new Date(utcDate.getTime() + 9 * 60 * 60 * 1000);
    return kstDate.toISOString();
  } catch (e) {
    console.error('시간 변환 오류:', e);
    return new Date().toISOString();
  }
};

// 로컬 스토리지에서 데이터 로드
const loadDataFromStorage = () => {
  try {
    const initialData = {
      environment: [],
      appearance: [],
      discharge: [],
      system: []
    };

    // 각 데이터 타입별로 로컬 스토리지에서 로드
    Object.keys(initialData).forEach(key => {
      const storedData = localStorage.getItem(STORAGE_KEYS[key.toUpperCase()]);
      if (storedData) {
        initialData[key] = JSON.parse(storedData);
        console.log(`[WebSocket] ${key} 데이터 로컬 스토리지에서 복원: ${initialData[key].length}개`);
      }
    });

    return initialData;
  } catch (error) {
    console.error('로컬 스토리지에서 데이터 로드 중 오류:', error);
    return {
      environment: [],
      appearance: [],
      discharge: [],
      system: []
    };
  }
};

// 로컬 스토리지에 데이터 저장
const saveDataToStorage = (dataType, dataArray) => {
  try {
    if (Array.isArray(dataArray) && dataArray.length > 0) {
      // 최대 30개만 저장
      const dataToSave = dataArray.slice(-30);
      localStorage.setItem(STORAGE_KEYS[dataType.toUpperCase()], JSON.stringify(dataToSave));
      localStorage.setItem(STORAGE_KEYS.TIMESTAMP, new Date().toISOString());
    }
  } catch (error) {
    console.error(`${dataType} 데이터 저장 중 오류:`, error);
  }
};

export default function useWebSocket(url) {
  // 로컬 스토리지에서 초기 데이터 로드
  const initialData = loadDataFromStorage();
  
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  const [data, setData] = useState({
    environment: null,
    appearance: null,
    discharge: null,
    system: null
  });
  
  // 누적 데이터 상태 (새로고침해도 유지됨)
  const [accumulatedData, setAccumulatedData] = useState(initialData);
  
  const wsRef = useRef(null);
  const [messageHistory, setMessageHistory] = useState([]);
  const reconnectTimeoutRef = useRef(null);
  const reconnectAttemptsRef = useRef(0);
  
  // WebSocket 연결 - 서버 시작 시점부터 항상 연결 시도
  const connectWebSocket = useCallback(() => {
    // 이미 연결 시도 중이면 중복 연결 방지
    if (wsRef.current && wsRef.current.readyState === WebSocket.CONNECTING) {
      return; 
    }
    
    // 재연결 시도 횟수 증가
    reconnectAttemptsRef.current += 1;
    const currentAttempt = reconnectAttemptsRef.current;
    
    // 백오프 전략 (지수 백오프) - 최대 30초
    const backoffTime = Math.min(5000 * Math.pow(1.5, currentAttempt - 1), 30000);
    
    try {
      console.log(`WebSocket 연결 시도 (${currentAttempt}번째): ${url}`);
      const ws = new WebSocket(url);
      
      ws.onopen = () => {
        console.log('WebSocket 연결 성공');
        setConnected(true);
        setError(null);
        reconnectAttemptsRef.current = 0; // 연결 성공 시 재시도 카운터 초기화
        
        // 재연결 타임아웃 초기화
        if (reconnectTimeoutRef.current) {
          clearTimeout(reconnectTimeoutRef.current);
          reconnectTimeoutRef.current = null;
        }
      };
      
      ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          // console.log('WebSocket 메시지 수신:', message); // 로그 양 줄이기
          
          // 최근 메시지 히스토리 업데이트
          setMessageHistory(prev => {
            const newMessages = [message, ...prev];
            return newMessages.slice(0, 20); // 최대 20개 메시지 유지
          });
          
          // 데이터 타입에 따라 상태 업데이트
          if (message.type) {
            // 현재 메시지 상태 업데이트
            setData(prevData => ({
              ...prevData,
              [message.type]: message
            }));
            
            // 시간 정보 추가 및 한국 시간으로 변환
            const now = new Date().toISOString();
            const utcTimestamp = message.sent_at || message.recorded_at || message.timestamp || now;
            const kstTimestamp = convertToKST(utcTimestamp);
            
            const dataWithTimestamp = {
              ...message,
              timestamp: kstTimestamp,
              recorded_at: kstTimestamp,
              original_timestamp: utcTimestamp  // 원본 UTC 시간 백업
            };
            
            // 누적 데이터 업데이트 및 로컬 스토리지에 저장
            setAccumulatedData(prevData => {
              const updatedData = {
                ...prevData,
                [message.type]: [...(prevData[message.type] || []), dataWithTimestamp].slice(-30)
              };
              
              // 로컬 스토리지에 저장
              saveDataToStorage(message.type, updatedData[message.type]);
              
              return updatedData;
            });
          }
        } catch (e) {
          console.error('WebSocket 메시지 처리 오류:', e);
        }
      };
      
      ws.onclose = (event) => {
        console.log('WebSocket 연결 종료:', event.code, event.reason);
        setConnected(false);
        
        // 자동 재연결 시도 - 계속해서 서버와 연결 유지 시도
        if (!reconnectTimeoutRef.current) {
          reconnectTimeoutRef.current = setTimeout(() => {
            console.log(`WebSocket 재연결 시도... 대기 시간: ${backoffTime}ms`);
            connectWebSocket();
            reconnectTimeoutRef.current = null;
          }, backoffTime);
        }
      };
      
      ws.onerror = (event) => {
        console.error('WebSocket 오류 발생:', event);
        setError('WebSocket 연결 중 오류가 발생했습니다.');
      };
      
      wsRef.current = ws;
    } catch (error) {
      console.error('WebSocket 연결 시도 중 오류:', error);
      setError(`WebSocket 연결 오류: ${error.message}`);
      
      // 연결 오류 시에도 재연결 시도
      if (!reconnectTimeoutRef.current) {
        reconnectTimeoutRef.current = setTimeout(() => {
          console.log(`WebSocket 재연결 시도... 대기 시간: ${backoffTime}ms`);
          connectWebSocket();
          reconnectTimeoutRef.current = null;
        }, backoffTime);
      }
    }
  }, [url]);
  
  // 페이지 로드 즉시 WebSocket 연결 시작
  useEffect(() => {
    // 애플리케이션 시작 시 즉시 연결
    connectWebSocket();
    
    // Visibility 변경 감지 - 탭 활성화 시 재연결
    const handleVisibilityChange = () => {
      if (document.visibilityState === 'visible') {
        // 탭이 포커스를 받았을 때 연결 상태 확인 및 필요시 재연결
        if (wsRef.current && wsRef.current.readyState !== WebSocket.OPEN) {
          console.log('페이지 가시성 변경 - WebSocket 재연결 시도');
          reconnect();
        }
      }
    };
    
    document.addEventListener('visibilitychange', handleVisibilityChange);
    
    // 개발 환경에서 테스트 이벤트 리스너 추가
    if (process.env.NODE_ENV === 'development') {
      const handleTestEvent = (event) => {
        const message = event.detail.data;
        console.log('WebSocket 테스트 메시지:', message);
        
        setMessageHistory(prev => {
          const newMessages = [message, ...prev];
          return newMessages.slice(0, 20);
        });
        
        if (message.type) {
          setData(prevData => ({
            ...prevData,
            [message.type]: message
          }));
          
          // 테스트 데이터도 누적 데이터에 추가
          setAccumulatedData(prevData => {
            const updatedData = {
              ...prevData,
              [message.type]: [...(prevData[message.type] || []), message].slice(-30)
            };
            
            // 로컬 스토리지에 저장
            saveDataToStorage(message.type, updatedData[message.type]);
            
            return updatedData;
          });
        }
      };
      
      window.addEventListener('websocket-test', handleTestEvent);
      return () => {
        document.removeEventListener('visibilitychange', handleVisibilityChange);
        window.removeEventListener('websocket-test', handleTestEvent);
        if (wsRef.current) wsRef.current.close();
        if (reconnectTimeoutRef.current) clearTimeout(reconnectTimeoutRef.current);
      };
    }
    
    // 프로덕션 환경 클린업
    return () => {
      document.removeEventListener('visibilitychange', handleVisibilityChange);
      if (wsRef.current) wsRef.current.close();
      if (reconnectTimeoutRef.current) clearTimeout(reconnectTimeoutRef.current);
    };
  }, [connectWebSocket]);
  
  // 수동 재연결 함수
  const reconnect = useCallback(() => {
    if (wsRef.current) wsRef.current.close();
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
    // 재연결 시도 카운터 초기화 (즉시 재시도)
    reconnectAttemptsRef.current = 0;
    connectWebSocket();
  }, [connectWebSocket]);
  
  return { 
    connected, 
    error, 
    data, 
    accumulatedData, // 누적된 데이터 추가 (새로고침해도 유지됨)
    messageHistory, 
    reconnect 
  };
}