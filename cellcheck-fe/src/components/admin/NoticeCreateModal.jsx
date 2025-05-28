import React, { useState } from 'react';
import { X } from 'lucide-react';
import Button from '../common/Button';
import { createNotice } from '../../api/noticeApi';
import { NOTICE_TYPES } from '../../utils/constants';

const NoticeCreateModal = ({ isOpen, onClose, onSuccess }) => {
  const [title, setTitle] = useState('');
  const [content, setContent] = useState('');
  const [type, setType] = useState(2); // 기본값: 일반
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState('');

  // 폼 초기화 함수
  const resetForm = () => {
    setTitle('');
    setContent('');
    setType(2);
    setError('');
    setIsSubmitting(false);
  };

  // 모달 닫기 핸들러
  const handleClose = () => {
    resetForm();
    onClose();
  };

  // 폼 제출 핸들러
  const handleSubmit = async (e) => {
    e.preventDefault();
    
    // 입력 검증
    if (!title.trim()) {
      setError('제목을 입력해주세요.');
      return;
    }
    
    if (!content.trim()) {
      setError('내용을 입력해주세요.');
      return;
    }
    
    setIsSubmitting(true);
    setError('');
    
    try {
      const response = await createNotice({
        noticeTitle: title,
        noticeContent: content,
        noticeType: type
      });
      
      if (response.success) {
        resetForm();
        onSuccess && onSuccess(response.data);
        onClose();
      } else {
        setError(response.message || '공지사항 등록에 실패했습니다.');
      }
    } catch (err) {
      console.error('공지사항 등록 중 오류 발생:', err);
      setError('서버 오류가 발생했습니다. 나중에 다시 시도해주세요.');
    } finally {
      setIsSubmitting(false);
    }
  };

  // 모달이 닫혀있으면 아무것도 렌더링하지 않음
  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50 p-4">
      <div className="bg-gray-800 rounded-lg shadow-lg max-w-md w-full overflow-hidden relative">
        {/* 헤더 */}
        <div className="p-4 border-b border-gray-700 flex justify-between items-center">
          <h2 className="text-lg font-medium text-white">공지사항 등록</h2>
          <button 
            onClick={handleClose}
            className="text-gray-400 hover:text-white"
          >
            <X size={20} />
          </button>
        </div>
        
        {/* 내용 */}
        <form onSubmit={handleSubmit} className="p-4">
          {/* 에러 메시지 */}
          {error && (
            <div className="mb-4 p-3 bg-red-900 bg-opacity-30 border border-red-500 rounded text-red-400 text-sm">
              {error}
            </div>
          )}
          
          {/* 공지 유형 */}
          <div className="mb-4">
            <label className="block text-gray-300 text-sm font-medium mb-2">
              공지 유형
            </label>
            <select
              value={type}
              onChange={(e) => setType(Number(e.target.value))}
              className="w-full bg-gray-700 border border-gray-600 rounded-md py-2 px-3 text-white focus:outline-none focus:ring-2 focus:ring-blue-500"
            >
              <option value={NOTICE_TYPES.MAINTENANCE}>점검</option>
              <option value={NOTICE_TYPES.URGENT}>긴급</option>
              <option value={NOTICE_TYPES.NORMAL}>일반</option>
            </select>
          </div>
          
          {/* 제목 */}
          <div className="mb-4">
            <label className="block text-gray-300 text-sm font-medium mb-2">
              제목
            </label>
            <input
              type="text"
              value={title}
              onChange={(e) => setTitle(e.target.value)}
              className="w-full bg-gray-700 border border-gray-600 rounded-md py-2 px-3 text-white focus:outline-none focus:ring-2 focus:ring-blue-500"
              placeholder="공지사항 제목을 입력하세요"
            />
          </div>
          
          {/* 내용 */}
          <div className="mb-4">
            <label className="block text-gray-300 text-sm font-medium mb-2">
              내용
            </label>
            <textarea
              value={content}
              onChange={(e) => setContent(e.target.value)}
              className="w-full bg-gray-700 border border-gray-600 rounded-md py-2 px-3 text-white focus:outline-none focus:ring-2 focus:ring-blue-500 min-h-[120px]"
              placeholder="공지사항 내용을 입력하세요"
            />
          </div>
          
          {/* 버튼 영역 */}
          <div className="flex justify-end space-x-2 mt-6">
            <Button 
              variant="outline" 
              onClick={handleClose}
              disabled={isSubmitting}
            >
              취소
            </Button>
            <Button 
              type="submit"
              variant="primary"
              disabled={isSubmitting}
            >
              {isSubmitting ? '등록 중...' : '등록하기'}
            </Button>
          </div>
        </form>
      </div>
    </div>
  );
};

export default NoticeCreateModal; 