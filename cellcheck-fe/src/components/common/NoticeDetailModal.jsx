import React from 'react';
import { X, Bell, Wrench, AlertTriangle, Info } from 'lucide-react';
import Button from './Button';
import { getNoticeTypeLabel, getNoticeTypeColorClass, formatRelativeTime } from '../../api/noticeApi';

const NoticeDetailModal = ({ isOpen, onClose, notice, isAdminMode = false }) => {
  if (!isOpen || !notice) return null;

  // 공지사항 타입에 따른 아이콘 렌더링
  const renderNoticeTypeIcon = (type) => {
    switch (type) {
      case 0: // 점검
        return <Wrench className={`h-5 w-5 ${isAdminMode ? 'text-yellow-400' : 'text-yellow-600'}`} />;
      case 1: // 긴급
        return <AlertTriangle className={`h-5 w-5 ${isAdminMode ? 'text-red-400' : 'text-red-600'}`} />;
      case 2: // 일반
        return <Info className={`h-5 w-5 ${isAdminMode ? 'text-blue-400' : 'text-blue-600'}`} />;
      default:
        return <Info className={`h-5 w-5 ${isAdminMode ? 'text-gray-400' : 'text-gray-600'}`} />;
    }
  };

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center">
      {/* 배경 오버레이 */}
      <div 
        className="absolute inset-0 bg-black bg-opacity-50" 
        onClick={onClose}
      />
      
      {/* 모달 컨텐츠 */}
      <div className={`relative w-full max-w-2xl rounded-lg shadow-lg overflow-hidden ${
        isAdminMode ? 'bg-gray-800' : 'bg-white'
      }`}>
        {/* 모달 헤더 */}
        <div className={`px-6 py-4 border-b ${
          isAdminMode ? 'border-gray-700' : 'border-gray-200'
        }`}>
          <div className="flex items-center justify-between">
            <div className="flex items-center space-x-2">
              <Bell className={`h-5 w-5 ${
                isAdminMode ? 'text-blue-400' : 'text-blue-600'
              }`} />
              <h3 className={`text-lg font-semibold ${
                isAdminMode ? 'text-white' : 'text-gray-900'
              }`}>
                공지사항 상세
              </h3>
            </div>
            <button
              onClick={onClose}
              className={`rounded-full p-1 ${
                isAdminMode 
                  ? 'text-gray-400 hover:bg-gray-700 hover:text-gray-200' 
                  : 'text-gray-500 hover:bg-gray-100 hover:text-gray-700'
              }`}
            >
              <X className="h-5 w-5" />
            </button>
          </div>
        </div>
        
        {/* 모달 본문 */}
        <div className={`px-6 py-4 ${
          isAdminMode ? 'bg-gray-800' : 'bg-white'
        }`}>
          <div className="mb-6">
            <div className="flex items-center mb-2">
              <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium mr-2 ${
                getNoticeTypeColorClass(notice.noticeType, isAdminMode)
              }`}>
                {renderNoticeTypeIcon(notice.noticeType)}
                <span className="ml-1">{getNoticeTypeLabel(notice.noticeType)}</span>
              </span>
              <span className={`text-sm ${
                isAdminMode ? 'text-gray-400' : 'text-gray-500'
              }`}>
                {formatRelativeTime(notice.createdAt)}
              </span>
            </div>
            
            <h4 className={`text-xl font-medium mb-2 ${
              isAdminMode ? 'text-white' : 'text-gray-900'
            }`}>
              {notice.noticeTitle}
            </h4>
          </div>
          
          <div className={`border-t border-b py-4 mb-4 ${
            isAdminMode ? 'border-gray-700 text-gray-300' : 'border-gray-200 text-gray-700'
          }`}>
            <p className="whitespace-pre-line">{notice.noticeContent}</p>
          </div>
        </div>
        
        {/* 모달 푸터 */}
        <div className={`px-6 py-3 flex justify-end ${
          isAdminMode ? 'bg-gray-800' : 'bg-gray-50'
        }`}>
          <Button
            variant={isAdminMode ? "secondary" : "outline"}
            onClick={onClose}
          >
            닫기
          </Button>
        </div>
      </div>
    </div>
  );
};

export default NoticeDetailModal; 