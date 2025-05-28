import React from 'react';
import { X } from 'lucide-react';
import AdminSignupForm from './AdminSignupForm';

const AdminSignupModal = ({ isOpen, onClose, onSuccess }) => {
  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 flex items-center justify-center z-50">
      {/* 배경 오버레이 */}
      <div 
        className="fixed inset-0 bg-black bg-opacity-75 transition-opacity"
        onClick={onClose}
      ></div>
      
      {/* 모달 컨텐츠 */}
      <div className="relative bg-gray-800 rounded-lg shadow-xl max-w-md w-full mx-4 sm:mx-auto p-6 overflow-hidden z-10">
        {/* 닫기 버튼 */}
        <button
          onClick={onClose}
          className="absolute top-4 right-4 text-gray-400 hover:text-white"
        >
          <X className="h-6 w-6" />
        </button>
        
        {/* 관리자 회원가입 폼 */}
        <AdminSignupForm 
          onSuccess={() => {
            onSuccess && onSuccess();
            onClose();
          }}
          onCancel={onClose}
        />
      </div>
    </div>
  );
};

export default AdminSignupModal; 