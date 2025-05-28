// src/components/auth/AdminKeyModal.jsx
import React, { useState } from 'react';
import { Lock, Eye, EyeOff, ShieldAlert, AlertTriangle, X } from 'lucide-react';
import Modal from '../common/Modal';
import Input from '../common/Input';
import Button from '../common/Button';

const AdminKeyModal = ({ 
  isOpen, 
  onClose, 
  onSubmit 
}) => {
  const [adminKey, setAdminKey] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [adminKeyError, setAdminKeyError] = useState('');

  const handleAdminKeyChange = (e) => {
    setAdminKey(e.target.value);
    setAdminKeyError('');
  };

  const handlePasswordVisibility = () => {
    setShowPassword(!showPassword);
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter') {
      handleSubmit();
    }
  };

  const handleSubmit = () => {
    if (!adminKey.trim()) {
      setAdminKeyError('관리자 키를 입력해주세요.');
      return;
    }
    
    onSubmit(adminKey);
  };

  const handleClose = () => {
    setAdminKey('');
    setAdminKeyError('');
    onClose();
  };

  return (
    <Modal
      isOpen={isOpen}
      onClose={handleClose}
      title="관리자 접근"
      titleIcon={<ShieldAlert className="h-5 w-5" />}
      accentColor="blue"
    >
      <div className="relative">
        {/* 그라데이션 배경 */}
        <div className="absolute inset-0 bg-gradient-to-r-admin shadow-lg transform -rotate-3 rounded-xl -z-10"></div>
        
        <div className="relative p-6 bg-white rounded-xl shadow-md">
          <div className="mb-4 bg-yellow-50 border-l-4 border-yellow-400 p-3 text-sm text-yellow-700">
            <p className="flex items-center">
              <AlertTriangle className="h-4 w-4 mr-2 flex-shrink-0" />
              <span>관리자 접근은 허가된 사용자만 가능합니다.</span>
            </p>
          </div>
          
          <Input
            id="adminKey"
            name="adminKey"
            type={showPassword ? "text" : "password"}
            label="관리자 키"
            value={adminKey}
            onChange={handleAdminKeyChange}
            onKeyPress={handleKeyPress}
            placeholder="관리자 키를 입력하세요"
            icon={<Lock className="h-4 w-4" />}
            error={adminKeyError}
            rightElement={
              <button
                type="button"
                onClick={handlePasswordVisibility}
                className="text-gray-500 hover:text-gray-700"
              >
                {showPassword ? (
                  <EyeOff className="h-4 w-4" />
                ) : (
                  <Eye className="h-4 w-4" />
                )}
              </button>
            }
          />
          
          <div className="flex justify-end mt-4">
            <Button
              variant="outline"
              onClick={handleClose}
              className="mr-2 border-gray-400 text-gray-700 hover:bg-gray-100"
            >
              취소
            </Button>
            <Button
              variant="primary"
              onClick={handleSubmit}
              className="bg-red-600 hover:bg-red-700"
            >
              접근
            </Button>
          </div>

          <div className="mt-4 pt-4 border-t border-gray-200 text-xs text-gray-500 flex items-center">
            <Key className="h-3 w-3 mr-1" />
            <span>단축키: Ctrl+Shift+F (개발자 및 관리자 전용)</span>
          </div>
        </div>
      </div>
    </Modal>
  );
};

// Key 컴포넌트 정의 (lucide-react에서 가져오지 않았으므로 간단히 정의)
const Key = ({ className }) => {
  return (
    <svg 
      xmlns="http://www.w3.org/2000/svg" 
      width="24" 
      height="24" 
      viewBox="0 0 24 24" 
      fill="none" 
      stroke="currentColor" 
      strokeWidth="2" 
      strokeLinecap="round" 
      strokeLinejoin="round" 
      className={className}
    >
      <path d="M21 2l-2 2m-7.61 7.61a5.5 5.5 0 1 1-7.778 7.778 5.5 5.5 0 0 1 7.777-7.777zm0 0L15.5 7.5m0 0l3 3L22 7l-3-3m-3.5 3.5L19 4" />
    </svg>
  );
};

export default AdminKeyModal;