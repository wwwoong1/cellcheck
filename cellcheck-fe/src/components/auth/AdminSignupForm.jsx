import React, { useState } from 'react';
import { User, Key, Eye, EyeOff, Mail, Check, X } from 'lucide-react';
import Input from '../common/Input';
import Button from '../common/Button';
import { registerAdmin } from '../../api/authApi';

const AdminSignupForm = ({ onSuccess, onCancel }) => {
  const [formData, setFormData] = useState({
    loginId: '',
    password: '',
    passwordConfirm: '',
    adminName: ''
  });
  
  const [showPassword, setShowPassword] = useState(false);
  const [showPasswordConfirm, setShowPasswordConfirm] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [validationErrors, setValidationErrors] = useState({});

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData({
      ...formData,
      [name]: value
    });
    
    // 입력 필드 변경 시 유효성 검사 오류 초기화
    if (validationErrors[name]) {
      setValidationErrors({
        ...validationErrors,
        [name]: null
      });
    }
  };

  const validateForm = () => {
    const errors = {};
    
    // ID 유효성 검사
    if (!formData.loginId.trim()) {
      errors.loginId = '아이디를 입력해주세요';
    } else if (formData.loginId.length < 5) {
      errors.loginId = '아이디는 최소 5자 이상이어야 합니다';
    }
    
    // 비밀번호 유효성 검사
    if (!formData.password) {
      errors.password = '비밀번호를 입력해주세요';
    } else if (formData.password.length < 8) {
      errors.password = '비밀번호는 최소 8자 이상이어야 합니다';
    }
    
    // 비밀번호 확인 유효성 검사
    if (formData.password !== formData.passwordConfirm) {
      errors.passwordConfirm = '비밀번호가 일치하지 않습니다';
    }
    
    // 이름 유효성 검사
    if (!formData.adminName.trim()) {
      errors.adminName = '이름을 입력해주세요';
    }
    
    setValidationErrors(errors);
    return Object.keys(errors).length === 0;
  };

  const handlePasswordVisibility = () => {
    setShowPassword(!showPassword);
  };

  const handlePasswordConfirmVisibility = () => {
    setShowPasswordConfirm(!showPasswordConfirm);
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    
    // 폼 유효성 검사
    if (!validateForm()) {
      return;
    }
    
    setLoading(true);
    setError(null);
    
    try {
      // API 호출
      const response = await registerAdmin(
        formData.loginId,
        formData.password,
        formData.adminName
      );
      
      if (response.success) {
        // 성공 처리
        onSuccess && onSuccess();
      } else {
        // 실패 처리
        setError(response.message || '회원가입에 실패했습니다. 다시 시도해주세요.');
      }
    } catch (error) {
      setError('회원가입 중 오류가 발생했습니다. 다시 시도해주세요.');
      console.error('관리자 등록 오류:', error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="space-y-4">
      <h2 className="text-2xl font-semibold text-gray-300 mb-6 text-center">관리자 계정 등록</h2>
      
      {error && (
        <div className="bg-red-100 border-l-4 border-red-500 text-red-700 p-4 mb-4 rounded">
          <div className="flex items-center">
            <X className="h-5 w-5 mr-2 text-red-500" />
            <p>{error}</p>
          </div>
        </div>
      )}
      
      <Input
        id="loginId"
        name="loginId"
        type="text"
        label="아이디"
        value={formData.loginId}
        onChange={handleChange}
        placeholder="아이디를 입력하세요"
        autoComplete="username"
        required
        icon={<User className="h-4 w-4" />}
        darkMode={true}
        error={validationErrors.loginId}
      />
      
      <Input
        id="adminName"
        name="adminName"
        type="text"
        label="이름"
        value={formData.adminName}
        onChange={handleChange}
        placeholder="이름을 입력하세요"
        autoComplete="name"
        required
        icon={<User className="h-4 w-4" />}
        darkMode={true}
        error={validationErrors.adminName}
      />
      
      <Input
        id="password"
        name="password"
        type={showPassword ? "text" : "password"}
        label="비밀번호"
        value={formData.password}
        onChange={handleChange}
        placeholder="비밀번호를 입력하세요"
        autoComplete="new-password"
        required
        icon={<Key className="h-4 w-4" />}
        darkMode={true}
        error={validationErrors.password}
        rightElement={
          <button
            type="button"
            onClick={handlePasswordVisibility}
            className="text-gray-400 hover:text-gray-300"
          >
            {showPassword ? (
              <EyeOff className="h-4 w-4" />
            ) : (
              <Eye className="h-4 w-4" />
            )}
          </button>
        }
      />
      
      <Input
        id="passwordConfirm"
        name="passwordConfirm"
        type={showPasswordConfirm ? "text" : "password"}
        label="비밀번호 확인"
        value={formData.passwordConfirm}
        onChange={handleChange}
        placeholder="비밀번호를 다시 입력하세요"
        autoComplete="new-password"
        required
        icon={<Key className="h-4 w-4" />}
        darkMode={true}
        error={validationErrors.passwordConfirm}
        rightElement={
          <button
            type="button"
            onClick={handlePasswordConfirmVisibility}
            className="text-gray-400 hover:text-gray-300"
          >
            {showPasswordConfirm ? (
              <EyeOff className="h-4 w-4" />
            ) : (
              <Eye className="h-4 w-4" />
            )}
          </button>
        }
      />
      
      <div className="flex space-x-3 pt-2">
        <Button
          type="button"
          variant="outline"
          fullWidth
          className="border-gray-500 text-gray-800 bg-white hover:bg-gray-200 hover:text-gray-900 border rounded-md shadow-sm transition-colors"
          onClick={onCancel}
          disabled={loading}
        >
          취소
        </Button>
        
        <Button
          type="submit"
          variant="primary"
          fullWidth
          disabled={loading}
          className="bg-red-600 hover:bg-red-700 focus:ring-red-500"
          icon={loading ? null : <Check className="h-4 w-4" />}
        >
          {loading ? '처리 중...' : '관리자 등록'}
        </Button>
      </div>
    </form>
  );
};

export default AdminSignupForm; 