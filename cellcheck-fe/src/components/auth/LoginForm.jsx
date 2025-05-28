// src/components/auth/LoginForm.jsx
import React, { useState } from 'react';
import { User, Key, Eye, EyeOff, LogIn } from 'lucide-react';
import Input from '../common/Input';
import Button from '../common/Button';

const LoginForm = ({ 
  isAdminMode = false, 
  onAdminModeRequest = () => {}, 
  onSwitchToUserMode = () => {},
  onSubmit = () => {}
}) => {
  const [formData, setFormData] = useState({
    loginId: '',
    password: ''
  });
  const [showPassword, setShowPassword] = useState(false);
  const [rememberMe, setRememberMe] = useState(false);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData({
      ...formData,
      [name]: value
    });
  };

  const handlePasswordVisibility = () => {
    setShowPassword(!showPassword);
  };

  const handleRememberMeChange = () => {
    setRememberMe(!rememberMe);
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    onSubmit({ ...formData, rememberMe }, isAdminMode);
  };

  const getButtonColor = isAdminMode ? 'red' : 'blue';
  const getTextColor = isAdminMode ? 'text-gray-100' : 'text-gray-900';
  const getInputDarkMode = isAdminMode;

  return (
    <form onSubmit={handleSubmit}>
      <Input
        id="loginId"
        name="loginId"
        type="text"
        label="아이디"
        value={formData.loginId}
        onChange={handleChange}
        placeholder={isAdminMode ? "관리자 ID를 입력하세요" : "아이디를 입력하세요"}
        autoComplete="username"
        required
        icon={<User className="h-4 w-4" />}
        darkMode={getInputDarkMode}
      />
      
      <Input
        id="password"
        name="password"
        type={showPassword ? "text" : "password"}
        label="비밀번호"
        value={formData.password}
        onChange={handleChange}
        placeholder={isAdminMode ? "비밀번호를 입력하세요" : "비밀번호를 입력하세요"}
        autoComplete="current-password"
        required
        icon={<Key className="h-4 w-4" />}
        darkMode={getInputDarkMode}
        rightElement={
          <button
            type="button"
            onClick={handlePasswordVisibility}
            className={isAdminMode ? "text-gray-400 hover:text-gray-300" : "text-gray-500 hover:text-gray-700"}
          >
            {showPassword ? (
              <EyeOff className="h-4 w-4" />
            ) : (
              <Eye className="h-4 w-4" />
            )}
          </button>
        }
      />
      
      <div className="flex items-center mb-6">
        <input
          id="remember-me"
          name="remember-me"
          type="checkbox"
          checked={rememberMe}
          onChange={handleRememberMeChange}
          className={`h-4 w-4 rounded ${isAdminMode ? 'text-red-600 focus:ring-red-500 border-gray-700' : 'text-blue-500 focus:ring-blue-500 border-gray-300'}`}
        />
        <label 
          htmlFor="remember-me" 
          className={`ml-2 block text-sm ${isAdminMode ? 'text-gray-300' : 'text-gray-700'}`}
        >
          로그인 정보 저장
        </label>
      </div>
      
      <Button
        type="submit"
        variant="primary"
        fullWidth
        icon={<LogIn className="h-4 w-4" />}
        className={isAdminMode 
          ? "bg-red-600 hover:bg-red-700 focus:ring-red-500" 
          : "bg-blue-600 hover:bg-blue-700 focus:ring-blue-500"
        }
      >
        {isAdminMode ? "관리자 로그인" : "로그인"}
      </Button>

      {isAdminMode && (
        <div className="mt-4 text-center">
          <button 
            onClick={onSwitchToUserMode}
            type="button"
            className="text-sm font-medium text-gray-400 hover:text-gray-300 inline-flex items-center"
          >
            <span className="mr-1">✕</span>
            <span>일반 사용자 모드로 돌아가기</span>
          </button>
        </div>
      )}
    </form>
  );
};

export default LoginForm;