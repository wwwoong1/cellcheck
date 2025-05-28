// src/components/common/Input.jsx
import React from 'react';

const Input = ({
  id,
  name,
  type = 'text',
  label,
  value,
  onChange,
  onKeyPress,
  placeholder = '',
  icon,
  error = '',
  required = false,
  autoComplete = '',
  className = '',
  rightElement,
  darkMode = false
}) => {
  const inputClasses = [
    'block w-full rounded-md text-sm focus:outline-none',
    darkMode 
      ? 'border-gray-600 bg-gray-700 text-white focus:ring-red-500 focus:border-red-500' 
      : 'border-gray-300 focus:ring-blue-500 focus:border-blue-500',
    error ? 'border-red-300' : 'border',
    className
  ].join(' ');

  const labelClasses = [
    'block text-sm font-medium mb-1 flex items-center',
    darkMode ? 'text-gray-300' : 'text-gray-700'
  ].join(' ');

  const errorClasses = 'mt-1 text-sm text-red-600';

  return (
    <div className="mb-4">
      {label && (
        <label htmlFor={id} className={labelClasses}>
          {icon && <span className="mr-1">{icon}</span>}
          {label}
          {required && <span className="text-red-500 ml-1">*</span>}
        </label>
      )}
      <div className="relative">
        <input
          id={id}
          name={name}
          type={type}
          value={value}
          onChange={onChange}
          onKeyPress={onKeyPress}
          className={`px-3 py-2 ${inputClasses}`}
          placeholder={placeholder}
          autoComplete={autoComplete}
          required={required}
        />
        {rightElement && (
          <div className="absolute right-2 top-1/2 transform -translate-y-1/2">
            {rightElement}
          </div>
        )}
      </div>
      {error && <p className={errorClasses}>{error}</p>}
    </div>
  );
};

export default Input;