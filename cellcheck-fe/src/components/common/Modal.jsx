// src/components/common/Modal.jsx
import React from 'react';
import { X } from 'lucide-react';
import Button from './Button';

const Modal = ({
  isOpen,
  onClose,
  title,
  titleIcon,
  children,
  footer,
  accentColor = 'blue',
  headerElement,
  footerElement,
  width = 'max-w-md',
  darkMode = false
}) => {
  if (!isOpen) return null;

  const bgColor = darkMode ? 'bg-gray-800' : 'bg-white';
  const textColor = darkMode ? 'text-white' : 'text-gray-900';
  const borderColor = darkMode ? 'border-gray-700' : 'border-gray-200';

  const headerBgColor = 
    accentColor === 'blue' ? 'bg-blue-600' : 
    accentColor === 'red' ? 'bg-red-600' : 
    'bg-gray-600';

  return (
    <div className="fixed inset-0 flex items-center justify-center z-50">
      <div 
        className="absolute inset-0 bg-black bg-opacity-50 backdrop-blur-sm"
        onClick={onClose}
      ></div>
      
      <div className={`${bgColor} rounded-lg shadow-xl w-full ${width} z-10 overflow-hidden`}>
        {/* 헤더 */}
        {(title || headerElement) && (
          <div className={`${headerBgColor} px-4 py-3 flex items-center justify-between`}>
            {headerElement || (
              <div className="flex items-center text-white">
                {titleIcon && <span className="mr-2">{titleIcon}</span>}
                <h3 className="text-lg font-medium">{title}</h3>
              </div>
            )}
            <button 
              onClick={onClose}
              className="text-white hover:text-gray-200"
            >
              <X className="h-5 w-5" />
            </button>
          </div>
        )}
        
        {/* 본문 */}
        <div className="p-6">
          {children}
        </div>
        
        {/* 푸터 */}
        {(footer || footerElement) && (
          <div className={`${darkMode ? 'bg-gray-700' : 'bg-gray-50'} px-4 py-3 ${borderColor} border-t`}>
            {footerElement || (
              <div className="flex justify-end">
                <Button 
                  variant="outline" 
                  onClick={onClose} 
                  className="mr-2"
                >
                  취소
                </Button>
                {footer}
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

export default Modal;