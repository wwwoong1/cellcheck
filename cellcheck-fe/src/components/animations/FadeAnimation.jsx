// src/components/animations/CircleAnimation.jsx
import React, { useEffect, useState } from 'react';

const FadeAnimation = ({ show, isEntering = true }) => {
  const [animationClass, setAnimationClass] = useState('');
  
  useEffect(() => {
    if (show) {
      setAnimationClass(isEntering ? 'animate-fade-in' : 'animate-fade-out');
    } else {
      setAnimationClass('');
    }
  }, [show, isEntering]);
  
  if (!show && !animationClass) return null;
  
  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center pointer-events-none overflow-hidden">
      <div className={`absolute inset-0 transition-opacity duration-500 ease-in-out ${animationClass}`}>
        <div className={`absolute inset-0 ${
          isEntering 
            ? 'bg-gradient-to-br from-blue-400/80 via-blue-500/70 to-indigo-600/60' 
            : 'bg-gradient-to-br from-red-500/80 via-red-600/70 to-purple-600/60'
        }`}></div>
        
        <div className={`absolute inset-0 ${
          isEntering ? 'bg-pattern opacity-20' : 'bg-pattern-admin opacity-20'
        }`}></div>
        
        {/* 블러 효과 */}
        <div className="absolute inset-0 backdrop-blur-sm"></div>
        
        {/* 중앙 아이콘 */}
        <div className={`absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 
          ${isEntering ? 'text-blue-100' : 'text-red-100'} 
          transition-all duration-500 ease-in-out 
          ${animationClass === 'animate-fade-in' ? 'scale-100' : 'scale-50'}`}>
          <svg xmlns="http://www.w3.org/2000/svg" className="h-24 w-24" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            {isEntering ? (
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M13 10V3L4 14h7v7l9-11h-7z" />
            ) : (
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M8 11V7a4 4 0 118 0m-4 8v-3m-6 3h12a2 2 0 002-2v-6a2 2 0 00-2-2H6a2 2 0 00-2 2v6a2 2 0 002 2z" />
            )}
          </svg>
        </div>
      </div>
    </div>
  );
};

export default FadeAnimation;