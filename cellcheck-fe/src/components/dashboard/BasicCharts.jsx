import React from 'react';
import { Card, Title, AreaChart, BarChart, DonutChart } from '@tremor/react';

// 테마 설정
const darkThemeColors = {
  background: '#1f2937', // 다크 모드 배경색
  text: '#f3f4f6',       // 다크 모드 텍스트
  primary: '#3b82f6',    // 다크 모드 주 색상
  secondary: '#60a5fa',  // 다크 모드 보조 색상
  accent: '#f59e0b',     // 다크 모드 강조 색상
  success: '#10b981',    // 다크 모드 성공 색상
  warning: '#f59e0b',    // 다크 모드 경고 색상
  danger: '#ef4444',     // 다크 모드 위험 색상
  info: '#6366f1',       // 다크 모드 정보 색상
};

const lightThemeColors = {
  background: '#ffffff', // 라이트 모드 배경색
  text: '#1f2937',       // 라이트 모드 텍스트
  primary: '#3b82f6',    // 라이트 모드 주 색상
  secondary: '#60a5fa',  // 라이트 모드 보조 색상
  accent: '#f59e0b',     // 라이트 모드 강조 색상
  success: '#10b981',    // 라이트 모드 성공 색상
  warning: '#f59e0b',    // 라이트 모드 경고 색상
  danger: '#ef4444',     // 라이트 모드 위험 색상
  info: '#6366f1',       // 라이트 모드 정보 색상
};

// 차트 색상 배열 설정
const chartColors = (isAdminMode) => {
  const colors = isAdminMode ? darkThemeColors : lightThemeColors;
  return [
    colors.primary,
    colors.secondary,
    colors.accent,
    colors.success,
    colors.warning,
    colors.danger,
    colors.info
  ];
};

// 간단한 바 차트 컴포넌트
export const BarChart2 = ({ data = [], isAdminMode = false }) => {
  const colors = chartColors(isAdminMode);
  const chartProps = {
    data,
    index: 'label',
    categories: ['value'],
    colors: [colors[0]],
    showLegend: false,
    valueFormatter: (value) => `${value}${data[0]?.unit || ''}`,
    className: 'h-72 mt-4'
  };

  const cardProps = {
    className: `p-4 rounded-xl shadow-md ${isAdminMode ? 'bg-gray-800 text-gray-300' : 'bg-white text-gray-800'}`
  };

  return (
    <Card {...cardProps}>
      <Title className={`text-lg font-medium ${isAdminMode ? 'text-gray-200' : 'text-gray-800'}`}>
        {data.title || "데이터 분포"}
      </Title>
      <BarChart {...chartProps} />
    </Card>
  );
};

// 다중 바 차트 컴포넌트
export const MultiBarChart = ({ data = [], isAdminMode = false }) => {
  if (!data.length) return null;

  const colors = chartColors(isAdminMode);
  // 모든 데이터 항목의 고유 카테고리들 추출
  const allCategories = [...new Set(data.flatMap(item => 
    Object.keys(item).filter(key => key !== 'label' && key !== 'title')
  ))];

  const chartProps = {
    data: data,
    index: 'label',
    categories: allCategories,
    colors: colors.slice(0, allCategories.length),
    valueFormatter: (value) => `${value}`,
    className: 'h-72 mt-4'
  };

  const cardProps = {
    className: `p-4 rounded-xl shadow-md ${isAdminMode ? 'bg-gray-800 text-gray-300' : 'bg-white text-gray-800'}`
  };

  return (
    <Card {...cardProps}>
      <Title className={`text-lg font-medium ${isAdminMode ? 'text-gray-200' : 'text-gray-800'}`}>
        {data[0]?.title || "다중 데이터 분포"}
      </Title>
      <BarChart {...chartProps} />
    </Card>
  );
};

// 간단한 도넛 차트 컴포넌트
export const PieChart = ({ data = [], isAdminMode = false }) => {
  const colors = chartColors(isAdminMode);
  // Tremor의 DonutChart에 맞게 데이터 변환
  const chartData = data.map(item => ({
    name: item.label,
    value: item.value
  }));

  const chartProps = {
    data: chartData,
    category: 'value',
    index: 'name',
    colors: colors.slice(0, chartData.length),
    variant: 'pie',
    valueFormatter: (value) => `${value}${data[0]?.unit || ''}`,
    className: 'h-72 mt-4'
  };

  const cardProps = {
    className: `p-4 rounded-xl shadow-md ${isAdminMode ? 'bg-gray-800 text-gray-300' : 'bg-white text-gray-800'}`
  };

  return (
    <Card {...cardProps}>
      <Title className={`text-lg font-medium ${isAdminMode ? 'text-gray-200' : 'text-gray-800'}`}>
        {data.title || "데이터 분포"}
      </Title>
      <DonutChart {...chartProps} />
    </Card>
  );
};

// 도넛 차트 컴포넌트
export const DonutChart2 = ({ data = [], isAdminMode = false }) => {
  const colors = chartColors(isAdminMode);
  // Tremor의 DonutChart에 맞게 데이터 변환
  const chartData = data.map(item => ({
    name: item.label,
    value: item.value
  }));

  const chartProps = {
    data: chartData,
    category: 'value',
    index: 'name',
    colors: colors.slice(0, chartData.length),
    valueFormatter: (value) => `${value}${data[0]?.unit || ''}`,
    className: 'h-72 mt-4'
  };

  const cardProps = {
    className: `p-4 rounded-xl shadow-md ${isAdminMode ? 'bg-gray-800 text-gray-300' : 'bg-white text-gray-800'}`
  };

  return (
    <Card {...cardProps}>
      <Title className={`text-lg font-medium ${isAdminMode ? 'text-gray-200' : 'text-gray-800'}`}>
        {data.title || "데이터 분포"}
      </Title>
      <DonutChart {...chartProps} />
    </Card>
  );
};

// 영역 차트 컴포넌트
export const AreaChart2 = ({ data = [], isAdminMode = false }) => {
  const colors = chartColors(isAdminMode);
  
  // 모든 데이터 항목의 고유 카테고리들 추출
  const allCategories = [...new Set(data.flatMap(item => 
    Object.keys(item).filter(key => key !== 'label' && key !== 'title')
  ))];

  const chartProps = {
    data: data,
    index: 'label',
    categories: allCategories,
    colors: colors.slice(0, allCategories.length),
    valueFormatter: (value) => `${value}`,
    showLegend: true,
    showGridLines: false,
    autoMinValue: true,
    className: 'h-72 mt-4'
  };

  const cardProps = {
    className: `p-4 rounded-xl shadow-md ${isAdminMode ? 'bg-gray-800 text-gray-300' : 'bg-white text-gray-800'}`
  };

  return (
    <Card {...cardProps}>
      <Title className={`text-lg font-medium ${isAdminMode ? 'text-gray-200' : 'text-gray-800'}`}>
        {data[0]?.title || "시계열 데이터"}
      </Title>
      <AreaChart {...chartProps} />
    </Card>
  );
};

export default { BarChart2, MultiBarChart, PieChart, DonutChart2, AreaChart2 }; 