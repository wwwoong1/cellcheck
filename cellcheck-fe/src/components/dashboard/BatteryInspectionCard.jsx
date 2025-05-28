import React from 'react';
import { CheckCircle, XCircle, Eye, Battery } from 'lucide-react';

const BatteryInspectionCard = ({ inspectionData = null, isAdminMode = false, dischargeData = null }) => {
  // 관리자 모드에서는 표시하지 않음
  if (isAdminMode) {
    return null;
  }
  
  // 스타일 설정
  const bgColor = isAdminMode ? 'bg-gray-800' : 'bg-white';
  const textColor = isAdminMode ? 'text-gray-300' : 'text-gray-800';
  const titleColor = isAdminMode ? 'text-gray-200' : 'text-gray-800';
  const borderColor = isAdminMode ? 'border-gray-700' : 'border-gray-200';
  const cardBgColor = isAdminMode ? 'bg-gray-700' : 'bg-gray-50';
  
  // 배터리 타입 매핑
  const batteryTypeMap = {
    'AAA': '초소형 (AAA)',
    'AA': '소형 (AA)',
    'C': '중형 (C)',
    'D': '대형 (D)',
    null: '미분류'
  };
  
  // 검사 결과에 따른 색상과 메시지
  const getInspectionStyle = (isPass) => {
    if (isPass === false) { // 정상 (외관 검사 통과, 방전 대상)
      return {
        bgColor: isAdminMode ? 'bg-green-900 bg-opacity-30' : 'bg-green-50',
        textColor: isAdminMode ? 'text-green-400' : 'text-green-600',
        icon: <CheckCircle className={`h-6 w-6 ${isAdminMode ? 'text-green-400' : 'text-green-500'}`} />,
        message: '정상 (방전 대상)'
      };
    } else { // 불량 (외관 검사 불통과)
      return {
        bgColor: isAdminMode ? 'bg-red-900 bg-opacity-30' : 'bg-red-50',
        textColor: isAdminMode ? 'text-red-400' : 'text-red-600',
        icon: <XCircle className={`h-6 w-6 ${isAdminMode ? 'text-red-400' : 'text-red-500'}`} />,
        message: '외관 불량'
      };
    }
  };
  
  // 배터리 타입 표시 색상
  const getBatteryTypeColor = (type) => {
    switch (type) {
      case 'AAA':
        return isAdminMode ? 'bg-blue-900 text-blue-300' : 'bg-blue-100 text-blue-800';
      case 'AA':
        return isAdminMode ? 'bg-green-900 text-green-300' : 'bg-green-100 text-green-800';
      case 'C':
        return isAdminMode ? 'bg-purple-900 text-purple-300' : 'bg-purple-100 text-purple-800';
      case 'D':
        return isAdminMode ? 'bg-yellow-900 text-yellow-300' : 'bg-yellow-100 text-yellow-800';
      default:
        return isAdminMode ? 'bg-gray-700 text-gray-300' : 'bg-gray-200 text-gray-800';
    }
  };
  
  // 방전 데이터가 있는 경우 검사 카드를 숨기거나 특별한 메시지를 표시
  const isDischargeInProgress = dischargeData && Object.keys(dischargeData).length > 0;
  
  // 검사 결과 스타일
  const inspStyle = inspectionData ? getInspectionStyle(inspectionData.appearanceInspection) : null;
  
  // 진행 중인 프로세스 확인
  const getProcessMessage = () => {
    if (isDischargeInProgress) {
      return "방전 공정이 진행 중입니다. 완료 후 다음 배터리를 넣어주세요.";
    }
    return "검사 대기 중... 배터리를 넣어주세요.";
  };
  
  return (
    <div className={`${bgColor} p-6 rounded-lg shadow-md ${isAdminMode ? '' : 'border'} ${borderColor}`}>
      <div className="flex items-center mb-4">
        <Eye className={`h-5 w-5 ${isAdminMode ? 'text-indigo-400' : 'text-indigo-500'} mr-2`} />
        <h3 className={`text-lg font-medium ${titleColor}`}>검사 공정</h3>
      </div>
      
      {inspectionData ? (
        <div>
          {/* 검사 결과 표시 카드 */}
          <div className={`mb-4 p-4 rounded-lg flex items-center justify-center ${inspStyle.bgColor}`}>
            <div className="mr-3 flex-shrink-0">
              {inspStyle.icon}
            </div>
            <div className="text-center">
              <h4 className={`text-lg font-medium ${inspStyle.textColor}`}>
                {inspectionData.appearanceInspection ? '외관 불량' : '정상'}
              </h4>
              <p className={`text-sm ${inspStyle.textColor}`}>
                {inspectionData.appearanceInspection ? 
                  '외관 검사 불통과 - 불량통으로 이동' : 
                  '외관 검사 통과 - 방전 공정 진행'}
              </p>
            </div>
          </div>
          
          {/* 배터리 정보 */}
          <div className={`${cardBgColor} p-4 rounded-lg`}>
            <div className="flex justify-between items-center mb-2">
              <span className={textColor}>배터리 타입:</span>
              <span className={`px-2 py-1 rounded-md text-sm font-medium ${getBatteryTypeColor(inspectionData.batteryType)}`}>
                {batteryTypeMap[inspectionData.batteryType]}
              </span>
            </div>
            <div className="flex justify-between items-center">
              <span className={textColor}>처리 방향:</span>
              <span className={`font-medium ${inspStyle.textColor}`}>
                {inspectionData.appearanceInspection ? '외관불량통' : '방전 공정'}
              </span>
            </div>
          </div>
        </div>
      ) : (
        <div className={`${cardBgColor} p-6 rounded-lg text-center`}>
          <p className={isAdminMode ? "text-gray-400" : "text-gray-500"}>
            {getProcessMessage()}
          </p>
        </div>
      )}
    </div>
  );
};

export default BatteryInspectionCard; 