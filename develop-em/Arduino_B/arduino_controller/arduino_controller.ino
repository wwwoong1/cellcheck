// 아두이노 컨트롤러 코드 (Arduino B) - 수정본

// 핀 정의 (그대로 유지)
#define DIR_PIN 45     // 스테퍼 모터 방향 핀
#define STEP_PIN 47    // 스테퍼 모터 스텝 핀
#define ENABLE_PIN 49  // 스테퍼 모터 활성화 핀
#define CONVEYOR_LED_PIN 5  // 컨베이어 벨트 LED (테스트용)
#define BUTTON_PIN 6        // 시스템 시작/정지 버튼
#define RELAY1_PIN 14  // 리니어 액추에이터 연장용 릴레이
#define RELAY2_PIN 15  // 리니어 액추에이터 수축용 릴레이
#define IR_SENSOR_A_PIN 2  // 적외선 센서A
#define IR_SENSOR_B_PIN 3  // 적외선 센서B
#define BARRIER_A_PIN 8    // 차단판A 서보
#define BARRIER_B_PIN 9    // 차단판B 서보

#include <Servo.h>
#include <AccelStepper.h>

// 서보모터 객체
Servo barrierServoA;
Servo barrierServoB;

//추가사항 시작/////////////////////////////////////////////////////////////////
// 로봇팔 서보모터 객체 생성
Servo servos[4];

// 서보모터 핀 정의
const int SERVO_PINS[4] = {10, 11, 12, 13};

// 현재 펄스 및 목표 펄스 (마이크로초)
float startPulses[4] = {2136, 2400, 1472, 544}; // 초기위치(150도, 180도, 90도, 0도에 해당)
float currentPulses[4] = {2136, 2400, 1472, 544}; // 현재 위치
float targetPulses[4]  = {2136, 2400, 1472, 544}; // 목표 위치

// 이동 설정
const int updateInterval = 40;  // 펄스 업데이트 간격 (ms)
const float pulseStep = 10.0;    // 한 번에 변경할 펄스 크기

// 로봇 팔 초기화 관련 변수
bool isInitializing = false;
unsigned long lastUpdateTime = 0;
int currentServoIndex = 0;

// 각도를 펄스로 변환하는 함수
int angleToPulse(float angle) {
  // 0도 = 544us, 180도 = 2400us로 매핑
  return map(angle, 0, 180, 544, 2400);
}

// 펄스를 각도로 변환하는 함수
float pulseToAngle(int pulse) {
  return map(pulse, 544, 2400, 0, 180);
}
//추가사항 종료/////////////////////////////////////////////////////////////////

// 스테퍼 모터 객체 생성
AccelStepper conveyor(1, STEP_PIN, DIR_PIN);  // 1은 STEP+DIR 인터페이스

// 컨베이어 상태
bool conveyorEnabled = false;

// 버튼 디바운싱 변수
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int lastButtonState = HIGH;
int buttonState = HIGH;

// 시스템 상태
bool systemRunning = false;  // 초기 상태 IDLE
bool conveyorRunning = false;

// 적외선 센서 상태
bool lastIrAState = false;
bool lastIrBState = false;

// 서보모터 제어 타이머 변수
unsigned long barrierATimer = 0;
unsigned long barrierBTimer = 0;
int barrierAState = 0;  // 0: 닫힘, 1: 열림, 2: 대기 중, 3: 닫는 중, 4: 열리는 중
int barrierBState = 0;  // 0: 닫힘, 1: 열림, 2: 대기 중, 3: 닫는 중, 4: 열리는 중

// 액추에이터 사이클 관련 변수
unsigned long actuatorExtendTime = 0;
bool actuatorCycleActive = false;
int actuatorCycleState = 0; // 0: 대기, 1: 확장 중, 2: 확장 완료, 3: 수축 중, 4: 완료

int barrierAPosition = 0;
int barrierBPosition = 0;

void setup() {
  // 시리얼 통신 시작
  Serial.begin(9600);
  
  // 핀 모드 설정
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR_SENSOR_A_PIN, INPUT);
  pinMode(IR_SENSOR_B_PIN, INPUT);
  pinMode(CONVEYOR_LED_PIN, OUTPUT);
  digitalWrite(CONVEYOR_LED_PIN, LOW);  // 명시적으로 LED 끄기
  
  // 서보모터 초기화
  barrierServoA.attach(BARRIER_A_PIN);
  barrierServoB.attach(BARRIER_B_PIN);
  
  // 차단판 초기 위치 (0도 = 차단)
  barrierServoA.write(0);
  barrierServoB.write(0);

  //추가사항 시작/////////////////////////////////////////////////////////////////
  // 로봇팔 서보모터 초기화
  for (int i = 0; i < 4; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].writeMicroseconds(currentPulses[i]);
  }
  //추가사항 종료/////////////////////////////////////////////////////////////////
  
  // 리니어 액추에이터 릴레이 핀 설정
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  
  // 릴레이 초기 상태 (정지)
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  
  // 스테퍼 모터 설정
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  // 초기에는 비활성화
  
  conveyor.setMaxSpeed(1000);
  conveyor.setSpeed(400);
  
  Serial.println("Arduino Controller initialized - System IDLE");
}

void loop() {
  // 버튼 디바운싱 처리 (항상 활성화)
  handleButton();
  
  // 적외선 센서 상태 체크 (RUNNING 상태일 때만)
  if (systemRunning) {
    checkIrSensors();
  }
  
  // 시리얼 명령 처리 (일부 명령은 항상 활성화)
  handleSerialCommands();
  
  // 서보모터 상태 업데이트 (비차단식 동작을 위해)
  updateBarrierA();
  updateBarrierB();

  //추가사항 시작/////////////////////////////////////////////////////////////////
  // 로봇팔 초기화 업데이트 추가
  updateRobotArmInit();
  //추가사항 종료/////////////////////////////////////////////////////////////////

  // 액추에이터 사이클 업데이트 추가
  updateActuatorCycle();
  
  // 컨베이어 벨트 구동 (활성화된 경우에만)
  if (systemRunning && conveyorEnabled) {
    conveyor.runSpeed();
  }
  
  // 컨베이어 벨트 상태 업데이트 (LED로 시각화)
  digitalWrite(CONVEYOR_LED_PIN, conveyorRunning ? HIGH : LOW);
  
  // 상태 디버깅 (5초마다)
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 5000) {
    lastDebugTime = millis();
    Serial.print("System State: ");
    Serial.print(systemRunning ? "RUNNING" : "IDLE");
    Serial.print(", Conveyor: ");
    Serial.print(conveyorRunning ? "ON" : "OFF");
    Serial.print(", Barrier A: ");
    Serial.print(barrierAState);
    Serial.print(", Barrier B: ");
    Serial.println(barrierBState);
  }
  
}

void handleButton() {
  // 버튼 상태 읽기 (풀업 저항으로 누르면 LOW)
  int reading = digitalRead(BUTTON_PIN);
  
  // 디바운싱
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      
      // 버튼이 눌렸을 때 (LOW)
      if (buttonState == LOW) {
        systemRunning = !systemRunning;
        
        // 버튼 이벤트 전송
        if (systemRunning) {
          Serial.println("BUTTON:START");
          conveyorRunning = true;
          startConveyor();
        } else {
          Serial.println("BUTTON:STOP");
          conveyorRunning = false;
          stopConveyor();
        }
      }
    }
  }
  
  lastButtonState = reading;
}

void checkIrSensors() {
  // 시스템이 RUNNING 상태일 때만 실행 (이미 loop에서 확인)
  
  // 적외선 센서A 상태 확인
  bool irAState = digitalRead(IR_SENSOR_A_PIN) == LOW; // LOW = 감지됨
  if (irAState != lastIrAState) {
    Serial.print("IR_A:");
    Serial.println(irAState ? "1" : "0");
    lastIrAState = irAState;
  }
  
  // 적외선 센서B 상태 확인
  bool irBState = digitalRead(IR_SENSOR_B_PIN) == LOW; // LOW = 감지됨
  if (irBState != lastIrBState) {
    Serial.print("IR_B:");
    Serial.println(irBState ? "1" : "0");
    lastIrBState = irBState;
  }
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    Serial.print("Received command: ");
    Serial.println(command);
    
    //추가사항 시작/////////////////////////////////////////////////////////////////
    // 로봇팔 각도 제어 명령
    if (command.startsWith("ROBOT_ARM:")) {
      String action = command.substring(10);
      
      // 초기 위치 복귀
      if (action == "INIT") {
        initRobotArm();
      }
      // 명령 위치 조작
      else {
        moveRobotArm(action);
      }
    }
    //추가사항 종료/////////////////////////////////////////////////////////////////

    // 액추에이터 제어 명령
    if (command.startsWith("ACTUATOR:")) {
      String action = command.substring(9);
      
      if (action == "CYCLE") {
        startActuatorCycle();
      }
      // 개별 명령은 디버깅이나 테스트용으로 남겨둘 수 있음
      else if (action == "EXTEND") {
        extendActuator();
      } 
      else if (action == "RETRACT") {
        retractActuator();
      }
      else if (action == "STOP") {
        stopActuator();
      }
    }
    
// 컨베이어 벨트 제어
    else if (command.startsWith("CONVEYOR:")) {
      int value = command.substring(9).toInt();
      
      if (value == 1) {
        startConveyor();
        Serial.println("Conveyor started");
      } else {
        stopConveyor();
        Serial.println("Conveyor stopped");
      }
    }
    
    
    // START/STOP 명령은 항상 처리 (시스템 전체 제어)
    if (command == "START") {
      systemRunning = true;
      conveyorRunning = true;
      Serial.println("SYSTEM:RUNNING");
    } 
    else if (command == "STOP") {
      systemRunning = false;
      conveyorRunning = false;
      Serial.println("SYSTEM:IDLE");
    }
    // 아래 명령들은 시스템이 RUNNING 상태일 때만 처리
    else if (systemRunning) {
      // 차단판A 제어
      if (command.startsWith("BARRIER_A:")) {
        int value = command.substring(10).toInt();
        if (value == 1) {
          startOpenBarrierA();
        } else {
          closeBarrierA();
        }
      }
      // 차단판B 제어
      else if (command.startsWith("BARRIER_B:")) {
        int value = command.substring(10).toInt();
        if (value == 1) {
          startOpenBarrierB();
        } else {
          closeBarrierB();
        }
      }
      // 컨베이어 벨트 제어
      else if (command.startsWith("CONVEYOR:")) {
        int value = command.substring(9).toInt();
        conveyorRunning = (value == 1);
        Serial.print("Conveyor set to: ");
        Serial.println(conveyorRunning ? "ON" : "OFF");
      }
    }
    else {
      Serial.println("Command ignored - System is IDLE");
    }
  }
}

// 비차단식 서보모터 제어 함수들
void startOpenBarrierA() {
  if (barrierAState == 0) {  // 닫힌 상태에서만 시작
    barrierAState = 1;  // 열림 상태로 설정
    barrierServoA.write(90);  // 서보모터 열기
    barrierATimer = millis();  // 타이머 시작
    Serial.println("Barrier A: Opening");
  }
}

void updateBarrierA() {
  // 열린 상태에서 1초 후 닫기
  if (barrierAState == 1 && millis() - barrierATimer >= 2000) {
    barrierAState = 3;  // 닫는 중 상태로 변경
    barrierServoA.write(0);  // 서보모터 닫기
    Serial.println("Barrier A: Closing");
    barrierATimer = millis();  // 타이머 재설정
  }
  
  // 닫는 중 상태에서 500ms 후 완료 상태로 변경
  if (barrierAState == 3 && millis() - barrierATimer >= 500) {
    barrierAState = 0;  // 닫힘 상태로 변경
    Serial.println("Barrier A: Closed");
    Serial.println("BARRIER_A:CLOSED");
  }
}


void closeBarrierA() {
  barrierServoA.write(0);
  barrierAState = 0;
  Serial.println("Barrier A: Force Closed");
}

void startOpenBarrierB() {
  if (barrierBState == 0) {  // 닫힌 상태에서만 시작
    barrierBState = 1;  // 열림 상태로 설정
    barrierServoB.write(90);  // 서보모터 열기
    barrierBTimer = millis();  // 타이머 시작
    Serial.println("Barrier B: Opening");
  }
}

void updateBarrierB() {
  // 열린 상태에서 1초 후 닫기
  if (barrierBState == 1 && millis() - barrierBTimer >= 2000) {
    barrierBState = 3;  // 닫는 중 상태로 변경
    barrierServoB.write(0);  // 서보모터 닫기
    Serial.println("Barrier B: Closing");
    barrierBTimer = millis();  // 타이머 재설정
  }

  // 닫는 중 상태에서 500ms 후 완료 상태로 변경
  if (barrierBState == 3 && millis() - barrierBTimer >= 500) {
    barrierBState = 0;  // 닫힘 상태로 변경
    Serial.println("Barrier B: Closed");
    Serial.println("BARRIER_B:CLOSED");
  }
}


void closeBarrierB() {
  barrierServoB.write(0);
  barrierBState = 0;
  Serial.println("Barrier B: Force Closed");
}

// 리니어 액추에이터 제어 함수
void extendActuator() {
  digitalWrite(RELAY1_PIN, HIGH);  // 릴레이1 ON
  digitalWrite(RELAY2_PIN, LOW); // 릴레이2 OFF
}

void retractActuator() {
  digitalWrite(RELAY1_PIN, LOW); // 릴레이1 OFF
  digitalWrite(RELAY2_PIN, HIGH);  // 릴레이2 ON
}

void stopActuator() {
  digitalWrite(RELAY1_PIN, HIGH); // 둘 다 OFF
  digitalWrite(RELAY2_PIN, HIGH);
}

// 컨베이어 벨트 제어 함수
void startConveyor() {
  digitalWrite(ENABLE_PIN, LOW);  // 스테퍼 모터 드라이버 활성화
  conveyor.setSpeed(400);
  conveyorEnabled = true;
}

void stopConveyor() {
  digitalWrite(ENABLE_PIN, HIGH); // 스테퍼 모터 드라이버 비활성화
  conveyorEnabled = false;
}


// 액추에이터 사이클 업데이트 함수 수정
// 액추에이터 사이클 업데이트 함수 수정
void updateActuatorCycle() {
  if (!actuatorCycleActive) return;
  
  unsigned long currentTime = millis();
  
  switch (actuatorCycleState) {
    case 1: // 확장 중 (밀어내는 중)
      if (currentTime - actuatorExtendTime >= 5000) { // 확장 시간 5초
        stopActuator(); // 확장 완료 후 정지
        actuatorCycleState = 2;
        actuatorExtendTime = currentTime;
        Serial.println("Actuator fully extended and stopped");
      }
      break;
      
    case 2: // 확장 후 대기 중 (불량품이 떨어질 시간)
      if (currentTime - actuatorExtendTime >= 1000) { // 1초 대기
        Serial.println("ACTUATOR:COMPLETE"); // ROS에 완료 신호 보냄
        retractActuator(); // 수축 시작
        actuatorCycleState = 3;
        actuatorExtendTime = currentTime;
        Serial.println("Waiting complete, actuator retracting");
      }
      break;
      
    case 3: // 수축 중 (들어가는 중)
      if (currentTime - actuatorExtendTime >= 5000) { // 수축 시간 5초
        stopActuator();
        actuatorCycleState = 4;
        actuatorExtendTime = currentTime;
        Serial.println("Actuator fully retracted and stopped");

      }
      break;

    case 4: // 완료 상태 - 완료 메시지 즉시 전송
      actuatorCycleActive = false;  // 사이클 종료
      actuatorCycleState = 0;  // 대기 상태로 복귀
      break;
  }
}

// 액추에이터 사이클 시작 함수 수정
void startActuatorCycle() {
  if (!actuatorCycleActive) {
    stopActuator();     // 둘 다 끄고
    delay(100);

    // 현재 위치를 '수축 완료'로 가정하고 확장부터 시작
    actuatorCycleActive = true;
    actuatorCycleState = 1;             // 확장 시작
    actuatorExtendTime = millis();
    extendActuator();
    Serial.println("Actuator cycle started - extending");
  }
}

//추가사항 시작/////////////////////////////////////////////////////////////////
// 로봇팔 초기 위치 저장 함수
void initRobotArm() {
  // 초기화가 이미 진행 중이면 리턴
  if (isInitializing) return;
  
  // 로봇팔 초기 위치 저장
  for (int i = 0; i < 4; i++) {
    targetPulses[i] = startPulses[i];
  }
  
  // 초기화 시작
  isInitializing = true;
  currentServoIndex = 0;
  lastUpdateTime = millis();
}

// 로봇팔 초기화 업데이트 함수 (loop()에서 호출)
void updateRobotArmInit() {
  if (!isInitializing) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime < updateInterval) return;
  
  lastUpdateTime = currentTime;
  
  // 현재 서보의 목표 위치와 현재 위치 차이 확인
  float diff = targetPulses[3-currentServoIndex] - currentPulses[3-currentServoIndex];
  
  if (abs(diff) > 10) {
    // 서보 이동
    if (diff > 0) {
      currentPulses[3-currentServoIndex] += pulseStep;
    } else {
      currentPulses[3-currentServoIndex] -= pulseStep;
    }
    servos[3-currentServoIndex].writeMicroseconds(currentPulses[3-currentServoIndex]);
  } else {
    // 현재 서보 이동 완료, 다음 서보로
    currentServoIndex++;
    if (currentServoIndex >= 4) {
      // 모든 서보 초기화 완료
      isInitializing = false;
      Serial.println("ROBOT_ARM:INIT_COMPLETE");
    }
  }
}

// 로봇팔 목표 위치 조작 함수
void moveRobotArm(String action) {
  // 파싱 S1:150,S2:90,S3:120,S4:30
  for (int i = 0; i < 4; i++) {
    int startIndex = action.indexOf("S" + String(i+1) + ":");
    if (startIndex == -1) continue;
    startIndex += 3;
    int nextStart = action.indexOf("S" + String(i+2) + ":");
    float targetAngle;
    if (nextStart == -1) {
      targetAngle = action.substring(startIndex).toFloat();
    } else {
      targetAngle = action.substring(startIndex, nextStart).toFloat();
    }

    // 각도 제한 및 펄스 변환
    targetAngle = constrain(targetAngle, 0, 180);
    targetPulses[i] = angleToPulse(targetAngle);
  }

  // 현재 서보모터 이동
  for (int i = 0; i < 4; i++) {
    while (targetPulses[i] - currentPulses[i] > 10 || targetPulses[i] - currentPulses[i] < -10)
    {
      if (targetPulses[i] > currentPulses[i]) {
        currentPulses[i] += pulseStep;
      } else {
        currentPulses[i] -= pulseStep;
      }
      servos[i].writeMicroseconds(currentPulses[i]);
      delay(updateInterval); // 펄스 업데이트 간격 유지
    }
  }

  Serial.println("ROBOT_ARM:" + action);
}
//추가사항 종료/////////////////////////////////////////////////////////////////

