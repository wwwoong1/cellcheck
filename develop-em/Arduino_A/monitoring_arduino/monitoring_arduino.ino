#include <Wire.h>               // I2C통신 라이브러리
#include <Adafruit_INA219.h>    // INA219(전류센서) 라이브러리
#include <Adafruit_MLX90614.h>  // MLX90614(온도센서) 라이브러리

// 라이브러리 관련 전역 변수 선언
Adafruit_INA219 ina219;   // 전류센서
Adafruit_MLX90614 mlx_a;  // A회로 온도센서
Adafruit_MLX90614 mlx_b;  // B회로 온도센서

// 제어 핀 정의
const int circuit_relay_a_pin = 5;  // A회로 제어 릴레이 핀
const int circuit_relay_b_pin = 4;  // B회로 제어 릴레이 핀
const int fan_a_pin = 3;            // a쿨링팬 제어 릴레이 핀
const int fan_b_pin = 2;            // b쿨링팬 제어 릴레이 핀
const int ir_sensor_error_pin = 10;  // IR 센서 (에러분류통) 핀
const int ir_sensor_normal_pin = 11; // IR 센서 (정상분류통) 핀

// TCA9548A I2C 멀티플렉서 주소
const int tca9548a_multiplexer_addr = 0x70;

// TCA9548A 채널 선택 함수
void tcaselect(uint8_t i) {
  if (i > 7) return;
  
  Wire.beginTransmission(tca9548a_multiplexer_addr);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// 현재 연결된 회로 상태 ('O': 연결 해제, 'A': A회로, 'B': B회로)
char circuit_type = 'A';
bool is_first_connection = true;  // 첫 연결 여부를 추적하는 플래그
bool was_battery_connected = false;  // 배터리 연결 상태를 추적하는 플래그

// 상수 정의
const float temp_danger_threshold = 70.0;   // 위험 온도 임계값
const float fan_on_temp = 50.0;             // 쿨링팬 켜기 온도
const float fan_off_temp = 30.0;            // 쿨링팬 끄기 온도

// 시간 측정용 변수
unsigned long previous_millis = 0;
const long time_interval = 1000;  // 측정 간격 (1초)

void setup(void) {
  Serial.begin(9600);   // 시리얼통신 시작
  Wire.begin();         // I2C 시작(생략 가능)

  // 릴레이, 쿨링팬 출력으로 설정
  pinMode(circuit_relay_a_pin, OUTPUT);
  pinMode(circuit_relay_b_pin, OUTPUT);
  pinMode(fan_a_pin, OUTPUT);
  pinMode(fan_b_pin, OUTPUT);
  
  // IR 센서 입력으로 설정
  pinMode(ir_sensor_error_pin, INPUT);
  pinMode(ir_sensor_normal_pin, INPUT);

  // 초기값 설정(릴레이의 경우 HIGH가 연결안된 상태)
  digitalWrite(circuit_relay_a_pin, HIGH);
  digitalWrite(circuit_relay_b_pin, HIGH);
  digitalWrite(fan_a_pin, HIGH);
  digitalWrite(fan_b_pin, HIGH);

  // 전압/전류센서 초기화 (멀티플렉서 사용 안함)
  if (!ina219.begin()) {
    Serial.println("INA219가 존재하지 않습니다!");
    while (1) { delay(10); }
  }
  ina219.setCalibration_32V_2A();  // 2A까지 측정 가능하도록 변경

  // 온도센서A 초기화 (채널 2 사용)
  tcaselect(2);
  if (!mlx_a.begin()) {
    Serial.println("온도센서A가 존재하지 않습니다!");
    while (1) { delay(10); }
  }

  // 온도센서B 초기화 (채널 3 사용)
  tcaselect(3);
  if (!mlx_b.begin()) {
    Serial.println("온도센서B가 존재하지 않습니다!");
    while (1) { delay(10); }
  }

  Serial.println("모든 센서 초기화 완료");
}

void loop(void) {
  // 현재 시간 가져오기
  unsigned long current_millis = millis();
  
  // 1초마다 실행(오버플로우 방지)
  if (current_millis - previous_millis >= time_interval) {
    previous_millis = current_millis;
    
    // 온도센서A에서 온도 측정 (채널 2)
    tcaselect(2);
    float ambient_temp = mlx_a.readAmbientTempC();    // 외부 온도
    float object_a_temp = mlx_a.readObjectTempC();    // 타겟 a 온도
    
    // 온도센서B에서 온도 측정 (채널 3)
    tcaselect(3);
    float object_b_temp = mlx_b.readObjectTempC();    // 타겟 b 온도

    // 각 기능별 함수 호출
    printMonitoring(ambient_temp, object_a_temp, object_b_temp);    // 모니터링 데이터 출력 함수
    printCircuitData(object_a_temp, object_b_temp);   // 회로 및 전력 데이터 출력 함수
  }
  
  // 이 부분에 추가 작업이 필요하면 지연 없이 실행될 수 있음
}

// 모니터링 데이터 출력 함수 (JSON 형식)
void printMonitoring(float ambient_temp, float object_a_temp, float object_b_temp) {
  // 쿨링팬 제어
  if (object_a_temp > fan_on_temp) {
    digitalWrite(fan_a_pin, LOW);
  } else if (object_a_temp < fan_off_temp) {
    digitalWrite(fan_a_pin, HIGH);
  }
  
  if (object_b_temp > fan_on_temp) {
    digitalWrite(fan_b_pin, LOW);
  } else if (object_b_temp < fan_off_temp) {
    digitalWrite(fan_b_pin, HIGH);
  }

  // IR 센서 상태 읽기
  int is_full_error = digitalRead(ir_sensor_error_pin);
  int is_full_normal = digitalRead(ir_sensor_normal_pin);

  // JSON 문자열 구성
  String monitoring_json_data = "{";
  monitoring_json_data += "\"ambient_temp\":" + String(ambient_temp) + ",";
  monitoring_json_data += "\"object_a_temp\":" + String(object_a_temp) + ",";
  monitoring_json_data += "\"object_b_temp\":" + String(object_b_temp) + ",";
  monitoring_json_data += "\"cooling_fan_a\":" + String(digitalRead(fan_a_pin) == LOW ? "1" : "0") + ",";
  monitoring_json_data += "\"cooling_fan_b\":" + String(digitalRead(fan_b_pin) == LOW ? "1" : "0") + ",";
  monitoring_json_data += "\"is_full_error\":" + String(is_full_error) + ",";
  monitoring_json_data += "\"is_full_normal\":" + String(is_full_normal);
  monitoring_json_data += "}";

  // 한 번에 전송
  Serial.println(monitoring_json_data);
}

// 회로 및 전력 데이터 출력 함수 (JSON 형식)
void printCircuitData(float object_a_temp, float object_b_temp) {
  // 회로 제어
  if (object_a_temp >= temp_danger_threshold && object_b_temp >= temp_danger_threshold) {
    // 두 회로 모두 과열 시 모두 차단
    digitalWrite(circuit_relay_a_pin, HIGH);
    digitalWrite(circuit_relay_b_pin, HIGH);
    circuit_type = 'O';
  } else if (object_a_temp >= temp_danger_threshold) {
    // A회로 과열 시 B회로로 전환
    digitalWrite(circuit_relay_a_pin, HIGH);
    digitalWrite(circuit_relay_b_pin, LOW);
    circuit_type = 'B';
  } else if (object_b_temp >= temp_danger_threshold) {
    // B회로 과열 시 A회로로 전환
    digitalWrite(circuit_relay_b_pin, HIGH);
    digitalWrite(circuit_relay_a_pin, LOW);
    circuit_type = 'A';
  } else if (circuit_type == 'A' || circuit_type == 'O') {
    // 정상 온도에서 A회로 사용 (초기 상태 또는 A회로 사용 중이었을 경우)
    digitalWrite(circuit_relay_b_pin, HIGH);
    digitalWrite(circuit_relay_a_pin, LOW);
    circuit_type = 'A';
  } else {
    // 정상 온도에서 B회로 유지
    digitalWrite(circuit_relay_a_pin, HIGH);
    digitalWrite(circuit_relay_b_pin, LOW);
    circuit_type = 'B';
  }

  // 전력 측정 (멀티플렉서 채널 전환 없이 바로 측정)
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);

  // 배터리 연결 상태 확인 (전류가 1mA 이상일 때 연결된 것으로 판단)
  bool is_battery_connected = abs(current_mA) > 1.0;

  // 배터리가 새로 연결되었을 때만 first_connection 출력
  if (is_battery_connected && !was_battery_connected) {
    is_first_connection = true;
  }
  was_battery_connected = is_battery_connected;

  // JSON 문자열 구성
  String circuit_json_data = "{";
  circuit_json_data += "\"circuit_type\":\"" + String(circuit_type) + "\",";
  circuit_json_data += "\"bus_voltage\":" + String(busvoltage) + ",";
  circuit_json_data += "\"load_voltage\":" + String(loadvoltage) + ",";
  circuit_json_data += "\"current\":" + String(current_mA);
  if (is_first_connection && is_battery_connected) {
    circuit_json_data += ",\"first_connection\":true";
    is_first_connection = false;  // 플래그를 false로 설정하여 다음부터는 출력하지 않음
  }
  circuit_json_data += "}";

  // 한 번에 전송
  Serial.println(circuit_json_data);
}