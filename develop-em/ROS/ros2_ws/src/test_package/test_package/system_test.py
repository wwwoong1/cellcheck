#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading
import signal
import sys
import Jetson.GPIO as GPIO  # GPIO 라이브러리 (젯슨 나노에서도 작동)

class SystemTestNode(Node):
    def __init__(self):
        super().__init__('system_test_node')
        
        # 파라미터 설정
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('toggle_button_pin', 17)
        self.declare_parameter('status_led_pin', 22)
        
        # 파라미터 가져오기
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.toggle_button_pin = self.get_parameter('toggle_button_pin').value
        self.status_led_pin = self.get_parameter('status_led_pin').value
        
        # 시스템 상태
        self.system_running = False
        
        # 버튼 디바운싱을 위한 변수
        self.last_button_time = 0
        self.debounce_time = 0.3  # 초 단위
        
        # 토픽 발행자
        self.state_publisher = self.create_publisher(
            String, 
            'system/state', 
            10
        )
        
        # 아두이노 시리얼 연결
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f'시리얼 포트 {self.serial_port} 연결 성공')
            
            # 시리얼 버퍼 비우기
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            # 시리얼 통신 스레드 시작
            self.serial_thread = threading.Thread(target=self.serial_read_thread)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
        except serial.SerialException as e:
            self.get_logger().error(f'시리얼 포트 연결 실패: {e}')
            self.ser = None
        
        # GPIO 설정 (젯슨 나노/라즈베리파이)
        GPIO.setmode(GPIO.BCM)  # BCM 모드 사용
        
        # 토글 버튼 설정 (풀업 저항 사용, 버튼 누르면 LOW)
        GPIO.setup(self.toggle_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # 상태 LED 핀 설정
        GPIO.setup(self.status_led_pin, GPIO.OUT)
        GPIO.output(self.status_led_pin, GPIO.LOW)  # 초기 상태: 꺼짐
        
        # GPIO 이벤트 감지 설정 (버튼이 눌릴 때 콜백 호출)
        GPIO.add_event_detect(self.toggle_button_pin, GPIO.FALLING, 
                             callback=self.toggle_button_callback, bouncetime=300)
        
        # 타이머 생성 (상태 업데이트를 위해)
        self.state_timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('시스템 테스트 노드 초기화 완료')
    
    def toggle_button_callback(self, channel):
        """토글 버튼이 눌렸을 때 호출되는 콜백"""
        # 디바운싱 (소프트웨어 방식)
        current_time = time.time()
        if (current_time - self.last_button_time) < self.debounce_time:
            return
        
        self.last_button_time = current_time
        
        # 시스템 상태 토글
        if self.system_running:
            self.get_logger().info('버튼 눌림. 시스템 정지...')
            self.stop_system()
        else:
            self.get_logger().info('버튼 눌림. 시스템 시작...')
            self.start_system()
    
    def start_system(self):
        """시스템 시작 명령"""
        if self.ser:
            try:
                self.ser.write(b'START_SYSTEM\n')
                self.get_logger().info('START_SYSTEM 명령 전송됨')
                
                # 상태 LED 켜기
                GPIO.output(self.status_led_pin, GPIO.HIGH)
                
                # 시스템 상태 업데이트
                self.system_running = True
                
                # 상태 발행
                msg = String()
                msg.data = 'RUNNING'
                self.state_publisher.publish(msg)
                
            except serial.SerialException as e:
                self.get_logger().error(f'시리얼 명령 전송 실패: {e}')
    
    def stop_system(self):
        """시스템 정지 명령"""
        if self.ser:
            try:
                self.ser.write(b'STOP_SYSTEM\n')
                self.get_logger().info('STOP_SYSTEM 명령 전송됨')
                
                # 상태 LED 끄기
                GPIO.output(self.status_led_pin, GPIO.LOW)
                
                # 시스템 상태 업데이트
                self.system_running = False
                
                # 상태 발행
                msg = String()
                msg.data = 'IDLE'
                self.state_publisher.publish(msg)
                
            except serial.SerialException as e:
                self.get_logger().error(f'시리얼 명령 전송 실패: {e}')
    
    def serial_read_thread(self):
        """아두이노로부터 데이터를 읽는 스레드"""
        while True and self.ser is not None:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').rstrip()
                    self.get_logger().info(f'아두이노로부터 수신: {line}')
                    
                    # 아두이노 응답 처리
                    if line == "SYSTEM_STARTED":
                        self.get_logger().info('시스템이 시작되었습니다')
                    elif line == "SYSTEM_STOPPED":
                        self.get_logger().info('시스템이 정지되었습니다')
                    elif line == "ARDUINO_B_READY":
                        self.get_logger().info('아두이노 B가 준비되었습니다')
                    
                time.sleep(0.01)  # CPU 사용률 감소
            except Exception as e:
                self.get_logger().error(f'시리얼 읽기 오류: {e}')
                time.sleep(1)  # 오류 발생 시 잠시 대기
    
    def publish_state(self):
        """시스템 상태 발행"""
        msg = String()
        if self.system_running:
            msg.data = 'RUNNING'
        else:
            msg.data = 'IDLE'
        self.state_publisher.publish(msg)
    
    def cleanup(self):
        """노드 종료 시 정리 작업"""
        if self.ser:
            # 시스템이 실행 중이면 정지
            if self.system_running:
                self.stop_system()
                time.sleep(0.5)  # 명령이 처리될 시간 부여
            
            # 시리얼 연결 종료
            self.ser.close()
        
        # GPIO 정리
        GPIO.cleanup()
        
        self.get_logger().info('시스템 테스트 노드 정리 완료')


def main(args=None):
    rclpy.init(args=args)
    
    # 노드 생성
    system_test_node = SystemTestNode()
    
    # 시그널 핸들러 설정 (Ctrl+C 처리)
    def signal_handler(sig, frame):
        system_test_node.get_logger().info('종료 신호 수신. 정리 작업 수행...')
        system_test_node.cleanup()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # 노드 실행
        rclpy.spin(system_test_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 정리 작업
        system_test_node.cleanup()
        system_test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
