#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
import serial
import threading
import time

class ArduinoControllerNode(Node):
    def __init__(self):
        super().__init__('arduino_controller_node')
        
        # 시리얼 포트 설정 (실제 환경에 맞게 조정 필요)
        self.serial_port = None
        try:
            self.serial_port = serial.Serial(
                port='/dev/arduino_controller',  # 아두이노 포트 (환경에 맞게 변경)
                baudrate=9600,        # 9600 보드레이트 사용
                timeout=1.0
            )
            self.get_logger().info('Arduino connected')
            time.sleep(2.0)  # 아두이노 초기화 대기
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
        
        # 퍼블리셔
        self.button_publisher = self.create_publisher(
            String, '/button_event', 10)
        self.ir_a_publisher = self.create_publisher(
            Bool, '/sensor/ir_a', 10)
        self.ir_b_publisher = self.create_publisher(
            Bool, '/sensor/ir_b', 10)
        # 로봇팔 초기 위치 복귀 완료 신호 발행자 추가
        self.robot_init_complete_publisher = self.create_publisher(
            String, '/robot/init_complete', 10)
        
        # 액추에이터/차단판B 완료 신호 발행자 추가
        self.actuator_complete_publisher = self.create_publisher(
            String, '/arduino/actuator_complete', 10)
        self.barrier_complete_publisher = self.create_publisher(
            String, '/arduino/barrier_complete', 10)
        self.barrier_a_complete_publisher = self.create_publisher(  # 추가
            String, '/arduino/barrier_a_complete', 10)


        # 구독자
        self.barrier_a_subscriber = self.create_subscription(
            Bool, '/control/barrier_a', self.barrier_a_callback, 10)
        self.barrier_b_subscriber = self.create_subscription(
            Bool, '/control/barrier_b', self.barrier_b_callback, 10)
        self.conveyor_subscriber = self.create_subscription(
            Bool, '/control/conveyor', self.conveyor_callback, 10)

        # 액추에이터 명령 구독자 추가
        self.actuator_command_subscriber = self.create_subscription(
            String, '/control/actuator_command', self.actuator_command_callback, 10)
        
        #추가사항 시작/////////////////////////////////////////////////////////////////
        # 로봇팔 각도 토픽 구독자 추가
        self.subscription = self.create_subscription(
            JointState,'arduino_joint_states',self.joint_states_callback,10)
        # 로봇팔 초기 위치 복귀 구독자 추가
        self.robot_init_subscriber = self.create_subscription(
            Bool, '/robot/init', self.robot_init_callback, 10)
        #추가사항 종료/////////////////////////////////////////////////////////////////

        # 로봇팔 완료 신호 구독자 (테스트용)
        self.robot_complete_subscriber = self.create_subscription(
            Bool, '/robot/complete', self.robot_complete_callback, 10)
        
        # 시리얼 읽기 스레드 시작
        if self.serial_port:
            self.serial_thread = threading.Thread(target=self.read_serial)
            self.serial_thread.daemon = True
            self.serial_thread.start()
        
        self.get_logger().info('Arduino Controller Node started')
    
    def read_serial(self):
        """아두이노로부터 데이터 읽기"""
        while rclpy.ok():
            if self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.readline().decode('utf-8').strip()
                        self.process_arduino_data(data)
                except Exception as e:
                    self.get_logger().error(f'Serial read error: {str(e)}')
                    time.sleep(1.0)
            else:
                time.sleep(1.0)
    
    def process_arduino_data(self, data):
        """아두이노 데이터 처리"""
        self.get_logger().debug(f'Received: {data}')
        
        if data.startswith("BUTTON:"):
            # 버튼 이벤트 처리
            button_event = data.split(":")[1]
            msg = String()
            msg.data = button_event
            self.button_publisher.publish(msg)
            self.get_logger().info(f'Button event: {button_event}')
        
        elif data.startswith("IR_A:"):
            # 적외선 센서A 처리
            detected = data.split(":")[1] == "1"
            msg = Bool()
            msg.data = detected
            self.ir_a_publisher.publish(msg)
            if detected:
                self.get_logger().info('IR Sensor A: Cell detected')
        
        elif data.startswith("IR_B:"):
            # 적외선 센서B 처리
            detected = data.split(":")[1] == "1"
            msg = Bool()
            msg.data = detected
            self.ir_b_publisher.publish(msg)
            if detected:
                self.get_logger().info('IR Sensor B: Cell detected')
    
        elif data.startswith("ACTUATOR:COMPLETE"):
            # 액추에이터 동작 완료 신호 처리
            msg = String()
            msg.data = "COMPLETE"
            self.actuator_complete_publisher.publish(msg)
            self.get_logger().info('Actuator cycle completed')
        
        elif data.startswith("BARRIER_B:CLOSED"):
            # 차단판B 닫힘 완료 신호 처리
            msg = String()
            msg.data = "CLOSED"
            self.barrier_complete_publisher.publish(msg)
            self.get_logger().info('Barrier B closed')

        elif data.startswith("BARRIER_A:CLOSED"):
            # 차단판A 닫힘 완료 신호 처리
            msg = String()
            msg.data = "CLOSED"
            self.barrier_a_complete_publisher.publish(msg)
            self.get_logger().info('Barrier A closed')
        
        elif data.startswith("ROBOT_ARM:INIT_COMPLETE"):
            # 로봇팔 초기 위치 복귀 완료 신호 처리
            msg = String()
            msg.data = "INIT_COMPLETE"
            self.robot_init_complete_publisher.publish(msg)
            self.get_logger().info('Robot arm initialized')
    

    def actuator_command_callback(self, msg):
        """액추에이터 명령 처리"""
        if self.serial_port and self.serial_port.is_open:
            try:
                command = f"ACTUATOR:{msg.data}"
                self.serial_port.write(f"{command}\n".encode('utf-8'))
                self.get_logger().info(f'Sent actuator command: {command}')
            except Exception as e:
                self.get_logger().error(f'Failed to send actuator command: {str(e)}')
    
    def actuator_callback(self, msg):
        """리니어 액추에이터 제어 (이전 방식 - 현재는 사용 안함)"""
        if self.serial_port and self.serial_port.is_open:
            try:
                if msg.data:
                    command = "ACTUATOR:EXTEND"  # 액추에이터 연장
                else:
                    command = "ACTUATOR:RETRACT"  # 액추에이터 수축
                    
                self.serial_port.write(f"{command}\n".encode('utf-8'))
                self.get_logger().info(f'Sent: {command}')
            except Exception as e:
                self.get_logger().error(f'Failed to send actuator command: {str(e)}')
    
    def conveyor_callback(self, msg):
        """컨베이어 벨트 제어"""
        if self.serial_port and self.serial_port.is_open:
            try:
                command = "CONVEYOR:1" if msg.data else "CONVEYOR:0"
                self.serial_port.write(f"{command}\n".encode('utf-8'))
                self.get_logger().info(f'Sent: {command}')
            except Exception as e:
                self.get_logger().error(f'Failed to send conveyor command: {str(e)}')

    #추가사항 시작/////////////////////////////////////////////////////////////////
    def joint_states_callback(self, msg):
        """로봇팔 각도 토픽 처리"""
        if self.serial_port and self.serial_port.is_open:
            try:
                # 각도 데이터 추출
                angles = [angle for angle in msg.position]
                data = f"ROBOT_ARM:S1:{angles[0]:.2f},S2:{angles[1]:.2f},S3:{angles[2]:.2f},S4:{angles[3]:.2f}\n"
                self.get_logger().error(f'Sent: {data}')
                self.serial_port.write(data.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Failed to send joint states: {str(e)}') 
        else:
            self.get_logger().error('joint_states check')
            angles = [angle * 180.0 / 3.14159 for angle in msg.position]
            self.get_logger().error(f'Sent: {angles[0]:.2f}, {angles[1]:.2f}, {angles[2]:.2f}, {angles[3]:.2f}')
            self.get_logger().error('Serial port is not open')
    
    def robot_init_callback(self, msg):
        """로봇팔 초기 위치 복귀 처리"""
        if self.serial_port and self.serial_port.is_open:
            try:
                command = "ROBOT_ARM:INIT"
                self.get_logger().error(f'Sent: {command}')
                self.serial_port.write(f"{command}\n".encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Failed to send robot init command: {str(e)}')
        else:
            self.get_logger().error('robot_init check')
            command = "ROBOT_ARM:INIT"
            self.get_logger().error(f"{command}")
            self.get_logger().error('Serial port is not open')
    #추가사항 종료/////////////////////////////////////////////////////////////////

    def robot_complete_callback(self, msg):
        """로봇팔 완료 신호 (테스트용)"""
        if msg.data:
            self.get_logger().info('Robot arm completed its work')
    
    def barrier_a_callback(self, msg):
        """차단판A 제어"""
        if self.serial_port and self.serial_port.is_open:
            try:
                command = "BARRIER_A:1" if msg.data else "BARRIER_A:0"
                self.serial_port.write(f"{command}\n".encode('utf-8'))
                self.get_logger().info(f'Sent: {command}')
            except Exception as e:
                self.get_logger().error(f'Failed to send barrier A command: {str(e)}')
    
    def barrier_b_callback(self, msg):
        """차단판B 제어"""
        if self.serial_port and self.serial_port.is_open:
            try:
                command = "BARRIER_B:1" if msg.data else "BARRIER_B:0"
                self.serial_port.write(f"{command}\n".encode('utf-8'))
                self.get_logger().info(f'Sent: {command}')
            except Exception as e:
                self.get_logger().error(f'Failed to send barrier B command: {str(e)}')
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()