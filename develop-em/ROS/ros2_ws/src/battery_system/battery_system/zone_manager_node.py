#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json

class ZoneManagerNode(Node):
    def __init__(self):
        super().__init__('zone_manager_node')
        
        # 구역 상태
        self.zone_a_state = "FREE"  # FREE 또는 OCCUPIED
        self.zone_b_state = "FREE"  # FREE 또는 OCCUPIED
        self.zone_b_processing = "IDLE"  # IDLE, PROCESSING, WAITING_ROBOT
        
        # 타이머 객체 저장
        self.barrier_a_timer = None
        self.zone_b_timer = None
        self.barrier_b_timer = None

        # 방전 진행 상태 변수
        self.discharging = False

        # 시스템 상태
        self.system_state = "IDLE"  # IDLE 또는 RUNNING
        
        # 퍼블리셔
        self.zone_a_publisher = self.create_publisher(
            String, '/zones/zoneA', 10)
        self.zone_b_publisher = self.create_publisher(
            String, '/zones/zoneB', 10)
        self.zone_b_processing_publisher = self.create_publisher(
            String, '/zones/zoneB/processing', 10)
        self.barrier_a_publisher = self.create_publisher(
            Bool, '/control/barrier_a', 10)
        self.barrier_b_publisher = self.create_publisher(
            Bool, '/control/barrier_b', 10)
        self.conveyor_publisher = self.create_publisher(
            Bool, '/control/conveyor', 10)
        
    # 액추에이터 명령 발행자 추가
        self.actuator_command_publisher = self.create_publisher(
            String, '/control/actuator_command', 10)
        
        # 로봇팔 명령 발행자 추가 (필요시)
        self.robot_command_publisher = self.create_publisher(
            String, '/robot/command', 10)
        
        #추가사항 시작/////////////////////////////////////////////////////////////////
        # 로봇팔 초기 위치 복귀 발행자 추가
        self.robot_init_publisher = self.create_publisher(
            Bool, '/robot/init', 10)
        #추가사항 종료/////////////////////////////////////////////////////////////////

        # 구독자
        self.system_state_subscriber = self.create_subscription(
            String, '/system/state', self.system_state_callback, 10)
        self.ir_a_subscriber = self.create_subscription(
            Bool, '/sensor/ir_a', self.ir_a_callback, 10)
        self.ir_b_subscriber = self.create_subscription(
            Bool, '/sensor/ir_b', self.ir_b_callback, 10)
        self.vision_result_subscriber = self.create_subscription(
            String, '/inspection/appearance', self.vision_result_callback, 10)
        self.robot_complete_subscriber = self.create_subscription(
            Bool, '/robot/complete', self.robot_complete_callback, 10)
        
            # 액추에이터 완료 신호 구독자 추가
        self.actuator_complete_subscriber = self.create_subscription(
            String, '/arduino/actuator_complete', self.actuator_complete_callback, 10)
        
        # 차단판A 닫힘 완료 신호 구독자 추가
        self.barrier_a_complete_subscriber = self.create_subscription(
            String, '/arduino/barrier_a_complete', self.barrier_a_complete_callback, 10)

        # 차단판B 닫힘 완료 신호 구독자 추가
        self.barrier_complete_subscriber = self.create_subscription(
            String, '/arduino/barrier_complete', self.barrier_complete_callback, 10)
        
        # # 방전 상태 구독자 추가
        self.discharge_complete_subscriber = self.create_subscription(
            Bool, '/battery/discharge_complete', self.discharge_complete_callback, 10
        )

        # 로봇팔 초기화 완료 구독자 추가
        self.robot_init_complete_subscriber = self.create_subscription(
            String, '/robot/init_complete', self.robot_init_complete_callback, 10
        )
        
        # 구역 상태 퍼블리싱 타이머 (0.5초마다)
        self.timer = self.create_timer(0.5, self.publish_zone_states)
        
        self.get_logger().info('Zone Manager Node started')
    
    def system_state_callback(self, msg):
        """시스템 상태 업데이트"""
        self.system_state = msg.data
        
        # 시스템 상태가 IDLE로 변경되면 기존 타이머 취소
        if self.system_state == "IDLE":
            if self.barrier_a_timer:
                self.barrier_a_timer.cancel()
                self.barrier_a_timer = None
            if self.zone_b_timer:
                self.zone_b_timer.cancel()
                self.zone_b_timer = None
    
    def ir_a_callback(self, msg):
        """적외선 센서A 감지 처리"""
        if self.system_state != "RUNNING":
            return
            
        if msg.data:  # 건전지 감지됨
            self.zone_a_state = "OCCUPIED"
            self.get_logger().info('Zone A: Cell detected')
            
            # 15초 후 구역B 확인 후 차단판A 열기 타이머 설정
            if self.barrier_a_timer:
                self.barrier_a_timer.cancel()
            self.barrier_a_timer = self.create_timer(5.0, self.check_and_open_barrier_a)
    
    def check_and_open_barrier_a(self):
        """15초 후 구역B 상태 확인 및 차단판A 제어"""
        if self.barrier_a_timer:
            self.barrier_a_timer.cancel()
            self.barrier_a_timer = None
            
        # 구역B가 비어있을 때만 차단판A 열기
        if self.zone_b_state == "FREE":
            self.get_logger().info('Opening barrier A (Zone B is free)')
            self.open_barrier_a()
        else:
            # 구역B가 아직 점유 중이면 1초마다 다시 확인
            self.get_logger().info('Zone B is still occupied, waiting...')
            self.barrier_a_timer = self.create_timer(1.0, self.check_and_open_barrier_a)
    
    def ir_b_callback(self, msg):
        """적외선 센서B 감지 처리"""
        if self.system_state != "RUNNING":
            return
            
        if msg.data and self.zone_b_state == "FREE":  # 건전지 감지됨 && 구역B가 비어있을 때만 처리
            
            self.zone_b_state = "OCCUPIED"  # 구역B 점유
            self.zone_b_processing = "IDLE"  # 처리 대기 상태
            self.get_logger().info('Zone B: Cell detected, Zone A is now free')
            
            # 구역B 처리 상태 발행
            self.publish_zone_b_processing()
    
    def vision_result_callback(self, msg):
        """비전 결과 처리"""
        if self.system_state != "RUNNING" or self.zone_b_state != "OCCUPIED":
            self.get_logger().info('Ignoring vision result - system not ready')
            return
        
        try:    
            result = json.loads(msg.data)
            
            # Python의 bool() 함수 활용하거나 특정 조건으로 변환
            abnormal_raw = result.get('abnormal', False)
            
            # 문자열일 경우 "true"와 비교, 아니면 그냥 bool() 적용
            if isinstance(abnormal_raw, str):
                abnormal = abnormal_raw.lower() == "true"
            else:
                abnormal = bool(abnormal_raw)
            
            # 디버깅을 위한 타입 출력
            self.get_logger().info(f'Abnormal value: {abnormal} (type: {type(abnormal).__name__})')
                
            battery_class = result.get('class', -1)
            bbox = result.get('bbox', [0, 0, 0, 0])

            self.get_logger().info(f'Vision result received: {result}')
            self.get_logger().info(f'Abnormal: {abnormal}, Class: {battery_class}, BBox: {bbox}')
            
            # 컨베이어 벨트 정지
            self.stop_conveyor()
            
            # 명시적 비교
            if abnormal:  # Boolean 처리
                self.zone_b_processing = "PROCESSING"
                self.get_logger().info('Defect item detected - activating actuator')
                self.publish_zone_b_processing()
                
                # 액추에이터 명령 발행
                actuator_msg = String()
                actuator_msg.data = "CYCLE"
                self.actuator_command_publisher.publish(actuator_msg)
                self.get_logger().info('Actuator command published: CYCLE')
                
            else:
                self.zone_b_processing = "WAITING_ROBOT"
                self.discharging = True  # 방전 진행 중으로 설정
                self.get_logger().info(f'Normal item detected - Class: {battery_class}')
                self.publish_zone_b_processing()

                # 로봇팔 명령 데이터 구성 - 원본 비전 결과 형식 유지
                robot_command_data = {
                    "abnormal": False,  # 정상품임을 명시
                    "class": battery_class,
                    "bbox": bbox
                }
                
                # JSON 직렬화 및 메시지 생성
                robot_msg = String()
                robot_msg.data = json.dumps(robot_command_data)
                
                # 로봇팔에게 명령 전송
                self.robot_command_publisher.publish(robot_msg)
                self.get_logger().info(f'Robot command sent: {robot_msg.data}')


        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse vision result JSON: {e}')
        except KeyError as e:
            self.get_logger().error(f'Missing key in vision result: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing vision result: {e}')
    
    def robot_complete_callback(self, msg):
        """로봇 작업 완료 처리 (로그 기록용)"""
        if msg.data:
            self.get_logger().info('Robot arm returned to home position')


    def discharge_complete_callback(self, msg):
        if self.system_state != "RUNNING" or self.zone_b_state != "OCCUPIED":
            return
        
        if msg.data and self.zone_b_state == "OCCUPIED" and self.zone_b_processing == "WAITING_ROBOT":
            self.get_logger().info('Robot work completed - opening barrier B')
            
            self.discharging = False

            # 구역A도 점유 상태인지 확인
            if self.zone_a_state == "OCCUPIED":
                # 구역A에도 건전지가 있으면 차단판 A와 B를 동시에 열기
                self.get_logger().info('Both zones have batteries - opening both barriers simultaneously')
                
                #추가사항 시작/////////////////////////////////////////////////////////////////
                # 로봇팔 초기 위치 복귀
                self.initRobotArm()
                #추가사항 종료/////////////////////////////////////////////////////////////////

                # 컨베이어 벨트 시작
                # self.start_conveyor()
                
                # # 차단판A와 B 동시에 열기
                # self.open_barrier_a()
                # self.open_barrier_b()
                
                # # 구역B 상태 업데이트 (차단판B가 닫힐 때 콜백에서도 처리되지만 미리 처리)
                # self.zone_b_state = "FREE"
                # self.zone_b_processing = "IDLE"
                # self.publish_zone_states()
                # self.publish_zone_b_processing()
                
            else:
                #추가사항 시작/////////////////////////////////////////////////////////////////
                # 로봇팔 초기 위치 복귀
                self.initRobotArm()
                #추가사항 종료/////////////////////////////////////////////////////////////////

                # 구역A가 비어있으면 기존 로직대로 처리
                # self.start_conveyor()
                
                # # 컨베이어 벨트 시작 후 잠시 대기 후 차단판B 열기 위한 타이머 설정
                # if self.barrier_b_timer:
                #     self.barrier_b_timer.cancel()
                
                # # 2초 후 차단판B 열기 (구역A가 비어있으면 기존대로 B만 열기)
                # self.barrier_b_timer = self.create_timer(2.0, self.delayed_open_barrier_b)


    def delayed_open_barrier_b(self):
        """컨베이어 벨트 시작 후 지연된 차단판B 열기"""
        if self.barrier_b_timer:
            self.barrier_b_timer.cancel()
            self.barrier_b_timer = None
            
        self.get_logger().info('Opening barrier B after conveyor start')
        # 차단판B 열기
        self.open_barrier_b()        

    #추가사항 시작/////////////////////////////////////////////////////////////////
    def initRobotArm(self):
        """로봇팔 초기 위치 복귀"""
        msg = Bool()
        msg.data = True
        self.robot_init_publisher.publish(msg)
    #추가사항 종료/////////////////////////////////////////////////////////////////
    
    def open_barrier_a(self):
        """차단판A 열기"""
        msg = Bool()
        msg.data = True
        self.barrier_a_publisher.publish(msg)
    
    def open_barrier_b(self):
        """차단판B 열기"""
        msg = Bool()
        msg.data = True
        self.barrier_b_publisher.publish(msg)
    
    def stop_conveyor(self):
        """컨베이어 벨트 정지"""
        msg = Bool()
        msg.data = False
        self.conveyor_publisher.publish(msg)
        self.get_logger().info('Conveyor stopped')
    
    def start_conveyor(self):
        """컨베이어 벨트 시작"""
        msg = Bool()
        msg.data = True
        self.conveyor_publisher.publish(msg)
        self.get_logger().info('Conveyor started')
    
    def publish_zone_states(self):
        """구역 상태 발행"""
        msg_a = String()
        msg_a.data = self.zone_a_state
        self.zone_a_publisher.publish(msg_a)
        
        msg_b = String()
        msg_b.data = self.zone_b_state
        self.zone_b_publisher.publish(msg_b)
    
    def publish_zone_b_processing(self):
        """구역B 처리 상태 발행"""
        msg = String()
        msg.data = self.zone_b_processing
        self.zone_b_processing_publisher.publish(msg)


    def actuator_complete_callback(self, msg):
        """액추에이터 동작 완료 처리"""
        self.get_logger().info(f'Received actuator complete signal: {msg.data}')
        
        # if self.system_state != "RUNNING" or self.zone_b_state != "OCCUPIED":
        #     self.get_logger().info('Ignoring actuator complete - system not in right state')
        #     return
            
        if msg.data == "COMPLETE":
            self.get_logger().info('Actuator operation completed')
            
            # 구역B 상태 업데이트
            self.zone_b_state = "FREE"
            self.zone_b_processing = "IDLE"
            
            # 상태 발행
            self.publish_zone_states()
            self.publish_zone_b_processing()
            
            # 컨베이어 벨트 재시작
            self.start_conveyor()
            
            self.get_logger().info('Zone B cleared after actuator operation')

    def barrier_complete_callback(self, msg):
        """차단판B 닫힘 완료 처리"""
        if self.system_state != "RUNNING" or self.zone_b_state != "OCCUPIED":
            return
            
        if msg.data == "CLOSED":
            self.get_logger().info('Barrier B closing completed')
            
            # 구역B 상태 업데이트
            self.zone_b_state = "FREE"
            self.zone_b_processing = "IDLE"
            
            # 상태 발행
            self.publish_zone_states()
            self.publish_zone_b_processing()
            
            # 컨베이어 벨트 재시작
            self.start_conveyor()
            
            self.get_logger().info('Zone B cleared after barrier B operation')

    def barrier_a_complete_callback(self, msg):
        """차단판A 닫힘 완료 처리"""
        if self.system_state != "RUNNING":
            return
            
        if msg.data == "CLOSED":
            self.get_logger().info('Barrier A closing completed')
            
            # 구역A 상태 업데이트
            self.zone_a_state = "FREE"
            
            # 상태 발행
            self.publish_zone_states()
            
            self.get_logger().info('Zone A state reset to FREE after barrier A operation')

    # 로봇팔 초기화 완료 콜백 함수 추가
    def robot_init_complete_callback(self, msg):
        """로봇팔 초기화 완료 처리"""
        if self.system_state != "RUNNING" or self.zone_b_state != "OCCUPIED":
            return
            
        if msg.data == "INIT_COMPLETE":
            self.get_logger().info('Robot arm initialization completed')
            
            # 로봇팔 초기화 완료 후 차단판B 작업 진행
            if self.zone_b_state == "OCCUPIED" and self.zone_b_processing == "WAITING_ROBOT":
                self.get_logger().info('Robot work completed - opening barrier B')
                
                # 구역A도 점유 상태인지 확인
                if self.zone_a_state == "OCCUPIED":
                    # 구역A에도 건전지가 있으면 차단판 A와 B를 동시에 열기
                    self.get_logger().info('Both zones have batteries - opening both barriers simultaneously')
                    
                    # 컨베이어 벨트 시작
                    self.start_conveyor()
                    
                    # 차단판A와 B 동시에 열기
                    self.open_barrier_a()
                    self.open_barrier_b()
                    
                    # 구역B 상태 업데이트 (차단판B가 닫힐 때 콜백에서도 처리되지만 미리 처리)
                    self.zone_b_state = "FREE"
                    self.zone_b_processing = "IDLE"
                    self.publish_zone_states()
                    self.publish_zone_b_processing()
                    
                else:
                    # 구역A가 비어있으면 기존 로직대로 처리
                    self.start_conveyor()
                    
                    # 컨베이어 벨트 시작 후 잠시 대기 후 차단판B 열기 위한 타이머 설정
                    if self.barrier_b_timer:
                        self.barrier_b_timer.cancel()
                    
                    # 1초 후 차단판B 열기 (구역A가 비어있으면 기존대로 B만 열기)
                    self.barrier_b_timer = self.create_timer(1.0, self.delayed_open_barrier_b)
    


def main(args=None):
    rclpy.init(args=args)
    node = ZoneManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
