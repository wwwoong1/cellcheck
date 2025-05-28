#!/usr/bin/env python3
import rclpy  # ROS 2 클라이언트 라이브러리 초기화 및 함수 제공
from rclpy.node import Node  # ROS 2의 기본 노드 클래스
from std_msgs.msg import String  # ROS 2 메시지 타입 String
import serial  # pySerial: 시리얼 통신 라이브러리
import json    # JSON 문자열 파싱/생성을 위한 표준 라이브러리

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader')  # 노드 이름을 'serial_reader'로 설정하여 ROS2 마스터에 등록

        # ---- 파라미터 선언 ------------------------------------------------
        # launch/YAML 파일에서 내려주는 설정을 받기 위해 선언
        port     = self.declare_parameter('serial_port', '').value
        baud     = self.declare_parameter('baudrate', 9600).value
        self.board_id = self.declare_parameter('board_id', '').value

        # 필수 파라미터 체크: 값이 없으면 노드 초기화 중단
        if not port or not self.board_id:
            self.get_logger().error(
                "serial_port 또는 board_id 파라미터가 설정되지 않았습니다."
            )
            raise RuntimeError("필수 파라미터 누락")

        # ---- 시리얼 포트 열기 ----------------------------------------------
        try:
            # port: '/dev/ttyACM1' 등, baud: 통신 속도, timeout: 읽기 대기 시간(초)
            self.ser = serial.Serial(port, baud, timeout=0.1)
            # 이전 세션의 잔여 데이터 삭제
            self.ser.reset_input_buffer()
            self.get_logger().info(f"{port} 포트 열림 (baud={baud})")
        except Exception as e:
            self.get_logger().error(f"{port} 열기 실패: {e}")
            raise

        # ---- 퍼블리셔 설정 ------------------------------------------------
        # '/arduino/{board_id}/raw' 토픽에 String 메시지 발행
        topic = f'/arduino/{self.board_id}/raw'
        self.pub = self.create_publisher(String, topic, 10)
        # QoS 깊이(depth)=10: 큐 최대 10개까지 메시지 보관
        self.get_logger().info(f"{port} → {topic} 연결됨")

        # ---- 주기적 실행 타이머 설정 ---------------------------------------
        # 1초 마다 read_port 메서드 호출
        self.create_timer(0.5, self.read_port)

    def read_port(self):
        # 시리얼 포트에서 한 줄 읽기
        try:
            if self.ser.in_waiting == 0:
                return
            raw = self.ser.readline().decode(errors='ignore').strip()
            if raw:
                self.get_logger().info(f"[시리얼 수신] raw = {repr(raw)}")
            # decode: 바이트열 → 문자열, errors='ignore': 디코딩 오류 무시
            # strip(): 문자열 양끝 공백이나 개행 제거
        except Exception as e:
            self.get_logger().warning(f"readline 오류 ({self.ser.port}): {e}")
            return  # 오류 시 해당 주기 건너뜀

        if not raw:
            return  # 읽은 내용이 없으면 종료하여 다음 주기로
        
        self.get_logger().debug(f"수신된 원본 문자열: '{raw}'")

        # ---- JSON 파싱 및 board_id 추가 -------------------------------------
        try:
            data = json.loads(raw)  # JSON 문자열 → Python dict
            data['board_id'] = self.board_id
            payload = json.dumps(data, ensure_ascii=False)
            # ensures_ascii=False: 한글이 깨지지 않도록
        except Exception as e:
            safe_raw = raw.replace('\x00', '\\x00')  # 널 문자 → 이스케이프 문자열로 치환
            self.get_logger().warning(f"JSON 파싱 실패: '{safe_raw}' → {e}")
            return

        # ---- 메시지 발행 --------------------------------------------------
        # String 메시지로 payload 발행
        msg = String()
        msg.data = payload
        self.pub.publish(msg)

    def destroy_node(self):
        # 노드 종료 시 시리얼 포트 닫기
        try:
            self.ser.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = SerialReaderNode()
    try:
        rclpy.spin(node)  # 콜백 대기
    finally:
        node.destroy_node()  # 종료 전 정리
        rclpy.shutdown()      # ROS 2 종료

if __name__ == '__main__':
    main()