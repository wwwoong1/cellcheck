#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import paho.mqtt.client as mqtt

class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        # 1) ROS2 파라미터 선언 & 읽기
        self.declare_parameter('mqtt.broker',        'localhost')
        self.declare_parameter('mqtt.port',           1883)
        self.declare_parameter('mqtt.username',       '')
        self.declare_parameter('mqtt.password',       '')
        self.declare_parameter('mqtt.cafile',         '')
        self.declare_parameter('mqtt.topic_prefix',  'factory1/machine01')

        broker       = self.get_parameter('mqtt.broker').value
        port         = self.get_parameter('mqtt.port').value
        user         = self.get_parameter('mqtt.username').value
        passwd       = self.get_parameter('mqtt.password').value
        cafile       = self.get_parameter('mqtt.cafile').value
        self.topic_prefix = self.get_parameter('mqtt.topic_prefix').value

        # 2) MQTT 클라이언트 초기화
        self.mqtt = mqtt.Client()
        if cafile:
            self.mqtt.tls_set(ca_certs=cafile)
        if user:
            self.mqtt.username_pw_set(user, passwd)
        self.mqtt.on_connect    = lambda c, u, f, rc: self.get_logger().info(f"MQTT connected ({rc})")
        self.mqtt.on_disconnect = lambda c, u, rc: self.get_logger().warn(f"MQTT disconnected ({rc})")
        self.mqtt.connect(broker, port)
        self.mqtt.loop_start()

        # 3) DDS 구독: 4가지 영역별로 구독
        #  - SerialReaderNode 에서 발행하는 ttyACM1 포트 환경 데이터
        self.create_subscription(String, '/arduino/uno/raw', 
                                 self.cb_environment, 10)
        #  - ws_node 또는 다른 노드에서 발행하는 외관검사 결과
        self.create_subscription(String, '/inspection/appearance', 
                                 self.cb_appearance, 10)
        #  - SerialReaderNode 에서 발행하는 ttyACM1 포트 방전 데이터
        self.create_subscription(String, '/battery/discharge', 
                                 self.cb_battery_discharge, 10)
        #  - TelemetryNode 에서 발행하는 시스템 리소스 모니터링 데이터
        self.create_subscription(String, '/telemetry/system', 
                                 self.cb_system, 10)

    def cb_environment(self, msg: String):
        try:
            raw = json.loads(msg.data)
        except ValueError:
            # JSON 파싱 실패 시, raw에 빈 dict 또는 원본 문자열을 넣고 넘어가도 됩니다
            self.get_logger().warn(f"환경 데이터 JSON 파싱 실패, 원본: {msg.data}")
            return
        
        if 'ambient_temp' not in raw:
            return
        
        mapped = {
            'resistanceTemperatureA': raw.get('object_a_temp'),
            'resistanceTemperatureB': raw.get('object_b_temp'),
            'ambientTemp':            raw.get('ambient_temp'),
            'coolingFanA':            raw.get('cooling_fan_a'),
            'coolingFanB':            raw.get('cooling_fan_b'),
            'isFullError':            raw.get('is_full_error'),
            'isFullNormal':           raw.get('is_full_normal'),
        }
        topic = f"{self.topic_prefix}/environment"
        payload = json.dumps(mapped)
        self.mqtt.publish(topic, payload, qos=1)
        self.get_logger().info(f"[ENV] → {topic}: {payload}")

    def cb_appearance(self, msg: String):
        try:
            raw = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn(f"APPEAR JSON 파싱 실패: {msg.data}")
            return

        mapped = {
            'appearanceInspection': raw.get('abnormal')
        }

        # abnormal False(정상)이면 숫자→문자열 매핑, True(비정상)이면 None
        if raw.get('abnormal') is False:
            class_map = {
                0: 'AAA',
                1: 'AA',
                2: 'C',
                3: 'D'
            }
            idx = raw.get('class')
            mapped['batteryType'] = class_map.get(idx, 'UNKNOWN')
        else:
            mapped['batteryType'] = None
            # 필요하면 imageUrl도 추가
            # mapped['imageUrl'] = raw.get('imageUrl')

        topic = f"{self.topic_prefix}/appearance"
        payload = json.dumps(mapped)
        self.mqtt.publish(topic, payload, qos=1)
        self.get_logger().info(f"[APPEAR] → {topic}: {payload}")


    def cb_battery_discharge(self, msg: String):
        try:
            raw = json.loads(msg.data)
        except ValueError as e:
            self.get_logger().warn(f"DISCHRG JSON 파싱 실패: '{msg.data}' → {e}")
            return
        
         # SoC, circuitType 만 추출
        mapped = {
            'SoC':         raw.get('SoC'),
            'circuitType': raw.get('circuitType'),
        }
        topic = f"{self.topic_prefix}/discharge"
        payload = json.dumps(mapped)
        self.mqtt.publish(topic, payload, qos=1)
        self.get_logger().info(f"[BATTERY] → {topic}: {payload}")

    def cb_system(self, msg: String):
        try:
            raw = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn(f"SYS JSON 파싱 실패: {msg.data}")
            return
        mapped = {
            'cpu_percent': raw.get('cpu_percent'),
            'mem_used':    raw.get('mem_used'),
            'cpu_temp':    raw.get('cpu_temp'),
            'soc_temp':    raw.get('soc_temp'),
        }
        topic = f"{self.topic_prefix}/system"
        payload = json.dumps(mapped)
        self.mqtt.publish(topic, payload, qos=1)
        self.get_logger().info(f"[SYS] → {topic}: {payload}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.mqtt.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()