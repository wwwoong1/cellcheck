#!/usr/bin/env python3
# telemetry_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json
import os

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        # 퍼블리시 주기 (Hz)
        self.declare_parameter('publish_frequency', 1.0)
        freq = self.get_parameter('publish_frequency').value

        # ROS2 퍼블리셔
        self.pub = self.create_publisher(String, '/telemetry/system', 10)

        # 타이머 설정
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)
        self.get_logger().info(f"TelemetryNode started, publishing at {freq} Hz")

    def read_temp_file(self, path):
        """ /sys/class/thermal/... 에서 온도(밀리°C) 읽어 °C로 반환 """
        try:
            with open(path, 'r') as f:
                return float(f.read().strip()) / 1000.0
        except Exception:
            return None

    def timer_callback(self):
        # CPU 사용률, 메모리 사용량
        cpu_percent = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory()
        mem_used = mem.used / 1073741824 / 4 * 100  # bytes

        # 온도 읽기 (Jetson Nano thermal_zone0: SoC, zone1: CPU—환경에 따라 경로 조정)
        soc_temp = self.read_temp_file('/sys/class/thermal/thermal_zone0/temp')
        cpu_temp = self.read_temp_file('/sys/class/thermal/thermal_zone1/temp')

        data = {
            'cpu_percent': cpu_percent,
            'mem_used':    mem_used,
            'cpu_temp':    cpu_temp,
            'soc_temp':    soc_temp,
        }
        payload = json.dumps(data)

        # 퍼블리시
        msg = String(data=payload)
        self.pub.publish(msg)
        self.get_logger().debug(f"Published telemetry: {payload}")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
