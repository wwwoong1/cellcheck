#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class SystemManagerNode(Node):
    def __init__(self):
        super().__init__('system_manager_node')
        
        # 상태 변수
        self.system_state = "IDLE"  # IDLE 또는 RUNNING
        
        # 퍼블리셔
        self.state_publisher = self.create_publisher(
            String, '/system/state', 10)
        
        # 구독자
        self.button_subscriber = self.create_subscription(
            String, '/button_event', self.button_callback, 10)
        
        # 상태 퍼블리싱 타이머 (1초마다)
        self.timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('System Manager Node started')
    
    def button_callback(self, msg):
        """버튼 이벤트 처리"""
        if msg.data == "START" and self.system_state == "IDLE":
            self.system_state = "RUNNING"
            self.get_logger().info('System started')
        elif msg.data == "STOP" and self.system_state == "RUNNING":
            self.system_state = "IDLE"
            self.get_logger().info('System stopped')
    
    def publish_state(self):
        """시스템 상태 발행"""
        msg = String()
        msg.data = self.system_state
        self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
