#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Point, Quaternion
import json

# 건전지 크기(직경 mm x 높이 mm)
# AAA : 10.5 x 44.5
# AA : 14.5 x 50.5
# C : 26.2 x 50.0
# D : 34.2 x 61.5

# 로봇팔 고정부분과 카메라 사이 거리 115mm
# 카메라 높이는 로봇팔 고정부분과 같음

class CameraUVBox(Node):
    def __init__(self):
        super().__init__('camera_uvbox')
        
        # 검사 결과 데이터를 저장할 변수들
        self.is_abnormal = False
        self.appearance_class = 0  # 0='AAA', 1='AA', 2='C', 3='D'
        self.bbox_x = 0.0
        self.bbox_y = 0.0
        self.bbox_width = 0.0
        self.bbox_height = 0.0
        
        # 카메라 내부 파라미터
        self.fx = 2494.57
        self.fy = 2499.13
        self.cx = 1640
        self.cy = 1232

        # 깊이 Z값 (mm 단위, 예: 고정된 평면)
        self.Z = 210.0
        
        # Subscriber 생성
        # {
        #     "class": 1,
        #     "bbox": [0, 0, 0, 0]
        # }
        self.appearance_subscription = self.create_subscription(
            String,
            '/robot/command',
            self.appearance_callback,
            10
        )
        
        # Publishers 생성
        # x, y, z, battery_type을 하나의 메시지로 발행
        self.position_publisher = self.create_publisher(
            Quaternion,
            'position_topic',  # 위치와 배터리 타입을 포함하는 의미있는 토픽 이름
            10
        )
        
    def convert_uv_to_xyz(self, u_min, v_min, u_max, v_max):
        self.get_logger().info(f'Converting UV to XYZ - Input: u_min={u_min}, v_min={v_min}, u_max={u_max}, v_max={v_max}')
        
        # 중심 좌표 계산
        u = (u_min + u_max) / 2.0
        v = (v_min + v_max) / 2.0

        # 역투영을 통해 3D 카메라 좌표계로 변환 (mm 단위)
        x = (u - self.cx) * self.Z / self.fx
        y = (v - self.cy) * self.Z / self.fy
        z = self.Z

        self.get_logger().info(f'Converted coordinates - x={x:.2f}mm, y={y:.2f}mm, z={z:.2f}mm')
        return x, y, z
        
    def appearance_callback(self, msg):
        self.get_logger().info('Received message on appearance callback')
        try:
            self.get_logger().info(f'Raw message data: {msg.data}')
            data = json.loads(msg.data)
            
            self.get_logger().info('Successfully parsed JSON data')
            
            # JSON 데이터를 각 변수에 저장
            self.is_abnormal = False
            self.appearance_class = data['class']
            
            # bbox 데이터 저장
            self.bbox_x = data['bbox'][0]
            self.bbox_y = data['bbox'][1]
            self.bbox_width = data['bbox'][2]
            self.bbox_height = data['bbox'][3]

            self.get_logger().info(f'Parsed data - class: {self.appearance_class}, bbox: {data["bbox"]}')

            # UV 좌표를 XYZ로 변환
            x, y, z = self.convert_uv_to_xyz(
                self.bbox_x,
                self.bbox_y,
                self.bbox_x + self.bbox_width,
                self.bbox_y + self.bbox_height
            )

            self.get_logger().info('Starting robot coordinate conversion')
            
            # 카메라 좌표를 로봇 좌표계로 변환
            robot_x = y / 1000.0 + 0.115
            robot_y = x / 1000.0
            robot_z = -z / 1000.0 + 0.29

            self.get_logger().info(f'Robot coordinates - x={robot_x:.3f}m, y={robot_y:.3f}m, z={robot_z:.3f}m')

            # 변환된 로봇 좌표계 값을 position_msg에 할당
            position_msg = Quaternion()
            position_msg.x = robot_x
            position_msg.y = robot_y
            position_msg.z = robot_z
            position_msg.w = float(self.appearance_class)

            # 로봇 좌표계로 변환하는 종료점

            # 불량 여부 메시지 생성 및 발행
            abnormal_msg = Bool()
            abnormal_msg.data = self.is_abnormal
            
            self.position_publisher.publish(position_msg)
            self.get_logger().info('Published position message')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing JSON: {e}')
        except KeyError as e:
            self.get_logger().error(f'Missing key in JSON data: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraUVBox()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
