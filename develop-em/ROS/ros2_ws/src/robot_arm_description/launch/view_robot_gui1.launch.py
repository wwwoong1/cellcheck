# 로봇 시각화 노드 런치파일
# 초기값을 주고 joint_state_publisher_gui를 사용하여 gui에서 조절 가능

import os
import tempfile
import base64
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def deg_to_rad(degrees):
    """각도를 라디안으로 변환하는 함수"""
    return degrees * math.pi / 180.0


def generate_launch_description():
    # 패키지 경로 얻기
    package_name = 'robot_arm_description'
    package_path = get_package_share_directory(package_name)
    
    # URDF 파일 경로
    urdf_file_path = os.path.join(package_path, 'urdf', 'robot_arm_simple.urdf')
    
    # RViz 설정 파일 경로
    rviz_config = os.path.join(package_path, 'config', 'robot_arm.rviz')

    initial_joint_states = {
        'base_to_servo1': deg_to_rad(150),      # 150도
        'upper_arm_to_servo2': deg_to_rad(180),  # 180도
        'forearm_to_servo3': deg_to_rad(90),    # 90도
        'gripper_control': deg_to_rad(0),        # 0도
    }
    
    # Robot state publisher 노드 - URDF 파일 경로를 직접 전달
    robot_state_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file_path],
    )
    
    # # Joint state publisher (GUI 없는 버전) - 관절을 제어
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     node_executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    # )

    # Joint state publisher (GUI 포함된 슬라이더 버전)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        node_executable='joint_state_publisher_gui',  # 최신 ROS2에서는 'executable' 사용
        name='joint_state_publisher',
        output='screen',
        parameters=[{'zeros': initial_joint_states}],
    )
    
    # RViz2 노드 - 로봇 시각화
    rviz_node = Node(
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
    ]) 