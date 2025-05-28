# 로봇 시각화 노드 런치파일
# 시각화만 있고 초기값이 없어서 joint_state토픽을 발행하는 다른 노드와 같이 사용 필요

import os
import tempfile
import base64
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 패키지 경로 얻기
    package_name = 'robot_arm_description'
    package_path = get_package_share_directory(package_name)
    
    # URDF 파일 경로
    urdf_file_path = os.path.join(package_path, 'urdf', 'robot_arm_simple.urdf')
    
    # RViz 설정 파일 경로
    rviz_config = os.path.join(package_path, 'config', 'robot_arm.rviz')

    # Robot state publisher 노드 - URDF 파일 경로를 직접 전달
    robot_state_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file_path],
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
        rviz_node,
    ]) 