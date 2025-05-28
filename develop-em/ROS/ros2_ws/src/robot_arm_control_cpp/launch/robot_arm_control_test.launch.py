from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Arduino Controller 노드
    arduino_controller_node = Node(
        package='arduino_interface',
        node_executable='arduino_controller_node',
        name='arduino_controller_node',
        output='screen',
    )

    # Camera Point 노드
    camera_point_node = Node(
        package='robot_arm_target',
        node_executable='camera_point',
        name='camera_point',
        output='screen',
    )

    return LaunchDescription([
        arduino_controller_node,
        camera_point_node,
    ]) 