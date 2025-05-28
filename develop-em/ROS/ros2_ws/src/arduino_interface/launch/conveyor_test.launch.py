from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arduino_interface',
            executable='arduino_conveyor_node',
            name='arduino_conveyor_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},  # 아두이노 포트에 맞게 수정
                {'baud_rate': 9600}
            ]
        ),
        Node(
            package='system_manager',
            executable='system_manager_node',
            name='system_manager_node',
            output='screen'
        )
    ])