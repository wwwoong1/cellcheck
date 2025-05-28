from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('jetson_ec2_mqtt_comm')
    return LaunchDescription([
        Node(
            package='jetson_ec2_mqtt_comm',
            node_executable='telemetry_node',
            name='telemetry_node',
            output='screen',
            parameters=[{
                'publish_frequency': 0.2  # 5초에 1번, 필요에 따라 조정
            }],
        ),
        Node(
            package='jetson_ec2_mqtt_comm',
            node_executable='mqtt_bridge_node',
            name='mqtt_bridge_node',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'mqtt_bridge_params.yaml')],
        ),
    ])
