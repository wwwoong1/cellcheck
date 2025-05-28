# ws_only.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('image_comm')
    param_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='image_comm',
            node_executable='ws_node',
            name='ws_node',
            output='screen',
            parameters=[param_file],
        ),
    ])
