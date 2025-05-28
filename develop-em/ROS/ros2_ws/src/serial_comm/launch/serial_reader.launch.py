from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('serial_comm')
    cfg_file  = os.path.join(pkg_share, 'config', 'serial_mapping.yaml')

    return LaunchDescription([
        Node(
            package='serial_comm',
            node_executable='serial_reader_node',
            name='serial_reader',
            output='screen',
            parameters=[cfg_file],
        ),
    ])
