#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # ─ battery_system 패키지 노드
    ld.add_action(Node(
        package='battery_system',
        node_executable='system_manager_node',
        name='system_manager_node',
        output='screen',
    ))
    ld.add_action(Node(
        package='battery_system',
        node_executable='zone_manager_node',
        name='zone_manager_node',
        output='screen',
    ))

    # ─ arduino_interface 패키지 노드
    ld.add_action(Node(
        package='arduino_interface',
        node_executable='arduino_controller_node',
        name='arduino_controller_node',
        output='screen',
    ))

    # ─ jetson_ec2_mqtt_comm 패키지 노드
    pkg_mqtt = get_package_share_directory('jetson_ec2_mqtt_comm')
    ca_path = os.path.join(pkg_mqtt,'resource','cert','ca.crt')
    mqtt_yaml_path = os.path.join(pkg_mqtt, 'config', 'mqtt_bridge_params.yaml')
    ld.add_action(Node(
        package='jetson_ec2_mqtt_comm',
        node_executable='telemetry_node',
        name='telemetry_node',
        output='screen',
        parameters=[{ 'publish_frequency': 0.2 }],
    ))
    ld.add_action(Node(
        package='jetson_ec2_mqtt_comm',
        node_executable='mqtt_bridge_node',
        name='mqtt_bridge_node',
        output='screen',
        parameters=[mqtt_yaml_path,{'mqtt.cafile':ca_path}],
    ))
    ld.add_action(Node(
        package='jetson_ec2_mqtt_comm',
        node_executable='battery_calc_node',
        name='battery_calc_node',
        output='screen',
    ))

    # ─ image_comm 패키지 노드
    pkg_image = get_package_share_directory('image_comm')
    ld.add_action(Node(
        package='image_comm',
        node_executable='ws_node',
        name='ws_node',
        output='screen',
        parameters=[os.path.join(pkg_image, 'config', 'params.yaml')],    
    ))
    ld.add_action(Node(
        package='image_comm',
        node_executable='capture_node',
        name='caputre_node',
        output='screen',
    ))
    # ─ serial_comm 패키지 노드
    pkg_serial = get_package_share_directory('serial_comm')
    ld.add_action(Node(
        package='serial_comm',
        node_executable='serial_reader_node',
        name='serial_reader',
        output='screen',
        parameters=[os.path.join(pkg_serial, 'config', 'serial_mapping.yaml')],
    ))

    # ─ robot_arm_control 관련 노드들
    robot_arm_description_pkg = get_package_share_directory('robot_arm_description')
    robot_arm_control_pkg = get_package_share_directory('robot_arm_control_cpp')
    
    # URDF 파일 경로
    urdf_file_path = os.path.join(robot_arm_description_pkg, 'urdf', 'robot_arm_simple.urdf')
    
    # Robot state publisher 노드
    ld.add_action(Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file_path],
    ))

    # Robot TF Publisher 노드
    ld.add_action(Node(
        package='robot_arm_control_cpp',
        node_executable='robot_tf_publisher',
        name='robot_tf_publisher',
        output='screen',
    ))

    # Camera UV Box 노드
    ld.add_action(Node(
        package='robot_arm_target',
        node_executable='camera_uvbox',
        name='camera_uvbox',
        output='screen',
    ))

    # Motor Topic Subscriber 노드
    ld.add_action(Node(
        package='robot_arm_control_cpp',
        node_executable='motor_topic',
        name='motor_topic_subscriber',
        output='screen',
    ))

    # RViz 설정 파일 경로
    # rviz_config = os.path.join(robot_arm_description_pkg, 'config', 'robot_arm.rviz')

    # # RViz2 노드
    # ld.add_action(Node(
    # package='rviz2',
    # node_executable='rviz2',
    # name='rviz2',
    # output='screen',
    # arguments=['-d', rviz_config],
    # ))

    return ld
