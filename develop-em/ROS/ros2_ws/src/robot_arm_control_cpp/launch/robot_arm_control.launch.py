from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 경로 얻기
    robot_arm_description_pkg = get_package_share_directory('robot_arm_description')
    robot_arm_control_pkg = get_package_share_directory('robot_arm_control_cpp')
    
    # URDF 파일 경로
    urdf_file_path = os.path.join(robot_arm_description_pkg, 'urdf', 'robot_arm_simple.urdf')
    
    # RViz 설정 파일 경로
    # rviz_config = os.path.join(robot_arm_description_pkg, 'config', 'robot_arm.rviz')

    # # RViz2 노드
    # rviz_node = Node(
    #     package='rviz2',
    #     node_executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config],
    # )

    # Arduino Controller 노드
    arduino_controller_node = Node(
        package='arduino_interface',
        node_executable='arduino_controller_node',
        name='arduino_controller_node',
        output='screen',
    )

    # Robot state publisher 노드
    robot_state_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file_path],
    )

    # Robot TF Publisher 노드
    robot_tf_publisher_node = Node(
        package='robot_arm_control_cpp',
        node_executable='robot_tf_publisher',
        name='robot_tf_publisher',
        output='screen',
    )

    # Camera UV Box 노드
    camera_uvbox_node = Node(
        package='robot_arm_target',
        node_executable='camera_uvbox',
        name='camera_uvbox',
        output='screen',
    )

    # Motor Topic Subscriber 노드
    motor_topic_node = Node(
        package='robot_arm_control_cpp',
        node_executable='motor_topic',
        name='motor_topic_subscriber',
        output='screen',
    )

    return LaunchDescription([
        # rviz_node,
        arduino_controller_node,
        robot_state_publisher,
        robot_tf_publisher_node,
        camera_uvbox_node,
        motor_topic_node,
    ]) 