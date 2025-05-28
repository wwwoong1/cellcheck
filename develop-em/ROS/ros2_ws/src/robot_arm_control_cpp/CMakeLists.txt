cmake_minimum_required(VERSION 3.8)
project(robot_arm_control_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)

# robot_tf_publisher
add_executable(robot_tf_publisher src/robot_tf_pub.cpp)
ament_target_dependencies(robot_tf_publisher
  rclcpp
  tf2_ros
  geometry_msgs
)

# motor_topic
add_executable(motor_topic src/motor_topic.cpp src/motor_functions.cpp)
target_include_directories(motor_topic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_topic
  rclcpp
  tf2_ros
  geometry_msgs
  sensor_msgs
  urdf
  ament_index_cpp
)
target_include_directories(motor_topic PRIVATE ${EIGEN3_INCLUDE_DIR})

# Install
install(TARGETS
  robot_tf_publisher
  motor_topic
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
