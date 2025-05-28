#!/bin/bash

# ROS 2 Dashing 관련 패키지 설치
echo "Installing ROS 2 Dashing packages..."

# 기본 ROS 2 패키지
sudo apt install -y \
    ros-dashing-rclcpp \
    ros-dashing-rclpy \
    ros-dashing-tf2-ros \
    ros-dashing-geometry-msgs \
    ros-dashing-sensor-msgs \
    ros-dashing-urdf \
    ros-dashing-ament-index-cpp \
    ros-dashing-ament-cmake \
    ros-dashing-ament-python \
    ros-dashing-rosidl-default-generators \
    ros-dashing-rosidl-default-runtime

# 개발 도구
sudo apt install -y \
    libeigen3-dev \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-runner \
    python3-setuptools \
    python3-vcstool \
    python3-rosdep

# Python 개발 도구
pip3 install \
    ament-copyright \
    ament-flake8 \
    ament-pep257 \
    ament-package \
    ament-cmake \
    ament-cmake-python

# rosdep 초기화 및 업데이트
echo "Initializing rosdep..."
sudo rosdep init
rosdep update

# 환경 설정
echo "Setting up ROS 2 environment..."
echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "ROS 2 Dashing installation completed!" 