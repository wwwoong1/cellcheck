#include "robot_arm_control_cpp/motor_functions.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>

void MotorFunctions::motor1_joint_states(
    const std::vector<double>& robot_positions,
    const geometry_msgs::msg::Quaternion& target_position,
    double& servo1_angle_,
    rclcpp::Logger logger)
{
    // 서보1의 고정 좌표 (x=0)
    double servo1_y = robot_positions[1];
    double servo1_z = robot_positions[2];

    // 목표 좌표
    double target_y = target_position.y;
    double target_z = target_position.z;

    // Left Finger와 Right Finger의 중간 좌표 계산
    double left_finger_y = robot_positions[13];
    double left_finger_z = robot_positions[14];
    double right_finger_y = robot_positions[16];
    double right_finger_z = robot_positions[17];

    double end_y = (left_finger_y + right_finger_y) / 2.0;
    double end_z = (left_finger_z + right_finger_z) / 2.0;

    // 서보1과 서보2 사이의 거리 (첫 번째 원의 반지름)
    double r1 = std::sqrt(
        std::pow(robot_positions[4] - servo1_y, 2) +
        std::pow(robot_positions[5] - servo1_z, 2)
    );

    // 서보2와 끝좌표 사이의 거리 (두 번째 원의 반지름)
    double r2 = std::sqrt(
        std::pow(robot_positions[4] - end_y, 2) +
        std::pow(robot_positions[5] - end_z, 2)
    );

    // 두 원의 중심 사이의 거리 (y-z 평면에서)
    double d = std::sqrt(
        std::pow(target_y - servo1_y, 2) +
        std::pow(target_z - servo1_z, 2)
    );

    // 두 원이 교차하는지 확인
    if (d > r1 + r2 || d < std::abs(r1 - r2)) {
        RCLCPP_ERROR(logger, "No intersection points found between the circles");
        return;
    }

    // 두 원의 교점 계산 (y-z 평면에서)
    double a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    double h = std::sqrt(r1 * r1 - a * a);

    // 중간점 계산
    double P2y = servo1_y + a * (target_y - servo1_y) / d;
    double P2z = servo1_z + a * (target_z - servo1_z) / d;

    // 교점 계산 (y-z 평면에서)
    double y3_1 = P2y + h * (target_z - servo1_z) / d;
    double z3_1 = P2z - h * (target_y - servo1_y) / d;

    double y3_2 = P2y - h * (target_z - servo1_z) / d;
    double z3_2 = P2z + h * (target_y - servo1_y) / d;

    // y좌표가 더 큰 교점 선택 (y좌표가 같다면 z좌표가 작은 쪽 선택)
    double selected_y, selected_z;
    if (y3_1 > y3_2 || (y3_1 == y3_2 && z3_1 < z3_2)) {
        selected_y = y3_1;
        selected_z = z3_1;
    } else {
        selected_y = y3_2;
        selected_z = z3_2;
    }

    // 선택된 교점에 대한 서보1의 각도 계산 (y-z 평면에서)
    double angle = std::atan2(selected_z - servo1_z, selected_y - servo1_y);
    // 각도 범위 조정 (0 ~ 2π)
    if (angle < 0) {
        angle += 2 * M_PI;
    }

    // 서보1 각도 업데이트 (π 만큼 회전)
    servo1_angle_ = angle - M_PI;
}

void MotorFunctions::motor2_joint_states(
    const std::vector<double>& robot_positions,
    const geometry_msgs::msg::Quaternion& target_position,
    double servo1_angle_,
    double& servo2_angle_,
    rclcpp::Logger logger)
{
    // 서보2의 고정 좌표
    double servo2_y = robot_positions[4];
    double servo2_z = robot_positions[5];

    // 목표 좌표
    double target_y = target_position.y;
    double target_z = target_position.z;

    // 서보2에서 목표좌표까지의 직선의 각도 계산 (y-z 평면)
    double world_angle = std::atan2(target_z - servo2_z, target_y - servo2_y);
    // 각도 범위 조정 (0 ~ 2π)
    if (world_angle < 0) {
        RCLCPP_ERROR(logger, "world_angle: %f", world_angle);
        world_angle += 2 * M_PI;
    }

    // 서보1의 각도는 이미 라디안 단위
    // 서보2의 실제 각도 계산 (30도를 라디안으로 변환하여 사용)
    servo2_angle_ = -(world_angle - M_PI - servo1_angle_) + (30.0 * M_PI / 180.0);
}

void MotorFunctions::motor4_joint_states(
    double target_w_,
    double& servo4_angle_,
    rclcpp::Logger logger)
{
    // target_w_ 값에 따라 servo4 각도 설정
    double angle = 0.0;  // 기본값 0도

    switch (static_cast<int>(target_w_)) {
        case 0:  // AAA
            angle = 0.6;
            break;
        case 1:  // AA
            angle = 0.46;
            break;
        case 2:  // C
            angle = 0.475;
            break;
        case 3:  // D
            angle = 0.0;
            break;
        default:
            angle = 0.0;
            // 예외 처리
            RCLCPP_ERROR(logger, "battery_type: %d", static_cast<int>(target_w_));
            break;
    }

    // 각도를 라디안으로 변환하여 저장
    servo4_angle_ = angle;
} 