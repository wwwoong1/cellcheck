#ifndef MOTOR_FUNCTIONS_HPP
#define MOTOR_FUNCTIONS_HPP

#include <vector>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/logger.hpp>

class MotorFunctions {
public:
    static void motor1_joint_states(
        const std::vector<double>& robot_positions,
        const geometry_msgs::msg::Quaternion& target_position,
        double& servo1_angle_,
        rclcpp::Logger logger
    );

    static void motor2_joint_states(
        const std::vector<double>& robot_positions,
        const geometry_msgs::msg::Quaternion& target_position,
        double servo1_angle_,
        double& servo2_angle_,
        rclcpp::Logger logger
    );

    static void motor4_joint_states(
        double target_w_,
        double& servo4_angle_,
        rclcpp::Logger logger
    );
};

#endif // MOTOR_FUNCTIONS_HPP 