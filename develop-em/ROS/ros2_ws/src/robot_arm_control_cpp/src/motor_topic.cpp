#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>
#include <chrono>
#include <thread>
#include "robot_arm_control_cpp/motor_functions.hpp"

// 0 : AAA : 35도
// 1 : AA : 25도
// 2 : C : 25도
// 3 : D : 0도

class MotorTopicSubscriber : public rclcpp::Node
{
public:
    MotorTopicSubscriber() : Node("motor_topic_subscriber")
    {
        // 초기 목표 좌표 설정
        target_position_ = geometry_msgs::msg::Quaternion();
        target_position_.x = 0.0;
        target_position_.y = -0.003;
        target_position_.z = 0.221;
        target_position_.w = -1.0;
        target_w_ = -1.0;  // w 값 초기화

        // 초기 서보모터 각도 설정 (라디안)
        servo1_angle_ = 150.0 * M_PI / 180.0;  // base_to_servo1: 150도
        servo2_angle_ = 180.0 * M_PI / 180.0;  // upper_arm_to_servo2: 180도
        servo3_angle_ = 90.0 * M_PI / 180.0;   // forearm_to_servo3: 90도
        servo4_angle_ = 0.0 * M_PI / 180.0;    // gripper_control: 0도

        // current_joint_states_ 초기화
        update_joint_states();

        // 토픽 구독 상태 변수 초기화
        position_topic_received_ = false;
        robot_positions_received_ = false;

        // position_topic 구독
        position_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "position_topic", 10,
            std::bind(&MotorTopicSubscriber::position_callback, this, std::placeholders::_1));

        // robot_positions 토픽 구독 (servo1 → servo2 → servo3 → servo4 → left_finger → right_finger)
        robot_positions_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "robot_positions", 10,
            std::bind(&MotorTopicSubscriber::robot_positions_callback, this, std::placeholders::_1));

        // joint_states 퍼블리셔 생성
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        arduino_joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("arduino_joint_states", 10);

        // joint_states 발행을 위한 타이머 (10Hz)
        joint_states_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorTopicSubscriber::publish_joint_states, this));

        // 상태 출력용 타이머 (1Hz)
        // log_timer_ = this->create_wall_timer(
        //     std::chrono::seconds(1),
        //     std::bind(&MotorTopicSubscriber::print_status, this));
    }

private:
    // position_topic 콜백 함수
    void position_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
    {
        try {
            position_topic_received_ = true;
            target_position_ = *msg;
            target_w_ = msg->w;  // w 값 별도 저장

            // target_position_ 값 출력
            RCLCPP_INFO(this->get_logger(), "Target Position - x: %.3f, y: %.3f, z: %.3f, w: %.3f",
                target_position_.x, target_position_.y, target_position_.z, target_position_.w);

            if (!robot_positions_received_) {
                RCLCPP_WARN(this->get_logger(), "로봇 위치 정보가 아직 수신되지 않았습니다. 계산을 건너뜁니다.");
                return;
            }

            motor1_joint_states();
            motor2_joint_states();
            motor4_joint_states();
            // joint_states 출력
            RCLCPP_INFO(this->get_logger(), "모터1 관절 상태 계산 완료: %.3f도", servo1_angle_ * 180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), "모터2 관절 상태 계산 완료: %.3f도", servo2_angle_ * 180.0 / M_PI);

            arduino_joint_states();  // 현재 joint_states 값을 한 번 발행
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in position_callback: %s", e.what());
            // 에러 발생 시 이전 저장값 유지
        }
    }

    // robot_positions 토픽 콜백 함수
    void robot_positions_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        try {
            if (msg->data.size() != 18) {
                RCLCPP_ERROR(this->get_logger(), "Invalid robot_positions data size: %zu (expected: 18)", msg->data.size());
                return;
            }

            robot_positions_received_ = true;
            robot_positions_ = msg->data;

            if (!position_topic_received_) {
                RCLCPP_WARN(this->get_logger(), "목표 위치 정보가 아직 수신되지 않았습니다. 계산을 건너뜁니다.");
                return;
            }

            // 각도 계산 실행
            motor1_joint_states();
            motor2_joint_states();
            motor4_joint_states();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in robot_positions_callback: %s", e.what());
            // 에러 발생 시 이전 저장값 유지
        }
    }

    // joint_states 발행 함수 (10Hz)
    void publish_joint_states()
    {
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();
        
        message.name = {
            "base_to_servo1",
            "upper_arm_to_servo2",
            "forearm_to_servo3",
            "gripper_control"
        };

        // 토픽 구독 상태 확인
        // if (!position_topic_received_ || !robot_positions_received_) {
        //     RCLCPP_WARN(this->get_logger(),
        //         "Using initial values. position_topic: %s, robot_positions: %s",
        //         position_topic_received_ ? "received" : "not received",
        //         robot_positions_received_ ? "received" : "not received");
        // }

        message.position = current_joint_states_;
        joint_states_pub_->publish(message);
    }

    // 상태 출력 함수 (1Hz)
    void print_status()
    {
        RCLCPP_INFO(this->get_logger(), "=== Robot Arm Status ===");
        RCLCPP_INFO(this->get_logger(), "Target Position (meters):");
        RCLCPP_INFO(this->get_logger(), "  x: %.3f, y: %.3f, z: %.3f", 
            target_position_.x, target_position_.y, target_position_.z);
        RCLCPP_INFO(this->get_logger(), "Current Joint States (degrees):");
        RCLCPP_INFO(this->get_logger(), "  base_to_servo1: %.3f", servo1_angle_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  upper_arm_to_servo2: %.3f", servo2_angle_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  forearm_to_servo3: %.3f", servo3_angle_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  gripper_control: %.3f", servo4_angle_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    }

    void motor1_joint_states()
    {
        MotorFunctions::motor1_joint_states(
            robot_positions_,
            target_position_,
            servo1_angle_,
            this->get_logger()
        );
        update_joint_states();
    }

    void motor2_joint_states()
    {
        MotorFunctions::motor2_joint_states(
            robot_positions_,
            target_position_,
            servo1_angle_,
            servo2_angle_,
            this->get_logger()
        );
        update_joint_states();
    }

    void motor4_joint_states()
    {
        MotorFunctions::motor4_joint_states(
            target_w_,
            servo4_angle_,
            this->get_logger()
        );
        update_joint_states();
    }

    void arduino_joint_states()
    {
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();
        
        message.name = {
            "base_to_servo1",
            "upper_arm_to_servo2",
            "forearm_to_servo3",
            "gripper_control"
        };

        // 라디안을 도로 변환 (radians * 180 / π)
        std::vector<double> positions = {
            servo1_angle_ * 180.0 / M_PI,      // base_to_servo1
            servo2_angle_ * 180.0 / M_PI,      // upper_arm_to_servo2
            servo3_angle_ * 180.0 / M_PI,      // forearm_to_servo3
            servo4_angle_ * 180.0 / M_PI       // gripper_control
        };
        
        message.position = positions;
        
        arduino_joint_states_pub_->publish(message);
    }

    // joint_states 업데이트 함수
    void update_joint_states()
    {
        current_joint_states_ = {
            servo1_angle_,
            servo2_angle_,
            servo3_angle_,
            servo4_angle_
        };
    }

    // 구독자
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr robot_positions_sub_;

    // 퍼블리셔
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arduino_joint_states_pub_;
    // 타이머
    rclcpp::TimerBase::SharedPtr joint_states_timer_;
    // rclcpp::TimerBase::SharedPtr log_timer_;

    // 토픽 구독 상태
    bool position_topic_received_;
    bool robot_positions_received_;

    // 저장 데이터
    geometry_msgs::msg::Quaternion target_position_;
    double target_w_;  // w 값을 저장할 새로운 변수
    std::vector<double> robot_positions_;
    std::vector<double> current_joint_states_;

    // 서보모터 각도 (라디안)
    double servo1_angle_;
    double servo2_angle_;
    double servo3_angle_;
    double servo4_angle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorTopicSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
