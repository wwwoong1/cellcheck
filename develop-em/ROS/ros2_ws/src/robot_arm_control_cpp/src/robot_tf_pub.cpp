// TF좌표 토픽 발행 노드

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>

using namespace std::chrono_literals;

class RobotTFPublisher : public rclcpp::Node
{
public:
    RobotTFPublisher() : Node("robot_tf_publisher")
    {
        // TF 버퍼와 리스너 초기화
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 통합된 좌표 토픽 퍼블리셔 초기화
        positions_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("robot_positions", 10);

        // 타이머 설정 (0.1초마다 실행)
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&RobotTFPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        try
        {
            // 각 서보모터의 TF 정보 가져오기
            auto servo1_tf = tf_buffer_->lookupTransform(
                "world", "servo1_link", tf2::TimePointZero);
            auto servo2_tf = tf_buffer_->lookupTransform(
                "world", "servo2_link", tf2::TimePointZero);
            auto servo3_tf = tf_buffer_->lookupTransform(
                "world", "servo3_link", tf2::TimePointZero);
            auto servo4_tf = tf_buffer_->lookupTransform(
                "world", "servo4_link", tf2::TimePointZero);

            // 집게 끝부분의 TF 정보 가져오기
            auto left_finger_tf = tf_buffer_->lookupTransform(
                "world", "left_finger", tf2::TimePointZero);
            auto right_finger_tf = tf_buffer_->lookupTransform(
                "world", "right_finger", tf2::TimePointZero);

            // 통합된 메시지 생성
            auto positions_msg = std_msgs::msg::Float64MultiArray();
            positions_msg.data = {
                servo1_tf.transform.translation.x, servo1_tf.transform.translation.y, servo1_tf.transform.translation.z,
                servo2_tf.transform.translation.x, servo2_tf.transform.translation.y, servo2_tf.transform.translation.z,
                servo3_tf.transform.translation.x, servo3_tf.transform.translation.y, servo3_tf.transform.translation.z,
                servo4_tf.transform.translation.x, servo4_tf.transform.translation.y, servo4_tf.transform.translation.z,
                left_finger_tf.transform.translation.x, left_finger_tf.transform.translation.y, left_finger_tf.transform.translation.z,
                right_finger_tf.transform.translation.x, right_finger_tf.transform.translation.y, right_finger_tf.transform.translation.z
            };

            // 메시지 발행
            positions_pub_->publish(positions_msg);

            // 콘솔 출력 (디버깅용)
            // RCLCPP_INFO(this->get_logger(), "Published all positions in a single message");
            // RCLCPP_INFO(this->get_logger(), "Servo1: [%.3f, %.3f, %.3f]", 
            //     positions_msg.data[0], positions_msg.data[1], positions_msg.data[2]);
            // RCLCPP_INFO(this->get_logger(), "Servo2: [%.3f, %.3f, %.3f]", 
            //     positions_msg.data[3], positions_msg.data[4], positions_msg.data[5]);
            // RCLCPP_INFO(this->get_logger(), "Servo3: [%.3f, %.3f, %.3f]", 
            //     positions_msg.data[6], positions_msg.data[7], positions_msg.data[8]);
            // RCLCPP_INFO(this->get_logger(), "Servo4: [%.3f, %.3f, %.3f]", 
            //     positions_msg.data[9], positions_msg.data[10], positions_msg.data[11]);
            // RCLCPP_INFO(this->get_logger(), "Left Finger: [%.3f, %.3f, %.3f]", 
            //     positions_msg.data[12], positions_msg.data[13], positions_msg.data[14]);
            // RCLCPP_INFO(this->get_logger(), "Right Finger: [%.3f, %.3f, %.3f]", 
            //     positions_msg.data[15], positions_msg.data[16], positions_msg.data[17]);
            // RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 통합된 좌표 토픽 퍼블리셔
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr positions_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotTFPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
