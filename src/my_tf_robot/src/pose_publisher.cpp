#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

class RobotPosePublisher : public rclcpp::Node {
public:
  RobotPosePublisher() : Node("robot_pose_publisher"), theta_(0.0) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RobotPosePublisher::publish_pose, this));
  }

private:
  void publish_pose() {
    double radius = 1.0;  // 圆的半径
    double x = radius * std::cos(theta_);
    double y = radius * std::sin(theta_);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    t.transform.rotation.w = std::cos(theta_ / 2);
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = std::sin(theta_ / 2);

    tf_broadcaster_->sendTransform(t);
    theta_ += 0.05;  // 增加角度，控制转速
    if (theta_ > 2 * M_PI) theta_ -= 2 * M_PI;
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double theta_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPosePublisher>());
  rclcpp::shutdown();
  return 0;
}