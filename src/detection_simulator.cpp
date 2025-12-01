#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class DetectionSimulator : public rclcpp::Node
{
public:
  DetectionSimulator()
      : Node("vision_sim")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(
        50ms, std::bind(&DetectionSimulator::detection_pub_callback, this));
  }

private:
  void publish_transform(std::vector<double> translation, std::vector<double> rotation_quat, const std::string parent_frame, std::string child_frame)
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id = parent_frame;
    transform_msg.child_frame_id = child_frame;

    // set transform msg translation and rotation
    transform_msg.transform.translation.x = translation[0];
    transform_msg.transform.translation.y = translation[1];
    transform_msg.transform.translation.z = translation[2];
    transform_msg.transform.rotation.x = rotation_quat[0];
    transform_msg.transform.rotation.y = rotation_quat[1];
    transform_msg.transform.rotation.z = rotation_quat[2];
    transform_msg.transform.rotation.w = rotation_quat[3];

    // Send the transformation
    tf_broadcaster_->sendTransform(transform_msg);
  }
  void
  detection_pub_callback()
  {
    std::vector<double> translation{0.3, 0.0, 0.05};
    std::vector<double> rotation_quat{0., 1., 0., 0.};
    publish_transform(translation, rotation_quat, "base_link", "cardboard");
  }
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionSimulator>());
  rclcpp::shutdown();
  return 0;
}