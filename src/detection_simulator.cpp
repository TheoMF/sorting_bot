#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "sorting_bot/data_structures.hpp"
#include "sorting_bot/detection_simulator_parameters.hpp"
#include "sorting_bot/ros_msg_conversions.hpp"

namespace sorting_bot {

class DetectionSimulator : public rclcpp::Node {
public:
  DetectionSimulator() : Node("detection_simulator") {
    // Load ROS parameters.
    if (!load_parameters()) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't load the parameters, stopping the node.");
      throw std::runtime_error("Failed to load parameters");
    }

    for (std::string &detection_frame : params_.detection_frames) {
      // Initialize detection
      std::string parent_frame = params_.detection_frames_map.at(detection_frame).parent_frame;
      Detection detection = Detection(parent_frame, detection_frame);

      // Convert detection pose to SE3.
      std::vector<double> in_parent_pose_detection =
          params_.detection_frames_map.at(detection_frame).in_parent_pose_detection;
      Eigen::Quaterniond in_parent_rot_detection_quat =
          Eigen::Quaterniond(in_parent_pose_detection[3], in_parent_pose_detection[4], in_parent_pose_detection[5],
                             in_parent_pose_detection[6]);
      Eigen::Vector3d in_parent_translation_detection =
          Eigen::Vector3d(in_parent_pose_detection[0], in_parent_pose_detection[1], in_parent_pose_detection[2]);
      detection.in_parent_M_frame =
          pinocchio::SE3(in_parent_rot_detection_quat.toRotationMatrix(), in_parent_translation_detection);
      RCLCPP_INFO(this->get_logger(), "Simulate detection with  parent frame : %s and detection frame : %s",
                  parent_frame.c_str(), detection_frame.c_str());

      detections_.push_back(detection);
    }

    // Initialize callback timer and tf broadcaster.
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / params_.rate)),
                                     std::bind(&DetectionSimulator::detection_pub_callback, this));
  }

private:
  void detection_pub_callback() {
    for (const Detection &detection : detections_) {
      // Ensure detection contains a transform.
      if (!detection.in_parent_M_frame.has_value()) {
        RCLCPP_WARN(this->get_logger(), "Detection with parent frame : %s and detection frame : %s has no transform",
                    detection.parent_frame.c_str(), detection.frame.c_str());
        continue;
      }

      // Send transform to tf.
      geometry_msgs::msg::TransformStamped transform_msg =
          SE3_to_transform_msg(detection.in_parent_M_frame.value(), detection.parent_frame, detection.frame);
      transform_msg.header.stamp = this->get_clock()->now();
      tf_broadcaster_->sendTransform(transform_msg);
    }
  }

  bool load_parameters() {
    try {
      parameter_listener_ = std::make_shared<detection_simulator::ParamListener>(get_node_parameters_interface());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception thrown during the loading of the parameters: %s \n", e.what());
      return false;
    }
    params_ = parameter_listener_->get_params();
    return true;
  }

  // Vector of all detections to publish.
  std::vector<Detection> detections_;

  // Callback timer and tf broadcaster.
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ROS params.
  std::shared_ptr<detection_simulator::ParamListener> parameter_listener_;
  detection_simulator::Params params_;
};
} // namespace sorting_bot

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sorting_bot::DetectionSimulator>());
  rclcpp::shutdown();
  return 0;
}