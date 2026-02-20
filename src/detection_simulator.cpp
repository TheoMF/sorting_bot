#include "sorting_bot/detection_simulator.hpp"

namespace sorting_bot {

DetectionSimulator::DetectionSimulator() : Node("detection_simulator") {
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
    RCLCPP_INFO(this->get_logger(), "Simulate detection with parent frame : %s and detection frame : %s",
                parent_frame.c_str(), detection_frame.c_str());

    detections_.push_back(detection);
  }

  // Initialize callback timer and tf broadcaster.
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / params_.rate)),
                                   std::bind(&DetectionSimulator::detection_pub_callback, this));
}

void DetectionSimulator::detection_pub_callback() {
  for (const Detection &detection : detections_) {
    // Ensure detection contains a transform.
    if (!detection.in_parent_M_frame.has_value()) {
      RCLCPP_WARN(this->get_logger(), "Detection with parent frame : %s and detection frame : %s has no transform",
                  detection.parent_frame.c_str(), detection.frame.c_str());
      continue;
    }

    // Send transform to tf.
    geometry_msgs::msg::TransformStamped transform_msg = SE3_to_transform_msg(
        detection.in_parent_M_frame.value(), detection.parent_frame, detection.frame, this->get_clock()->now());
    tf_broadcaster_->sendTransform(transform_msg);
  }
}

bool DetectionSimulator::load_parameters() {
  try {
    parameter_listener_ = std::make_shared<detection_simulator::ParamListener>(get_node_parameters_interface());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception thrown during the loading of the parameters: %s \n", e.what());
    return false;
  }
  params_ = parameter_listener_->get_params();
  return true;
}

} // namespace sorting_bot

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sorting_bot::DetectionSimulator>());
  rclcpp::shutdown();
  return 0;
}