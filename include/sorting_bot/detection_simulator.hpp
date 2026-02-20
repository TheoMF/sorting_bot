#ifndef SORTING_BOT_DETECTION_SIMULATOR_HPP_
#define SORTING_BOT_DETECTION_SIMULATOR_HPP_

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
  DetectionSimulator();

private:
  void detection_pub_callback();

  bool load_parameters();

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

#endif