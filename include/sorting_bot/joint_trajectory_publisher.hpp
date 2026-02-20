#ifndef SORTING_BOT_JOINT_TRAJECTORY_PUBLISHER_HPP_
#define SORTING_BOT_JOINT_TRAJECTORY_PUBLISHER_HPP_

#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "sorting_bot/planner_manager.hpp"

namespace sorting_bot {

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using MultiArrayDimension = std_msgs::msg::MultiArrayDimension;
using JointState = sensor_msgs::msg::JointState;
using Odometry = nav_msgs::msg::Odometry;
using String = std_msgs::msg::String;

class JointTrajectoryPublisher : public rclcpp::Node {
public:
  JointTrajectoryPublisher();

private:
  bool load_parameters();

  void joint_states_callback(const JointState &msg);

  void robot_description_callback(const String &msg);

  void
  nav_goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::SharedPtr &goal_handle) const;

  void nav_result_callback(const rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::WrappedResult &result);

  void odom_callback(const Odometry &msg);

  void publish_nav_goal(const std::vector<Eigen::VectorXd> &base_poses);

  void send_gripper_pose_msg(const double &gripper_pose) const;

  void send_joint_trajectory_msg(const Eigen::VectorXd &q_ref, const Eigen::VectorXd &q_dot_ref) const;

  void handle_nav_goal_publication();

  void update_traj_references();

  void do_actions();

  void update_integrated_q_err(const Eigen::VectorXd &q_err);

  Eigen::VectorXd get_q_ref(const Eigen::VectorXd &q_err) const;

  void joint_traj_pub_callback();

  void detections_update_callback();

  // ROS params.
  std::shared_ptr<joint_trajectory_publisher::ParamListener> parameter_listener_;
  joint_trajectory_publisher::Params params_;

  // ROS publishers, subscribers and timers.
  rclcpp::TimerBase::SharedPtr joint_traj_pub_timer_, detections_update_timer_;
  rclcpp::Publisher<JointTrajectory>::SharedPtr joint_trajectory_publisher_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr so_101_gripper_publisher_;
  rclcpp::Publisher<JointTrajectory>::SharedPtr lekiwi_gripper_publisher_;
  rclcpp::Subscription<JointState>::SharedPtr joint_states_subscriber_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<String>::SharedPtr robot_description_subscriber_;

  // Navigation attributes.
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr navigation_action_client_;
  rclcpp_action::ResultCode nav_result_ = rclcpp_action::ResultCode::UNKNOWN;

  // tf attributes.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Non-ROS related attributes.
  std::map<std::string, Detection> detection_map_;
  PlannerManager planner_manager_;
  ActionType last_action_ = NONE;
  std::mutex current_q_mutex_;
  int nq_;
  Eigen::VectorXd current_q_, q_traj_, q_dot_traj_, integrated_q_err_, last_q_, base_pose_ = Eigen::VectorXd::Zero(3);
  std::vector<Eigen::VectorXd> last_sent_base_waypoints = {Eigen::Vector3d(-10., -10., 0.)};
  bool over_traj_total_duration_ = false, first_joint_traj_pub_callback_iter_ = true, traj_ready_ = false;
  std::atomic<bool> joint_states_callback_ready_ = false, robot_description_ready_ = false;
};

} // namespace sorting_bot

#endif