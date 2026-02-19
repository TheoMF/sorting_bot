#ifndef SORTING_BOT_ROS_MSG_CONVERSIONS_HPP_
#define SORTING_BOT_ROS_MSG_CONVERSIONS_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "rclcpp/time.hpp"

namespace sorting_bot {

using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Transform = geometry_msgs::msg::Transform;
using TransformStamped = geometry_msgs::msg::TransformStamped;

inline pinocchio::SE3 transform_msg_to_SE3(const TransformStamped &transform_msg) {
  Transform transform = transform_msg.transform;
  Eigen::Quaterniond quat(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  Eigen::Vector3d translation(transform.translation.x, transform.translation.y, transform.translation.z);
  return pinocchio::SE3(quat, translation);
};

inline TransformStamped SE3_to_transform_msg(const pinocchio::SE3 &transform, const std::string &parent_frame,
                                             const std::string &child_frame) {
  // Set msg frames.
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;

  // Set transform msg translation and rotation.
  transform_msg.transform.translation.x = transform.translation().x();
  transform_msg.transform.translation.y = transform.translation().y();
  transform_msg.transform.translation.z = transform.translation().z();
  Eigen::Quaterniond q(transform.rotation());
  transform_msg.transform.rotation.x = q.x();
  transform_msg.transform.rotation.y = q.y();
  transform_msg.transform.rotation.z = q.z();
  transform_msg.transform.rotation.w = q.w();

  return transform_msg;
};

inline Eigen::VectorXd pose_msg_to_base_pose(const Pose &msg) {
  pinocchio::SE3::Quaternion quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  double yaw = quat.toRotationMatrix().eulerAngles(0, 1, 2)[2];
  std::vector<double> pose_vec = {msg.position.x, msg.position.y, yaw};
  return Eigen::Map<Eigen::VectorXd>(pose_vec.data(), pose_vec.size());
}

inline PoseStamped base_pose_to_pose_msg(const Eigen::VectorXd &base_pose, const std::string world_frame,
                                         const rclcpp::Time &time) {
  PoseStamped pose_msg;
  pose_msg.header.stamp = time;
  pose_msg.header.frame_id = world_frame;
  pose_msg.pose.position.x = base_pose[0];
  pose_msg.pose.position.y = base_pose[1];
  pose_msg.pose.position.z = 0.0;
  Eigen::AngleAxisd angle_axis_rot = Eigen::AngleAxisd(base_pose[2], Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat_rot = Eigen::Quaterniond(angle_axis_rot.toRotationMatrix());
  pose_msg.pose.orientation.x = quat_rot.x();
  pose_msg.pose.orientation.y = quat_rot.y();
  pose_msg.pose.orientation.z = quat_rot.z();
  pose_msg.pose.orientation.w = quat_rot.w();
  return pose_msg;
}
} // namespace sorting_bot
#endif