#include <vector>
#include <atomic>
#include <mutex>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sorting_bot/motion_planner.hpp"

enum StateMachine
{
  SEARCHING_OBJECTS,
  GOING_TO_GRASP_POSE,
  GRASPING,
  SEARCHING_BOX,
  PLACING
};
enum ActionType
{
  NONE,
  MOVE_JAW,
  SET_MOVING_BASE_CONFIGURATION,
  MOVE_BASE,
  WAIT,
  FOLLOW_TRAJ,
  SEARCH_OBJECT,
  SEARCH_BOX
};

struct DetectionStatus
{
  std::string camera_frame, frame;
  bool is_in_fov;
  std::optional<builtin_interfaces::msg::Time> last_stamp;
  std::optional<pinocchio::SE3> in_base_M_frame;

  DetectionStatus()
  {
    camera_frame = "";
    frame = "";
    is_in_fov = false;
    last_stamp = std::nullopt;
    in_base_M_frame = std::nullopt;
  }

  DetectionStatus(const std::string &_camera_frame, const std::string &_frame)
      : camera_frame(_camera_frame), frame(_frame)
  {
    is_in_fov = false;
    last_stamp = std::nullopt;
    in_base_M_frame = std::nullopt;
  }
};

pinocchio::SE3 transform_msg_to_SE3(const geometry_msgs::msg::Transform &transform)
{
  Eigen::Quaterniond q(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  Eigen::Vector3d t(transform.translation.x, transform.translation.y, transform.translation.z);
  pinocchio::SE3 se3(q, t);
  return se3;
}

class PlannerManager
{
public:
  PlannerManager() {}

  void initialize(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
                  const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster,
                  const std::string &urdf,
                  joint_trajectory_publisher::Params &params)
  {
    // Set tf attributes.
    tf_buffer_ = tf_buffer;
    tf_broadcaster_ = tf_broadcaster;

    // Set ROS parameters related attributes.
    nq_ = params.nq;
    objects_frame_ = params.objects_frame;
    joints_des_precision_ = params.joints_des_precision;
    robot_name_ = params.robot_name;
    world_frame_ = params.world_frame;
    base_frame_ = params.base_frame;
    ee_frame_name_ = params.ee_frame_name;
    box_dist_while_placing_ = params.box_dist_while_placing;
    wait_duration_before_vision_action_ = params.wait_duration_before_vision_action;
    q_base_moving_ = Eigen::Map<Eigen::VectorXd>(params.q_base_moving.data(), params.q_base_moving.size());
    state_update_period_ = 1.0 / params.rate;
    initialize_in_box_M_compartments(params.box_width, params.box_length);
    motion_planner.initialize(urdf, params);

    // Set in_object_translation_grasp_map.
    for (std::string &object_frame : params.objects_frame)
    {
      std::vector<double> in_grasp_translation_object_to_grasp_vec = params.objects_frame_map.at(object_frame).in_grasp_translation_object_to_grasp;
      Eigen::VectorXd in_grasp_translation_object_to_grasp = Eigen::Map<Eigen::Vector3d>(
          in_grasp_translation_object_to_grasp.data(), in_grasp_translation_object_to_grasp.size());
      in_grasp_translation_object_to_grasp_map_.insert({object_frame, in_grasp_translation_object_to_grasp});
      objects_to_grasp_from_top_map_.insert({object_frame, params.objects_frame_map.at(object_frame).grasp_from_top});
    }

    // Set qs_searching_objects.
    std::vector<std::vector<double>> qs_searching_objects = {params.q_searching_object, params.q_searching_object, params.q_searching_object};
    qs_searching_objects[1][0] -= params.q0_shift_q_searching_object;
    qs_searching_objects[2][0] += params.q0_shift_q_searching_object;
    for (std::vector<double> &q_searching_objects : qs_searching_objects)
      qs_searching_objects_.push_back(Eigen::Map<Eigen::VectorXd>(q_searching_objects.data(), q_searching_objects.size()));
    qs_searching_objects_.push_back(q_base_moving_);
  }

  void
  initialize_in_box_M_compartments(const double &box_width, const double &box_length)
  {
    Eigen::Quaterniond x_axis_rot_pi_quat(0., 1., 0., 0.);
    for (double length_idx = -1.; length_idx == -1. || length_idx == 1.; length_idx += 2.)
    {
      for (double width_idx = -1.; width_idx == -1. || width_idx == 1.; width_idx += 2.)
      {
        Eigen::Vector3d box_to_compartment_trans(length_idx * box_length / 4.0,
                                                 0.18,
                                                 -box_width / 2.0 + width_idx * box_width / 4.0);
        pinocchio::SE3 in_box_M_compartment(x_axis_rot_pi_quat, box_to_compartment_trans);
        in_box_M_compartments_.push_back(in_box_M_compartment);
      }
    }
  }

  pinocchio::SE3 get_most_recent_transform(const std::string &parent_frame, const std::string &child_frame)
  {
    geometry_msgs::msg::TransformStamped stamped_transform = tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
    return transform_msg_to_SE3(stamped_transform.transform);
  }

  void update_det_status_map(const std::map<std::string, DetectionStatus> &det_status_map)
  {
    std::lock_guard<std::mutex> guard(det_status_map_mutex_);
    det_status_map_ = det_status_map;
  }

  void publish_transform(const pinocchio::SE3 &transform, const std::string parent_frame, std::string child_frame)
  {
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

    // Send the transformation.
    tf_broadcaster_->sendTransform(transform_msg);
  }

  pinocchio::SE3 get_in_base_M_grasp(const pinocchio::SE3 &in_base_M_object, const std::string &object_frame)
  {
    // Compute desired gripper orientation.
    bool grasp_from_top = objects_to_grasp_from_top_map_[object_frame];
    Eigen::Matrix3d in_base_rot_grasp;
    double yaw_angle = std::atan2(in_base_M_object.translation()[1], in_base_M_object.translation()[0]);
    if (grasp_from_top)
    {
      Eigen::AngleAxisd yaw_rot_angle_axis = Eigen::AngleAxisd(-yaw_angle, Eigen::Vector3d::UnitZ());
      in_base_rot_grasp = top_ideal_rot_quat_.toRotationMatrix() * yaw_rot_angle_axis.toRotationMatrix();
    }
    else
    {
      Eigen::AngleAxisd yaw_rot_angle_axis = Eigen::AngleAxisd(-yaw_angle, Eigen::Vector3d::UnitY());
      in_base_rot_grasp = front_ideal_rot_quat_.toRotationMatrix() * yaw_rot_angle_axis.toRotationMatrix();
    }

    // Compute desired gripper translation.
    Eigen::Vector3d in_base_translation_object_to_grasp = in_base_rot_grasp * in_grasp_translation_object_to_grasp_map_[object_frame];

    return pinocchio::SE3(in_base_rot_grasp, in_base_M_object.translation() + in_base_translation_object_to_grasp);
  }

  bool find_next_object_to_grasp_transform()
  {
    bool found_object = false;
    std::vector<std::string> objects_frame;
    if (state_ == SEARCHING_OBJECTS)
      objects_frame = objects_frame_;
    else
      objects_frame = {std::get<0>(current_object_to_sort_)};
    for (auto &object_frame : objects_frame)
    {
      std::lock_guard<std::mutex> guard(det_status_map_mutex_);
      if (det_status_map_[object_frame].is_in_fov)
      {
        pinocchio::SE3 in_base_M_object = det_status_map_[object_frame].in_base_M_frame.value();
        pinocchio::SE3 in_base_M_grasp = get_in_base_M_grasp(in_base_M_object, object_frame);
        found_object = true;
        current_object_to_sort_ = make_tuple(object_frame, in_base_M_grasp);
        RCLCPP_INFO(logger_, "found object : %s", object_frame.c_str());
        break;
      }
    }
    return found_object;
  }

  bool find_box_transform()
  {
    std::lock_guard<std::mutex> guard(det_status_map_mutex_);
    for (std::string &box_frame : box_frames_)
    {
      if (det_status_map_[box_frame].is_in_fov)
      {
        // Compute in_base_M_box with regards to the tag we found.
        in_base_M_box_ = det_status_map_[box_frame].in_base_M_frame.value();
        if (box_frame == "box_left")
          in_base_M_box_.translation() = in_base_M_box_.translation() + in_base_M_box_.rotation() * Eigen::Vector3d(0.071, 0., 0.);
        else if (box_frame == "box_right")
          in_base_M_box_.translation() = in_base_M_box_.translation() + in_base_M_box_.rotation() * Eigen::Vector3d(-0.064, 0., 0.);

        // Compute in_world_M_box.
        pinocchio::SE3 in_world_M_base = get_most_recent_transform(world_frame_, base_frame_);
        in_world_M_box_ = in_world_M_base * in_base_M_box_;
        return true;
      }
    }

    return false;
  }

  void do_search_object_action()
  {
    if (!find_next_object_to_grasp_transform())
    {
      RCLCPP_INFO(logger_, "didn't found objects, restart state actions");
      current_state_action_were_sent_ = false;
    }
    else if (state_ == SEARCHING_OBJECTS)
    {
      q_init_idx_ = 0;
      search_obj_base_waypoints_vec_idx_ = (search_obj_base_waypoints_vec_idx_ + 1) % base_poses_waypoints_vec_.size();
    }
  }

  void do_search_box_action()
  {
    // If we didn't found the box, restart state actions.
    if (!find_box_transform())
    {
      current_state_action_were_sent_ = false;
      actions_ = {};
      reset_actions_attributes();
    }
  }

  bool goal_pose_achieved(const Eigen::VectorXd &q)
  {
    if (!trajectory_ready_ || time_ < motion_planner.get_traj_duration())
      return false;
    for (int joint_idx = 0; joint_idx < nq_; joint_idx++)
    {
      if (std::abs(q_waypoints.back()[joint_idx] - q[joint_idx]) > joints_des_precision_[joint_idx])
        return false;
    }
    return true;
  }

  bool goal_base_pose_published()
  {
    return goal_base_pose_published_;
  }

  void compute_searching_box_base_waypoints()
  {
    Eigen::Vector3d last_waypoint;
    if (std::abs(std::abs(base_pose_[1]) < 0.15))
      searching_box_base_waypoints_ = {Eigen::Vector3d(base_pose_[0], base_pose_[1], M_PI)};
    else
      searching_box_base_waypoints_ = {
          Eigen::Vector3d(base_pose_[0], in_world_M_box_.translation()[1], M_PI),
      };
  }

  std::vector<Eigen::VectorXd> get_base_goal_waypoints()
  {
    if (state_ == SEARCHING_OBJECTS)
    {
      std::vector<Eigen::VectorXd> base_waypoints = base_poses_waypoints_vec_[search_obj_base_waypoints_vec_idx_];
      for (Eigen::VectorXd &base_waypoint : base_waypoints)
        base_waypoint += in_world_M_box_.translation();
      return base_waypoints;
    }
    else if (state_ == SEARCHING_BOX)
      return searching_box_base_waypoints_;
    else if (state_ == PLACING)
    {

      pinocchio ::SE3 in_world_M_base = get_most_recent_transform(world_frame_, base_frame_);
      if ((in_world_M_box_.inverse() * in_world_M_base).translation().norm() < box_dist_while_placing_)
        return {Eigen::Vector3d(base_pose_[0], base_pose_[1], M_PI)};
      else
      {
        return {Eigen::Vector3d(box_dist_while_placing_ + in_world_M_box_.translation()[0], in_world_M_box_.translation()[1], M_PI)};
      }
    }
    else
    {
      RCLCPP_ERROR(logger_, "Retrieving base waypoints in an unexpected state, state idx : %d", static_cast<int>(state_));
      return {Eigen::Vector3d(0.0, 0., 0.0)};
    }
  }

  void update_nav_action(const rclcpp_action::ResultCode &nav_result)
  {
    switch (nav_result)
    {
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        goal_base_pose_published_ = false;
        last_nav_succeed = true;
        break;
      case rclcpp_action::ResultCode::UNKNOWN:
        break;
      default:
        goal_base_pose_published_ = false;
        last_nav_succeed = false;
        time_ = 0.;
        break;
      }
    }
  }

  void set_goal_base_pose_published(const bool &goal_base_pose_published)
  {
    goal_base_pose_published_ = goal_base_pose_published;
  }

  double time()
  {
    return time_;
  }

  bool action_is_finished(Eigen::VectorXd q, double time, std::tuple<ActionType, double> &current_action, const rclcpp_action::ResultCode &nav_result)
  {
    ActionType action_type = std::get<0>(current_action);
    if ((action_type == WAIT && time >= std::get<1>(current_action)) || (action_type == MOVE_JAW) ||
        (action_type == SEARCH_OBJECT) || (action_type == SEARCH_BOX) ||
        (action_type == MOVE_BASE && nav_result == rclcpp_action::ResultCode::SUCCEEDED))
      return true;
    if (action_type == FOLLOW_TRAJ || action_type == SET_MOVING_BASE_CONFIGURATION)
    {
      if (trajectory_ready_ && goal_pose_achieved(q))
        return true;
    }
    return false;
  }

  std::vector<std::tuple<ActionType, double>> get_state_actions()
  {
    std::vector<std::tuple<ActionType, double>> actions = {};
    switch (state_)
    {
    case SEARCHING_OBJECTS:
      actions.push_back(std::make_tuple(MOVE_JAW, -0.45));
      if (robot_name_ == "LeKiwi")
        actions.push_back(std::make_tuple(MOVE_BASE, 1.0));
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 1.));
      actions.push_back(std::make_tuple(WAIT, wait_duration_before_vision_action_));
      actions.push_back(std::make_tuple(SEARCH_OBJECT, 1.0));
      break;
    case GOING_TO_GRASP_POSE:
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 2.));
      actions.push_back(std::make_tuple(WAIT, wait_duration_before_vision_action_));
      actions.push_back(std::make_tuple(SEARCH_OBJECT, 2.0));
      actions.push_back(std::make_tuple(MOVE_JAW, 1.0));
      actions.push_back(std::make_tuple(WAIT, 0.5));
      break;
    case GRASPING:
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 3.));
      actions.push_back(std::make_tuple(WAIT, 0.5));
      actions.push_back(std::make_tuple(MOVE_JAW, -0.45));
      actions.push_back(std::make_tuple(WAIT, 1.0));
      break;
    case SEARCHING_BOX:
      if (robot_name_ == "LeKiwi")
      {
        actions.push_back(std::make_tuple(SET_MOVING_BASE_CONFIGURATION, 2.0));
        actions.push_back(std::make_tuple(MOVE_BASE, 3.0));
      }
      actions.push_back(std::make_tuple(WAIT, wait_duration_before_vision_action_));
      actions.push_back(std::make_tuple(SEARCH_BOX, 2.0));
      break;
    case PLACING:
      if (robot_name_ == "LeKiwi")
      {
        actions.push_back(std::make_tuple(MOVE_BASE, 4.0));
        actions.push_back(std::make_tuple(WAIT, wait_duration_before_vision_action_));
        actions.push_back(std::make_tuple(SEARCH_BOX, 2.0));
      }
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 4.));
      actions.push_back(std::make_tuple(MOVE_JAW, 1.0));
      actions.push_back(std::make_tuple(WAIT, 0.5));
      if (robot_name_ == "LeKiwi")
        actions.push_back(std::make_tuple(SET_MOVING_BASE_CONFIGURATION, 1.0));
      break;
    }
    return actions;
  }

  void update_actions(const rclcpp_action::ResultCode &nav_result)
  {
    if (actions_.size() > 0)
    {
      std::tuple<ActionType, double> action = actions_[0];
      ActionType action_type = std::get<0>(action);
      if (start_new_action_)
      {
        start_new_action_ = false;
        if (action_type == SEARCH_OBJECT)
        {
          do_search_object_action();
        }
        if (action_type == SEARCH_BOX)
        {
          do_search_box_action();
        }
        if (action_type == FOLLOW_TRAJ || action_type == SET_MOVING_BASE_CONFIGURATION)
        {
          if (trajectory_computing_thread_.joinable())
            trajectory_computing_thread_.join();
          trajectory_computing_thread_ = std::thread(&PlannerManager::compute_trajectory, this);
        }
      }
    }
  }

  void reset_actions_attributes()
  {
    time_ = 0.;
    goal_base_pose_published_ = false;
    start_new_action_ = true;
    trajectory_ready_ = false;
  }

  void update_actions_status(const rclcpp_action::ResultCode &nav_result)
  {
    if (actions_.size() > 0)
    {
      std::tuple<ActionType, double> action = actions_[0];
      ActionType action_type = std::get<0>(action);
      if (action_type == MOVE_BASE)
      {
        update_nav_action(nav_result);
      }
      if (action_is_finished(current_q_, time_, action, nav_result))
      {
        actions_.erase(actions_.begin());
        time_ = 0.;
        reset_actions_attributes();
      }
    }
  }

  void update_state(const Eigen::VectorXd &current_q, const Eigen::VectorXd &base_pose, const rclcpp_action::ResultCode &nav_result, const rclcpp::Time ros_time)
  {
    current_q_ = current_q;
    base_pose_ = base_pose;
    ros_time_ = ros_time;
    update_actions(nav_result);
    if (new_transform != pinocchio::SE3::Identity())
    {
      publish_transform(first_transform, base_frame_, "first_transform");
      publish_transform(sec_transform, base_frame_, "sec_transform");
      publish_transform(new_transform, base_frame_, "new_transform");
    }

    //  Change state if we it has a goal and we reach it.
    if (actions_.size() == 0 && current_state_action_were_sent_)
    {
      current_state_action_were_sent_ = false;
      int new_state_idx = (int(state_) + 1) % state_order_.size();
      if (state_order_[new_state_idx] == SEARCHING_BOX && robot_name_ == "SO-101")
        new_state_idx++;
      state_ = state_order_[new_state_idx];
      RCLCPP_INFO(logger_, "New state idx : %d", new_state_idx);
      if (state_ == SEARCHING_BOX)
        compute_searching_box_base_waypoints();
    }

    // Retrieve state actions the first time we're in the state
    if (!current_state_action_were_sent_)
    {
      std::vector<std::tuple<ActionType, double>> actions = get_state_actions();
      actions_.insert(actions_.end(), actions.begin(), actions.end());
      current_state_action_were_sent_ = true;
    }
    std::tuple<ActionType, double> action = actions_[0];
    ActionType action_type = std::get<0>(action);
    if ((action_type != FOLLOW_TRAJ && action_type != SET_MOVING_BASE_CONFIGURATION) || trajectory_ready_)
      time_ += state_update_period_;
  }

  std::tuple<ActionType, double> get_current_action()
  {
    if (actions_.size() == 0)
      return std::make_tuple(NONE, 0.);
    return actions_[0];
  }

  void compute_trajectory()
  {
    RCLCPP_INFO(logger_, "Start new motion planning.");
    std::tuple<ActionType, double> action = actions_[0];
    ActionType action_type = std::get<0>(action);
    if (action_type == SET_MOVING_BASE_CONFIGURATION)
    {
      q_waypoints = {q_base_moving_};
    }

    else if (state_ == SEARCHING_OBJECTS)
    {
      q_waypoints = {qs_searching_objects_[q_init_idx_]};
      q_init_idx_ = (q_init_idx_ + 1) % qs_searching_objects_.size();
      if (q_init_idx_ == 0)
        search_obj_base_waypoints_vec_idx_ = (search_obj_base_waypoints_vec_idx_ + 1) % base_poses_waypoints_vec_.size();
    }
    else if (state_ == GOING_TO_GRASP_POSE)
    {
      pinocchio::SE3 grasping_in_base_M_gripper = std::get<1>(current_object_to_sort_);
      pinocchio::SE3 current_in_base_M_ee = motion_planner.get_frame_pose_at_q(current_q_, ee_frame_name_);
      double object_yaw_angle = std::atan2(grasping_in_base_M_gripper.translation()[1], grasping_in_base_M_gripper.translation()[0]);
      double ee_yaw_angle = std::atan2(current_in_base_M_ee.translation()[1], current_in_base_M_ee.translation()[0]);
      Eigen::AngleAxisd z_angle_axis_rot = Eigen::AngleAxisd(-(ee_yaw_angle - object_yaw_angle), Eigen::Vector3d::UnitZ());
      pinocchio::SE3 des_in_base_M_ee = current_in_base_M_ee;
      des_in_base_M_ee.translation() = z_angle_axis_rot.toRotationMatrix() * current_in_base_M_ee.translation();
      des_in_base_M_ee.rotation() = z_angle_axis_rot.toRotationMatrix() * des_in_base_M_ee.rotation();
      Eigen::VectorXd des_q = motion_planner.get_inverse_kinematic_at_pose(current_q_, des_in_base_M_ee);
      q_waypoints = {des_q};
    }
    else if (state_ == GRASPING)
    {
      std::string object_frame = std::get<0>(current_object_to_sort_);
      pinocchio::SE3 in_base_M_grasp = std::get<1>(current_object_to_sort_);

      Eigen::VectorXd q_goal = motion_planner.get_inverse_kinematic_at_pose(current_q_, in_base_M_grasp);
      pinocchio::SE3 above_object_des_transform = in_base_M_grasp;
      Eigen::VectorXd translation;
      if (object_frame == "metal")
        translation = Eigen::Vector3d(0., 0.01, -0.06);
      else
        translation = Eigen::Vector3d(0., -0.06, -0.01);
      translation = above_object_des_transform.rotation() * translation;
      above_object_des_transform.translation() += translation;
      Eigen::VectorXd q_waypoint_above_object = motion_planner.get_inverse_kinematic_at_pose(current_q_, above_object_des_transform);
      q_waypoints = {q_waypoint_above_object, q_goal};
      Eigen::Vector3d des_trans = in_base_M_grasp.translation();
    }
    else if (state_ == PLACING)
    {
      pinocchio::SE3 in_base_M_ee = motion_planner.get_frame_pose_at_q(current_q_, ee_frame_name_);
      pinocchio::SE3 first_waypoint_in_base_M_ee = in_base_M_ee;
      first_waypoint_in_base_M_ee.translation()[2] += 0.04;
      Eigen::VectorXd q_first_waypoint = motion_planner.get_inverse_kinematic_at_pose(current_q_, first_waypoint_in_base_M_ee);

      std::string object = std::get<0>(current_object_to_sort_);
      auto it = find(objects_frame_.begin(), objects_frame_.end(), object);
      int object_idx = static_cast<int>(std::distance(objects_frame_.begin(), it));
      pinocchio::SE3 in_box_M_compartment = in_box_M_compartments_[object_idx];
      pinocchio::SE3 des_in_base_M_object = in_base_M_box_ * in_box_M_compartment;
      pinocchio::SE3 in_base_M_grasp = get_in_base_M_grasp(des_in_base_M_object, object);
      Eigen::VectorXd q_above_compartment = motion_planner.get_inverse_kinematic_at_pose(current_q_, in_base_M_grasp);
      q_waypoints = {q_first_waypoint, q_above_compartment};
    }
    else
    {
      RCLCPP_ERROR(logger_, "Started motion planning in an unexpected state, state idx : %d",
                   static_cast<int>(state_));
      q_waypoints = {current_q_};
    }
    motion_planner.set_plan(current_q_, q_waypoints);
    trajectory_ready_ = true;
    RCLCPP_INFO(logger_, "Motion planning ready !");
  }

  std::tuple<Eigen::VectorXd, Eigen::VectorXd, bool> get_traj_value_at_t()
  {
    return motion_planner.get_traj_value_at_t(time_);
  }

  bool trajectory_ready()
  {
    return trajectory_ready_;
  }

  StateMachine get_state()
  {
    return state_;
  }

  double get_traj_duration()
  {
    return motion_planner.get_traj_duration();
  }
  bool last_nav_succeed = false;

private:
  int nq_;
  rclcpp::Time ros_time_;
  Eigen::VectorXd current_q_, q_base_moving_;
  std::map<std::string, Eigen::Vector3d> in_grasp_translation_object_to_grasp_map_;
  std::map<std::string, bool> objects_to_grasp_from_top_map_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Logger logger_ = rclcpp::get_logger("joint_trajectory_publisher");
  enum StateMachine state_ = SEARCHING_OBJECTS;
  std::vector<StateMachine> state_order_ = {SEARCHING_OBJECTS, GOING_TO_GRASP_POSE, GRASPING, SEARCHING_BOX, PLACING};
  std::vector<std::tuple<ActionType, double>> actions_ = {};
  std::string robot_name_, world_frame_, base_frame_, ee_frame_name_;
  std::atomic<bool> trajectory_ready_ = false;
  bool current_state_action_were_sent_ = false, goal_base_pose_published_ = false, start_new_action_ = true;
  double box_dist_while_placing_, wait_duration_before_vision_action_, time_ = 0., state_update_period_;
  MotionPlanner motion_planner;
  std::map<std::string, DetectionStatus> det_status_map_;
  std::mutex det_status_map_mutex_;
  std::vector<double> joints_des_precision_;
  std::vector<Eigen::VectorXd> q_waypoints = {}, qs_searching_objects_ = {}, searching_box_base_waypoints_ = {};
  std::vector<pinocchio::SE3> in_box_M_compartments_ = {};
  pinocchio::SE3 in_world_M_box_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.25, 0.0, 0.0)), in_base_M_box_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.25, 0.0, 0.0)), new_transform = pinocchio::SE3::Identity(), first_transform = pinocchio::SE3::Identity(), sec_transform = pinocchio::SE3::Identity();

  pinocchio::SE3::Quaternion front_ideal_rot_quat_ = Eigen::Quaternion(0.466896, -0.53408, 0.634467, -0.30695), top_ideal_rot_quat_ = Eigen::Quaternion(0.0136498, -0.624938, 0.780508, -0.00855614);
  std::vector<std::vector<Eigen::VectorXd>> base_poses_waypoints_vec_ = {{Eigen::Vector3d(0.25, 0.0, 0.0)}, {Eigen::Vector3d(1.0, 0.0, 0.0)}, {Eigen::Vector3d(1.0, -0.75, -M_PI_2)}, {Eigen::Vector3d(0.25, -0.75, -M_PI)}};
  int search_obj_base_waypoints_vec_idx_ = 0, q_init_idx_ = 0;
  std::thread trajectory_computing_thread_;
  std::vector<std::string> objects_frame_;
  std::tuple<std::string, pinocchio::SE3> current_object_to_sort_;
  Eigen::VectorXd base_pose_ = Eigen::VectorXd::Zero(3);
  std::vector<std::string> box_frames_ = {"box_center", "box_left", "box_right"};
};
