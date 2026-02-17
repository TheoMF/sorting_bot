#include <atomic>
#include <mutex>
#include <variant>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include "sorting_bot/motion_planner.hpp"

enum StateMachine {
  SEARCHING_OBJECTS,
  GOING_ABOVE_OBJECT_POSE, // Useful state to improve object detection precision.
  GRASPING,
  SEARCHING_BOX,
  PLACING
};

enum ActionType { NONE, WAIT, MOVE_JAW, MOVE_BASE, MOVE_ARM, SEARCH_OBJECT, SEARCH_BOX };

struct Action {
  ActionType type;
  int id;
  std::variant<std::monostate, double, Eigen::VectorXd> value;

  const char *get_value_type_name() {
    return std::visit([](auto &v) -> std::type_index { return typeid(v); }, value).name();
  }
};

struct DetectionStatus {
  std::string camera_frame, frame;
  bool is_in_fov;
  std::optional<builtin_interfaces::msg::Time> last_stamp;
  std::optional<pinocchio::SE3> in_base_M_frame;

  DetectionStatus() {
    camera_frame = "";
    frame = "";
    is_in_fov = false;
    last_stamp = std::nullopt;
    in_base_M_frame = std::nullopt;
  }

  DetectionStatus(const std::string &_camera_frame, const std::string &_frame)
      : camera_frame(_camera_frame), frame(_frame) {
    is_in_fov = false;
    last_stamp = std::nullopt;
    in_base_M_frame = std::nullopt;
  }
};

pinocchio::SE3 transform_msg_to_SE3(const geometry_msgs::msg::Transform &transform) {
  Eigen::Quaterniond q(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  Eigen::Vector3d t(transform.translation.x, transform.translation.y, transform.translation.z);
  pinocchio::SE3 se3(q, t);
  return se3;
}

class PlannerManager {
public:
  PlannerManager() {}

  void initialize(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
                  const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster, const std::string &urdf,
                  joint_trajectory_publisher::Params &params) {
    // Set tf attributes.
    tf_buffer_ = tf_buffer;
    tf_broadcaster_ = tf_broadcaster;

    // Set ROS parameters related attributes.
    nq_ = params.nq;
    objects_frame_ = params.objects_frame;
    box_frames_ = params.box_frames;
    joints_des_precision_ = params.joints_des_precision;
    robot_name_ = params.robot_name;
    world_frame_ = params.world_frame;
    base_frame_ = params.base_frame;
    ee_frame_name_ = params.ee_frame_name;
    open_gripper_position_ = params.open_gripper_position;
    closed_gripper_position_ = params.closed_gripper_position;
    box_dist_while_placing_ = params.box_dist_while_placing;
    search_box_y_dist_threshold_ = params.search_box_y_dist_threshold;
    wait_duration_before_vision_action_ = params.wait_duration_before_vision_action;
    wait_duration_after_gripper_moved_ = params.wait_duration_after_gripper_moved;
    initialize_in_box_M_compartment_map(params.box_width, params.box_length, params.object_height_while_placing_in_box);
    motion_planner.initialize(urdf, params);

    // Set ROS parameters related attributes requiring casting to Eigen containers.
    q_base_moving_ = Eigen::Map<Eigen::VectorXd>(params.q_base_moving.data(), params.q_base_moving.size());
    in_grasp_translation_grasp_to_pre_grasp_ = Eigen::Map<Eigen::Vector3d>(
        params.in_grasp_translation_grasp_to_pre_grasp.data(), params.in_grasp_translation_grasp_to_pre_grasp.size());
    in_base_translation_gripper_to_pre_placing_ =
        Eigen::Map<Eigen::Vector3d>(params.in_base_translation_gripper_to_pre_placing.data(),
                                    params.in_base_translation_gripper_to_pre_placing.size());
    state_update_period_ = 1.0 / params.rate;
    in_world_rot_top_grasp_quat_ =
        Eigen::Quaterniond(params.in_world_rot_top_grasp_quat[0], params.in_world_rot_top_grasp_quat[1],
                           params.in_world_rot_top_grasp_quat[2], params.in_world_rot_top_grasp_quat[3]);
    in_world_rot_front_grasp_quat_ =
        Eigen::Quaterniond(params.in_world_rot_front_grasp_quat[0], params.in_world_rot_front_grasp_quat[1],
                           params.in_world_rot_front_grasp_quat[2], params.in_world_rot_front_grasp_quat[3]);

    // Set in_world_M_box.
    Eigen::Quaterniond in_world_rot_box_quat =
        Eigen::Quaterniond(params.in_world_pose_box[3], params.in_world_pose_box[4], params.in_world_pose_box[5],
                           params.in_world_pose_box[6]);
    Eigen::Vector3d in_world_translation_world_to_box =
        Eigen::Vector3d(params.in_world_pose_box[0], params.in_world_pose_box[1], params.in_world_pose_box[2]);
    in_world_M_box_ = pinocchio::SE3(in_world_rot_box_quat.toRotationMatrix(), in_world_translation_world_to_box);

    // Set objects frame related maps.
    for (std::string &object_frame : params.objects_frame) {
      std::vector<double> in_grasp_translation_object_to_grasp_vec =
          params.objects_frame_map.at(object_frame).in_grasp_translation_object_to_grasp;
      Eigen::Vector3d in_grasp_translation_object_to_grasp = Eigen::Map<Eigen::Vector3d>(
          in_grasp_translation_object_to_grasp_vec.data(), in_grasp_translation_object_to_grasp_vec.size());
      in_grasp_translation_object_to_grasp_map_[object_frame] = in_grasp_translation_object_to_grasp;
      objects_to_grasp_from_top_map_[object_frame] = params.objects_frame_map.at(object_frame).grasp_from_top;
    }

    for (std::string &base_waypoint_name : params.base_waypoint_names) {
      std::vector<double> base_waypoint_vec = params.base_waypoint_names_map.at(base_waypoint_name).base_waypoint;
      Eigen::Vector3d base_waypoint = Eigen::Map<Eigen::Vector3d>(base_waypoint_vec.data(), base_waypoint_vec.size());
      base_waypoint_vec_.push_back(base_waypoint);
    }

    // Set box frames translation map.
    for (std::string &box_frame : params.box_frames) {
      std::vector<double> in_box_frame_translation_box_frame_to_box_center_vec =
          params.box_frames_map.at(box_frame).in_box_frame_translation_box_frame_to_box_center;
      Eigen::Vector3d in_box_frame_translation_box_frame_to_box_center =
          Eigen::Map<Eigen::Vector3d>(in_box_frame_translation_box_frame_to_box_center_vec.data(),
                                      in_box_frame_translation_box_frame_to_box_center_vec.size());
      in_box_frame_translation_box_frame_to_box_center_map_[box_frame] =
          in_box_frame_translation_box_frame_to_box_center;
    }

    // Set qs_searching_objects.
    std::vector<std::vector<double>> qs_searching_objects = {params.q_searching_object, params.q_searching_object,
                                                             params.q_searching_object};
    qs_searching_objects[1][0] -= params.q0_shift_q_searching_object;
    qs_searching_objects[2][0] += params.q0_shift_q_searching_object;
    for (std::vector<double> &q_searching_objects : qs_searching_objects)
      qs_searching_objects_.push_back(
          Eigen::Map<Eigen::VectorXd>(q_searching_objects.data(), q_searching_objects.size()));
    qs_searching_objects_.push_back(q_base_moving_);
  }

  void initialize_in_box_M_compartment_map(const double &box_width, const double &box_length,
                                           const double &object_height_while_placing_in_box) {
    Eigen::Quaterniond x_axis_rot_pi_quat(0., 1., 0., 0.);
    int object_idx = 0;
    for (double length_idx = -1.; length_idx == -1. || length_idx == 1.; length_idx += 2.) {
      for (double width_idx = -1.; width_idx == -1. || width_idx == 1.; width_idx += 2.) {
        Eigen::Vector3d box_to_compartment_trans(length_idx * box_length / 4.0, object_height_while_placing_in_box,
                                                 -box_width / 2.0 + width_idx * box_width / 4.0);
        pinocchio::SE3 in_box_M_compartment(x_axis_rot_pi_quat, box_to_compartment_trans);
        in_box_M_compartment_map_[objects_frame_[object_idx]] = in_box_M_compartment;
        object_idx++;
      }
    }
  }

  pinocchio::SE3 get_most_recent_transform(const std::string &parent_frame, const std::string &child_frame) {
    geometry_msgs::msg::TransformStamped stamped_transform =
        tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
    return transform_msg_to_SE3(stamped_transform.transform);
  }

  void update_det_status_map(const std::map<std::string, DetectionStatus> &det_status_map) {
    std::lock_guard<std::mutex> guard(det_status_map_mutex_);
    det_status_map_ = det_status_map;
  }

  void publish_transform(const pinocchio::SE3 &transform, const std::string parent_frame, std::string child_frame) {
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

    tf_broadcaster_->sendTransform(transform_msg);
  }

  pinocchio::SE3 get_in_base_M_grasp(const pinocchio::SE3 &in_base_M_object, const std::string &object_frame) {
    // Compute desired gripper orientation.
    bool grasp_from_top = objects_to_grasp_from_top_map_[object_frame];
    Eigen::Matrix3d in_base_rot_grasp;
    double yaw_angle = std::atan2(in_base_M_object.translation()[1], in_base_M_object.translation()[0]);
    if (grasp_from_top) {
      Eigen::AngleAxisd yaw_rot_angle_axis = Eigen::AngleAxisd(-yaw_angle, Eigen::Vector3d::UnitZ());
      in_base_rot_grasp = in_world_rot_top_grasp_quat_.toRotationMatrix() * yaw_rot_angle_axis.toRotationMatrix();
    } else {
      Eigen::AngleAxisd yaw_rot_angle_axis = Eigen::AngleAxisd(-yaw_angle, Eigen::Vector3d::UnitY());
      in_base_rot_grasp = in_world_rot_front_grasp_quat_.toRotationMatrix() * yaw_rot_angle_axis.toRotationMatrix();
    }

    // Compute translation from object frame to gripper frame while grasping
    Eigen::Vector3d in_base_translation_object_to_grasp =
        in_base_rot_grasp * in_grasp_translation_object_to_grasp_map_[object_frame];

    return pinocchio::SE3(in_base_rot_grasp, in_base_M_object.translation() + in_base_translation_object_to_grasp);
  }

  std::optional<DetectionStatus> search_frames_in_det_map(const std::vector<std::string> &frames) {
    std::lock_guard<std::mutex> guard(det_status_map_mutex_);
    for (const std::string &frame : frames) {
      if (det_status_map_[frame].is_in_fov) {
        return det_status_map_[frame];
      }
    }

    return std::nullopt;
  }

  void restart_state_actions() {
    current_state_action_were_retrieved_ = false;
    actions_ = {};
    reset_actions_attributes();
  }

  void do_search_object_action() {
    // Determine for which frames we are gonna search detections in tf.
    std::vector<std::string> objects_frame;
    if (state_ == SEARCHING_OBJECTS)
      objects_frame = objects_frame_;
    else
      objects_frame = {current_object_to_sort_.frame};

    // Check detections status for each objects until we found one in fov.
    std::optional<DetectionStatus> optional_det = search_frames_in_det_map(objects_frame);
    if (optional_det.has_value()) {
      current_object_to_sort_ = optional_det.value();
      RCLCPP_INFO(logger_, "Object %s is in fov.", current_object_to_sort_.frame.c_str());
      if (state_ == SEARCHING_OBJECTS) {
        // Move directly to next place next time we search an object.
        base_waypoint_vec_idx_ = (base_waypoint_vec_idx_ + 1) % base_waypoint_vec_.size();
        qs_searching_object_idx_ = 0;
      }
    } else {
      RCLCPP_INFO(logger_, "didn't found objects, restart state actions");
      restart_state_actions();
    }
  }

  void do_search_box_action() {
    std::optional<DetectionStatus> optional_det = search_frames_in_det_map(box_frames_);
    if (optional_det.has_value()) {
      // Compute in_base_M_box with regards to the tag we found.
      in_base_M_box_ = optional_det.value().in_base_M_frame.value();
      in_base_M_box_.translation() =
          in_base_M_box_.translation() +
          in_base_M_box_.rotation() * in_box_frame_translation_box_frame_to_box_center_map_[optional_det.value().frame];

      // Compute in_world_M_box.
      pinocchio::SE3 in_world_M_base = get_most_recent_transform(world_frame_, base_frame_);
      in_world_M_box_ = in_world_M_base * in_base_M_box_;
    } else {
      RCLCPP_INFO(logger_, "didn't found the box, restart state actions");
      restart_state_actions();
    }
  }

  bool goal_pose_achieved() {
    if (!trajectory_ready_ || time_ < motion_planner.get_traj_duration())
      return false;
    for (int joint_idx = 0; joint_idx < nq_; joint_idx++) {
      if (std::abs(q_waypoints.back()[joint_idx] - current_q_[joint_idx]) > joints_des_precision_[joint_idx])
        return false;
    }
    return true;
  }

  bool goal_base_pose_published() { return goal_base_pose_published_; }

  std::vector<Eigen::VectorXd> get_base_goal_waypoints() {
    if (state_ == SEARCHING_OBJECTS) {
      Eigen::VectorXd base_waypoint = base_waypoint_vec_[base_waypoint_vec_idx_];
      base_waypoint += in_world_M_box_.translation();
      return {base_waypoint};
    } else if (state_ == SEARCHING_BOX) {
      // If below y dist threshold, only turn around.
      if (std::abs(base_pose_[1] - in_world_M_box_.translation()[1]) < search_box_y_dist_threshold_)
        return {Eigen::Vector3d(base_pose_[0], base_pose_[1], M_PI)};
      else
        return {Eigen::Vector3d(base_pose_[0], in_world_M_box_.translation()[1], M_PI)};
    } else if (state_ == PLACING) {
      pinocchio ::SE3 in_world_M_base = get_most_recent_transform(world_frame_, base_frame_);
      if ((in_world_M_box_.inverse() * in_world_M_base).translation().norm() < box_dist_while_placing_)
        return {Eigen::Vector3d(base_pose_[0], base_pose_[1], M_PI)};
      else {
        return {Eigen::Vector3d(box_dist_while_placing_ + in_world_M_box_.translation()[0],
                                in_world_M_box_.translation()[1], M_PI)};
      }
    } else {
      RCLCPP_ERROR(logger_, "Retrieving base waypoints in an unexpected state, state idx : %d",
                   static_cast<int>(state_));
      return {Eigen::Vector3d(0.0, 0., 0.0)};
    }
  }

  void update_nav_action(const rclcpp_action::ResultCode &nav_result) {
    switch (nav_result) {
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

  void set_goal_base_pose_published(const bool &goal_base_pose_published) {
    goal_base_pose_published_ = goal_base_pose_published;
  }

  double time() { return time_; }

  bool action_is_finished(double time, Action &action, const rclcpp_action::ResultCode &nav_result) {
    if ((action.type == MOVE_JAW) || (action.type == SEARCH_OBJECT) || (action.type == SEARCH_BOX) ||
        (action.type == MOVE_BASE && nav_result == rclcpp_action::ResultCode::SUCCEEDED) ||
        (action.type == MOVE_ARM && trajectory_ready_ && goal_pose_achieved()))
      return true;
    if (action.type == WAIT) {
      if (const double *value = std::get_if<double>(&action.value)) {
        if (time >= *value)
          return true;
      } else {
        RCLCPP_ERROR(logger_, "Stored wrong variant value type %s for Wait action.", action.get_value_type_name());
        throw std::runtime_error("Got wrong type for wait action value");
      }
    }
    return false;
  }

  std::vector<Action> get_state_actions() {
    std::vector<Action> actions = {};
    switch (state_) {
    case SEARCHING_OBJECTS:
      actions.push_back({MOVE_JAW, 1, closed_gripper_position_});
      if (robot_name_ == "LeKiwi")
        actions.push_back({MOVE_BASE, 1, std::monostate()});
      actions.push_back({MOVE_ARM, 1, std::monostate()});
      actions.push_back({WAIT, 1, wait_duration_before_vision_action_});
      actions.push_back({SEARCH_OBJECT, 1, std::monostate()});
      break;
    case GOING_ABOVE_OBJECT_POSE:
      actions.push_back({MOVE_ARM, 2, std::monostate()});
      actions.push_back({WAIT, 2, wait_duration_before_vision_action_});
      actions.push_back({SEARCH_OBJECT, 2, std::monostate()});
      actions.push_back({MOVE_JAW, 2, open_gripper_position_});
      break;
    case GRASPING:
      actions.push_back({MOVE_ARM, 3, std::monostate()});
      actions.push_back({MOVE_JAW, 3, closed_gripper_position_});
      actions.push_back({WAIT, 5, wait_duration_after_gripper_moved_});
      break;
    case SEARCHING_BOX:
      actions.push_back({MOVE_ARM, 4, q_base_moving_});
      actions.push_back({MOVE_BASE, 2, std::monostate()});
      actions.push_back({WAIT, 6, wait_duration_before_vision_action_});
      actions.push_back({SEARCH_BOX, 1, std::monostate()});
      break;
    case PLACING:
      actions.push_back({MOVE_BASE, 3, std::monostate()});
      actions.push_back({WAIT, 7, wait_duration_before_vision_action_});
      actions.push_back({SEARCH_BOX, 1, std::monostate()});
      actions.push_back({MOVE_ARM, 5, std::monostate()});
      actions.push_back({MOVE_JAW, 4, open_gripper_position_});
      actions.push_back({WAIT, 8, wait_duration_after_gripper_moved_});
      if (robot_name_ == "LeKiwi")
        actions.push_back({MOVE_ARM, 6, q_base_moving_});
      break;
    }
    return actions;
  }

  void update_actions(const rclcpp_action::ResultCode &nav_result) {
    if (actions_.size() > 0) {
      Action action = actions_[0];
      if (start_new_action_) {
        start_new_action_ = false;
        if (action.type == SEARCH_OBJECT) {
          do_search_object_action();
        }
        if (action.type == SEARCH_BOX) {
          do_search_box_action();
        }
        if (action.type == MOVE_ARM) {
          if (trajectory_computing_thread_.joinable())
            trajectory_computing_thread_.join();
          trajectory_computing_thread_ = std::thread(&PlannerManager::compute_trajectory, this);
        }
      }
    }
  }

  void reset_actions_attributes() {
    time_ = 0.;
    goal_base_pose_published_ = false;
    start_new_action_ = true;
    trajectory_ready_ = false;
  }

  void update_actions_status(const rclcpp_action::ResultCode &nav_result) {
    if (actions_.size() > 0) {
      Action action = actions_[0];
      if (action.type == MOVE_BASE) {
        update_nav_action(nav_result);
      }
      if (action_is_finished(time_, action, nav_result)) {
        actions_.erase(actions_.begin());
        time_ = 0.;
        reset_actions_attributes();
      }
    }
  }

  void update_state(const Eigen::VectorXd &current_q, const Eigen::VectorXd &base_pose,
                    const rclcpp_action::ResultCode &nav_result, const rclcpp::Time ros_time) {
    current_q_ = current_q;
    base_pose_ = base_pose;
    ros_time_ = ros_time;
    update_actions(nav_result);
    if (new_transform != pinocchio::SE3::Identity()) {
      publish_transform(first_transform, base_frame_, "first_transform");
      publish_transform(sec_transform, base_frame_, "sec_transform");
      publish_transform(new_transform, base_frame_, "new_transform");
    }

    //  Change state if we it has a goal and we reach it.
    if (actions_.size() == 0 && current_state_action_were_retrieved_) {
      current_state_action_were_retrieved_ = false;
      int new_state_idx = (int(state_) + 1) % state_order_.size();
      if (state_order_[new_state_idx] == SEARCHING_BOX && robot_name_ == "SO-101")
        new_state_idx++;
      state_ = state_order_[new_state_idx];
      RCLCPP_INFO(logger_, "New state idx : %d", new_state_idx);
      // if (state_ == SEARCHING_BOX)
      //   compute_searching_box_base_waypoints();
    }

    // Retrieve state actions the first time we're in the state
    if (!current_state_action_were_retrieved_) {
      std::vector<Action> actions = get_state_actions();
      actions_.insert(actions_.end(), actions.begin(), actions.end());
      current_state_action_were_retrieved_ = true;
    }
    Action action = actions_[0];
    if (action.type != MOVE_ARM || trajectory_ready_)
      time_ += state_update_period_;
  }

  Action get_current_action() {
    if (actions_.size() == 0)
      return Action{NONE, 0, std::monostate()};
    return actions_[0];
  }

  void compute_trajectory() {
    RCLCPP_INFO(logger_, "Start new motion planning.");
    Action action = actions_[0];
    if (const Eigen::VectorXd *value = std::get_if<Eigen::VectorXd>(&action.value)) {
      RCLCPP_INFO(logger_, "Started motion planning with value");
      q_waypoints = {*value};
    } else if (state_ == SEARCHING_OBJECTS) {
      q_waypoints = {qs_searching_objects_[qs_searching_object_idx_]};

      // Update configuration and base waypoints attributes idxs to search in a different area next time.
      qs_searching_object_idx_ = (qs_searching_object_idx_ + 1) % qs_searching_objects_.size();
      if (qs_searching_object_idx_ == 0)
        base_waypoint_vec_idx_ = (base_waypoint_vec_idx_ + 1) % base_waypoint_vec_.size();
    } else if (state_ == GOING_ABOVE_OBJECT_POSE) {
      // Evaluate current yaw angle of gripper with respect to world frame.
      pinocchio::SE3 current_in_base_M_gripper = motion_planner.get_frame_pose_at_q(current_q_, ee_frame_name_);
      double current_gripper_yaw_angle =
          std::atan2(current_in_base_M_gripper.translation()[1], current_in_base_M_gripper.translation()[0]);

      // Evaluate yaw angle of gripper to be in top of object with respect to world frame.
      pinocchio::SE3 in_base_M_object = current_object_to_sort_.in_base_M_frame.value();
      double des_gripper_yaw_angle = std::atan2(in_base_M_object.translation()[1], in_base_M_object.translation()[0]);

      // Compute configuration above object pose.
      Eigen::AngleAxisd z_angle_axis_rot =
          Eigen::AngleAxisd(des_gripper_yaw_angle - current_gripper_yaw_angle, Eigen::Vector3d::UnitZ());
      pinocchio::SE3 des_in_base_M_gripper =
          pinocchio::SE3(z_angle_axis_rot.toRotationMatrix() * current_in_base_M_gripper.rotation(),
                         z_angle_axis_rot.toRotationMatrix() * current_in_base_M_gripper.translation());
      Eigen::VectorXd q_above_object = motion_planner.get_inverse_kinematic_at_pose(current_q_, des_in_base_M_gripper);
      q_waypoints = {q_above_object};
    } else if (state_ == GRASPING) {
      // Compute grasp coniguration.
      pinocchio::SE3 in_base_M_grasp =
          get_in_base_M_grasp(current_object_to_sort_.in_base_M_frame.value(), current_object_to_sort_.frame);
      Eigen::VectorXd q_grasp = motion_planner.get_inverse_kinematic_at_pose(current_q_, in_base_M_grasp);

      // Compute translation from grasp frame to pre-grasp frame.
      Eigen::VectorXd in_base_translation_grasp_to_pre_grasp;
      bool grasp_from_top = objects_to_grasp_from_top_map_[current_object_to_sort_.frame];
      if (grasp_from_top) {
        // Rotate in_grasp_translation_grasp_to_pre_grasp_ along x axis if grasping from top.
        Eigen::AngleAxisd x_rot_pi_2_angle_axis = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
        in_base_translation_grasp_to_pre_grasp = in_base_M_grasp.rotation() * x_rot_pi_2_angle_axis.toRotationMatrix() *
                                                 in_grasp_translation_grasp_to_pre_grasp_;
      } else
        in_base_translation_grasp_to_pre_grasp = in_base_M_grasp.rotation() * in_grasp_translation_grasp_to_pre_grasp_;

      // Compute pre-grasp configuration.
      pinocchio::SE3 in_base_M_pre_grasp = pinocchio::SE3(
          in_base_M_grasp.rotation(), in_base_M_grasp.translation() + in_base_translation_grasp_to_pre_grasp);
      Eigen::VectorXd q_pre_grasp = motion_planner.get_inverse_kinematic_at_pose(current_q_, in_base_M_pre_grasp);

      q_waypoints = {q_pre_grasp, q_grasp};
    } else if (state_ == PLACING) {
      // Compute pre-placing configuration.
      pinocchio::SE3 in_base_M_ee = motion_planner.get_frame_pose_at_q(current_q_, ee_frame_name_);
      pinocchio::SE3 in_base_M_pre_placing = pinocchio::SE3(
          in_base_M_ee.rotation(), in_base_M_ee.translation() + in_base_translation_gripper_to_pre_placing_);
      Eigen::VectorXd q_pre_placing = motion_planner.get_inverse_kinematic_at_pose(current_q_, in_base_M_pre_placing);

      // Compute placing configuration.
      pinocchio::SE3 in_box_M_compartment = in_box_M_compartment_map_[current_object_to_sort_.frame];
      pinocchio::SE3 in_base_M_object_placing = in_base_M_box_ * in_box_M_compartment;
      pinocchio::SE3 in_base_M_placing = get_in_base_M_grasp(in_base_M_object_placing, current_object_to_sort_.frame);
      Eigen::VectorXd q_placing = motion_planner.get_inverse_kinematic_at_pose(current_q_, in_base_M_placing);

      q_waypoints = {q_pre_placing, q_placing};
    } else {
      RCLCPP_ERROR(logger_, "Started motion planning in an unexpected state, state idx : %d", static_cast<int>(state_));
      q_waypoints = {current_q_};
    }
    motion_planner.set_plan(current_q_, q_waypoints);
    trajectory_ready_ = true;
    RCLCPP_INFO(logger_, "Motion planning ready !");
  }

  std::tuple<Eigen::VectorXd, Eigen::VectorXd, bool> get_traj_value_at_t() {
    return motion_planner.get_traj_value_at_t(time_);
  }

  bool trajectory_ready() { return trajectory_ready_; }

  StateMachine get_state() { return state_; }

  double get_traj_duration() { return motion_planner.get_traj_duration(); }
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
  std::vector<StateMachine> state_order_ = {SEARCHING_OBJECTS, GOING_ABOVE_OBJECT_POSE, GRASPING, SEARCHING_BOX,
                                            PLACING};
  std::vector<Action> actions_ = {};
  std::string robot_name_, world_frame_, base_frame_, ee_frame_name_;
  std::atomic<bool> trajectory_ready_ = false;
  bool current_state_action_were_retrieved_ = false, goal_base_pose_published_ = false, start_new_action_ = true;
  double box_dist_while_placing_, search_box_y_dist_threshold_, wait_duration_before_vision_action_,
      time_ = 0., state_update_period_, closed_gripper_position_, open_gripper_position_,
      wait_duration_after_gripper_moved_;
  MotionPlanner motion_planner;
  std::map<std::string, DetectionStatus> det_status_map_;
  std::map<std::string, Eigen::Vector3d> in_box_frame_translation_box_frame_to_box_center_map_;
  std::mutex det_status_map_mutex_;
  std::vector<double> joints_des_precision_;
  std::vector<Eigen::VectorXd> q_waypoints = {}, qs_searching_objects_ = {};
  Eigen::Vector3d in_grasp_translation_grasp_to_pre_grasp_, in_base_translation_gripper_to_pre_placing_;
  std::map<std::string, pinocchio::SE3> in_box_M_compartment_map_ = {};
  pinocchio::SE3 in_world_M_box_, in_base_M_box_, new_transform = pinocchio::SE3::Identity(),
                                                  first_transform = pinocchio::SE3::Identity(),
                                                  sec_transform = pinocchio::SE3::Identity();

  pinocchio::SE3::Quaternion in_world_rot_front_grasp_quat_, in_world_rot_top_grasp_quat_;
  std::vector<Eigen::VectorXd> base_waypoint_vec_;
  int base_waypoint_vec_idx_ = 0, qs_searching_object_idx_ = 0;
  std::thread trajectory_computing_thread_;
  std::vector<std::string> objects_frame_;
  DetectionStatus current_object_to_sort_;
  Eigen::VectorXd base_pose_ = Eigen::VectorXd::Zero(3);
  std::vector<std::string> box_frames_;
};
