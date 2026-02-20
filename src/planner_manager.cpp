#include "sorting_bot/planner_manager.hpp"

namespace sorting_bot {

void PlannerManager::initialize(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
                                const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster,
                                const std::string &urdf, joint_trajectory_publisher::Params &params) {
  // Set tf attributes.
  tf_buffer_ = tf_buffer;
  tf_broadcaster_ = tf_broadcaster;

  // Initialize other attributes from ROS parameters.
  initialize_basic_attributes(params);
  initialize_eigen_attributes(params);
  build_frame_maps_from_params(params);
  motion_planner.initialize(urdf, params);
}

void PlannerManager::initialize_basic_attributes(joint_trajectory_publisher::Params &params) {
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
}

void PlannerManager::initialize_eigen_attributes(joint_trajectory_publisher::Params &params) {
  // Set base_waypoint_vec.
  for (std::string &base_waypoint_name : params.base_waypoint_names) {
    std::vector<double> base_waypoint_vec = params.base_waypoint_names_map.at(base_waypoint_name).base_waypoint;
    Eigen::Vector3d base_waypoint = Eigen::Map<Eigen::Vector3d>(base_waypoint_vec.data(), base_waypoint_vec.size());
    base_waypoint_vec_.push_back(base_waypoint);
  }

  // Set qs_searching_objects.
  std::vector<std::vector<double>> qs_searching_objects = {params.q_searching_object, params.q_searching_object,
                                                           params.q_searching_object};
  qs_searching_objects[1][0] -= params.q0_shift_q_searching_object;
  qs_searching_objects[2][0] += params.q0_shift_q_searching_object;
  for (std::vector<double> &q_searching_objects : qs_searching_objects)
    qs_searching_objects_.push_back(
        Eigen::Map<Eigen::VectorXd>(q_searching_objects.data(), q_searching_objects.size()));
  q_base_moving_ = Eigen::Map<Eigen::VectorXd>(params.q_base_moving.data(), params.q_base_moving.size());
  qs_searching_objects_.push_back(q_base_moving_);

  // Set in_world_M_box.
  Eigen::Quaterniond in_world_rot_box_quat =
      Eigen::Quaterniond(params.in_world_pose_box[3], params.in_world_pose_box[4], params.in_world_pose_box[5],
                         params.in_world_pose_box[6]);
  Eigen::Vector3d in_world_translation_world_to_box =
      Eigen::Vector3d(params.in_world_pose_box[0], params.in_world_pose_box[1], params.in_world_pose_box[2]);
  in_world_M_box_ = pinocchio::SE3(in_world_rot_box_quat.toRotationMatrix(), in_world_translation_world_to_box);

  // Set remaining attributes requiring switch to Eigen containers.
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
  initialize_in_box_M_compartment_map(params.box_width, params.box_length, params.object_height_while_placing_in_box);
}

void PlannerManager::initialize_in_box_M_compartment_map(const double &box_width, const double &box_length,
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

void PlannerManager::build_frame_maps_from_params(joint_trajectory_publisher::Params &params) {
  // Set objects frame related maps.
  for (std::string &object_frame : params.objects_frame) {
    std::vector<double> in_grasp_translation_object_to_grasp_vec =
        params.objects_frame_map.at(object_frame).in_grasp_translation_object_to_grasp;
    Eigen::Vector3d in_grasp_translation_object_to_grasp = Eigen::Map<Eigen::Vector3d>(
        in_grasp_translation_object_to_grasp_vec.data(), in_grasp_translation_object_to_grasp_vec.size());
    in_grasp_translation_object_to_grasp_map_[object_frame] = in_grasp_translation_object_to_grasp;
    objects_to_grasp_from_top_map_[object_frame] = params.objects_frame_map.at(object_frame).grasp_from_top;
  }

  // Set box frames translation map.
  for (std::string &box_frame : params.box_frames) {
    std::vector<double> in_box_frame_translation_box_frame_to_box_center_vec =
        params.box_frames_map.at(box_frame).in_box_frame_translation_box_frame_to_box_center;
    Eigen::Vector3d in_box_frame_translation_box_frame_to_box_center =
        Eigen::Map<Eigen::Vector3d>(in_box_frame_translation_box_frame_to_box_center_vec.data(),
                                    in_box_frame_translation_box_frame_to_box_center_vec.size());
    in_box_frame_translation_box_frame_to_box_center_map_[box_frame] = in_box_frame_translation_box_frame_to_box_center;
  }
}

pinocchio::SE3 PlannerManager::get_most_recent_in_parent_M_child(const std::string &parent_frame,
                                                                 const std::string &child_frame) const {
  geometry_msgs::msg::TransformStamped stamped_transform =
      tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
  return transform_msg_to_SE3(stamped_transform);
}

void PlannerManager::update_detection_map(const std::map<std::string, Detection> &det_status_map) {
  std::lock_guard<std::mutex> guard(detection_map_mutex_);
  detection_map_ = det_status_map;
}

std::optional<Detection> PlannerManager::get_first_detection_in_fov(const std::vector<std::string> &frames) {
  std::lock_guard<std::mutex> guard(detection_map_mutex_);
  for (const std::string &frame : frames) {
    if (detection_map_[frame].is_in_fov) {
      return detection_map_[frame];
    }
  }
  return std::nullopt;
}

void PlannerManager::do_search_object_action() {
  // Determine for which frames we are gonna search detections in tf.
  std::vector<std::string> objects_frame;
  if (state_ == SEARCHING_OBJECTS)
    objects_frame = objects_frame_;
  else
    objects_frame = {object_detection_to_sort_.frame};

  // Check for a detection currently in fov, set object_detection_to_sort_ if we found one.
  std::optional<Detection> detection_in_fov = get_first_detection_in_fov(objects_frame);
  if (detection_in_fov.has_value()) {
    object_detection_to_sort_ = detection_in_fov.value();
    RCLCPP_INFO(logger_, "Object %s is in fov.", object_detection_to_sort_.frame.c_str());

    // start searching objects at next location next time.
    if (state_ == SEARCHING_OBJECTS)
      set_object_search_to_next_location();
  } else {
    RCLCPP_INFO(logger_, "didn't found objects, restart state actions");
    restart_state_actions();
  }
}

void PlannerManager::do_search_box_action() {
  std::optional<Detection> detection_in_fov = get_first_detection_in_fov(box_frames_);
  if (detection_in_fov.has_value()) {
    // Compute in_base_M_box with regards to the tag we found.
    in_base_M_box_ = detection_in_fov.value().in_base_M_frame.value();
    in_base_M_box_.translation() =
        in_base_M_box_.translation() +
        in_base_M_box_.rotation() *
            in_box_frame_translation_box_frame_to_box_center_map_[detection_in_fov.value().frame];

    // Compute in_world_M_box.
    pinocchio::SE3 in_world_M_base = get_most_recent_in_parent_M_child(world_frame_, base_frame_);
    in_world_M_box_ = in_world_M_base * in_base_M_box_;
  } else {
    RCLCPP_INFO(logger_, "didn't found the box, restart state actions");
    restart_state_actions();
  }
}

bool PlannerManager::base_waypoints_published() const { return base_waypoints_published_; }

std::vector<Eigen::VectorXd> PlannerManager::get_base_waypoints() const {
  // Move to next location with respect to current box position to search objects.
  if (state_ == SEARCHING_OBJECTS) {
    Eigen::VectorXd base_waypoint = base_waypoint_vec_[base_waypoint_vec_idx_];
    base_waypoint += in_world_M_box_.translation();
    return {base_waypoint};
  }

  // Set orientation goal if robot is in front of the box, navigate to be in front of box otherwise.
  else if (state_ == SEARCHING_BOX) {
    if (std::abs(base_pose_[1] - in_world_M_box_.translation()[1]) < search_box_y_dist_threshold_)
      return {Eigen::Vector3d(base_pose_[0], base_pose_[1], M_PI)};
    else
      return {Eigen::Vector3d(base_pose_[0], in_world_M_box_.translation()[1], M_PI)};
  }

  // Set orientation goal if robot is close enough to the box, navigate to be close enough to the box otherwise.
  else if (state_ == PLACING) {
    pinocchio ::SE3 in_world_M_base = get_most_recent_in_parent_M_child(world_frame_, base_frame_);
    if ((in_world_M_box_.inverse() * in_world_M_base).translation().norm() < box_dist_while_placing_)
      return {Eigen::Vector3d(base_pose_[0], base_pose_[1], M_PI)};
    else {
      return {Eigen::Vector3d(box_dist_while_placing_ + in_world_M_box_.translation()[0],
                              in_world_M_box_.translation()[1], M_PI)};
    }
  }

  // Function called in unexpected state.
  else {
    RCLCPP_ERROR(logger_, "Retrieving base waypoints in unexpected state  %s", get_state_as_string(state_).c_str());
    return {base_pose_};
  }
}

void PlannerManager::update_nav_action(const rclcpp_action::ResultCode &nav_result) {
  switch (nav_result) {
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      base_waypoints_published_ = false;
      last_nav_succeed = true;
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      break;
    default:
      base_waypoints_published_ = false;
      last_nav_succeed = false;
      action_time_ = 0.;
      break;
    }
  }
}

void PlannerManager::set_base_waypoints_published(const bool &base_waypoints_published) {
  base_waypoints_published_ = base_waypoints_published;
}

bool PlannerManager::action_is_finished(const Action &action, const rclcpp_action::ResultCode &nav_result) const {
  if ((action.type == MOVE_GRIPPER) || (action.type == SEARCH_OBJECT) || (action.type == SEARCH_BOX) ||
      (action.type == MOVE_BASE && nav_result == rclcpp_action::ResultCode::SUCCEEDED) ||
      (action.type == MOVE_ARM && trajectory_ready_ && goal_pose_achieved()))
    return true;
  if (action.type == WAIT) {
    if (const double *value = std::get_if<double>(&action.value)) {
      if (action_time_ >= *value)
        return true;
    } else {
      RCLCPP_ERROR(logger_, "Stored wrong variant value type %s for Wait action.", action.get_value_type_name());
      throw std::runtime_error("Got wrong type for wait action value");
    }
  }
  return false;
}

void PlannerManager::do_action_first_time_in_steps() {
  if (actions_.size() > 0) {
    Action action = get_current_action();
    if (action.type == SEARCH_OBJECT)
      do_search_object_action();
    if (action.type == SEARCH_BOX)
      do_search_box_action();
    if (action.type == MOVE_ARM) {
      if (trajectory_computing_thread_.joinable())
        trajectory_computing_thread_.join();
      trajectory_computing_thread_ = std::thread(&PlannerManager::compute_trajectory, this);
    }
  }
}

void PlannerManager::reset_actions_attributes() {
  action_time_ = 0.;
  base_waypoints_published_ = false;
  start_new_action_ = true;
  trajectory_ready_ = false;
}

void PlannerManager::update_actions_status(const rclcpp_action::ResultCode &nav_result) {
  Action action = get_current_action();
  if (action.type == MOVE_BASE) {
    update_nav_action(nav_result);
  }
  if (action_is_finished(action, nav_result)) {
    actions_.erase(actions_.begin());
    action_time_ = 0.;
    reset_actions_attributes();
  }
}

std::vector<Action> PlannerManager::get_state_actions() const {
  std::vector<Action> actions = {};
  switch (state_) {
  case SEARCHING_OBJECTS:
    actions.push_back({MOVE_GRIPPER, 1, closed_gripper_position_});
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
    actions.push_back({MOVE_GRIPPER, 2, open_gripper_position_});
    break;
  case GRASPING:
    actions.push_back({MOVE_ARM, 3, std::monostate()});
    actions.push_back({MOVE_GRIPPER, 3, closed_gripper_position_});
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
    actions.push_back({MOVE_GRIPPER, 4, open_gripper_position_});
    actions.push_back({WAIT, 8, wait_duration_after_gripper_moved_});
    if (robot_name_ == "LeKiwi")
      actions.push_back({MOVE_ARM, 6, q_base_moving_});
    break;
  }
  return actions;
}

void PlannerManager::restart_state_actions() {
  current_state_action_were_retrieved_ = false;
  actions_ = {};
  reset_actions_attributes();
}

void PlannerManager::update_state(const Eigen::VectorXd &current_q, const Eigen::VectorXd &base_pose,
                                  const rclcpp_action::ResultCode &nav_result, const rclcpp::Time &ros_time) {
  // Update state of the robot related attributes;
  current_q_ = current_q;
  base_pose_ = base_pose;
  ros_time_ = ros_time;

  // Change state if we retrieved actions and did them all.
  if (actions_.size() == 0 && current_state_action_were_retrieved_) {
    current_state_action_were_retrieved_ = false;
    int new_state_idx = (int(state_) + 1) % state_order_.size();
    if (state_order_[new_state_idx] == SEARCHING_BOX && robot_name_ == "SO-101")
      new_state_idx++;
    state_ = state_order_[new_state_idx];
    RCLCPP_INFO(logger_, "New state : %s", get_state_as_string(state_).c_str());
  }

  // Retrieve state actions.
  if (!current_state_action_were_retrieved_) {
    std::vector<Action> actions = get_state_actions();
    actions_.insert(actions_.end(), actions.begin(), actions.end());
    current_state_action_were_retrieved_ = true;
  }

  if (start_new_action_) {
    do_action_first_time_in_steps();
    start_new_action_ = false;
  }

  // Increase action time if we're not waiting for motion planning to be ready.
  Action action = get_current_action();
  if (action.type != MOVE_ARM || trajectory_ready_)
    action_time_ += state_update_period_;
}

Action PlannerManager::get_current_action() const {
  if (actions_.size() == 0)
    return Action{NONE, 0, std::monostate()};
  return actions_[0];
}

void PlannerManager::compute_trajectory() {
  Action action = get_current_action();
  if (const Eigen::VectorXd *value = std::get_if<Eigen::VectorXd>(&action.value))
    q_waypoints = {*value};
  else if (state_ == SEARCHING_OBJECTS) {
    q_waypoints = {qs_searching_objects_[qs_searching_object_idx_]};
    set_object_search_to_next_location();
  } else if (state_ == GOING_ABOVE_OBJECT_POSE)
    q_waypoints = get_going_above_object_pose_q_waypoints();
  else if (state_ == GRASPING)
    q_waypoints = get_grasping_q_waypoints();
  else if (state_ == PLACING)
    q_waypoints = get_placing_q_waypoints();
  else {
    RCLCPP_ERROR(logger_, "Started motion planning in unexpected state %s", get_state_as_string(state_).c_str());
    q_waypoints = {current_q_};
  }
  motion_planner.set_motion_planning(current_q_, q_waypoints);
  trajectory_ready_ = true;
  RCLCPP_INFO(logger_, "Motion planning ready !");
}

void PlannerManager::set_object_search_to_next_location() {
  qs_searching_object_idx_ = (qs_searching_object_idx_ + 1) % qs_searching_objects_.size();
  if (qs_searching_object_idx_ == 0)
    base_waypoint_vec_idx_ = (base_waypoint_vec_idx_ + 1) % base_waypoint_vec_.size();
}

std::vector<Eigen::VectorXd> PlannerManager::get_going_above_object_pose_q_waypoints() const {
  // Evaluate current yaw angle of gripper with respect to world frame.
  pinocchio::SE3 current_in_base_M_gripper = motion_planner.get_in_base_M_gripper_at_q(current_q_, ee_frame_name_);
  double current_gripper_yaw_angle =
      std::atan2(current_in_base_M_gripper.translation()[1], current_in_base_M_gripper.translation()[0]);

  // Evaluate yaw angle of gripper to be in top of object with respect to world frame.
  pinocchio::SE3 in_base_M_object = object_detection_to_sort_.in_base_M_frame.value();
  double des_gripper_yaw_angle = std::atan2(in_base_M_object.translation()[1], in_base_M_object.translation()[0]);

  // Compute configuration above object pose.
  Eigen::AngleAxisd z_angle_axis_rot =
      Eigen::AngleAxisd(des_gripper_yaw_angle - current_gripper_yaw_angle, Eigen::Vector3d::UnitZ());
  pinocchio::SE3 des_in_base_M_gripper =
      pinocchio::SE3(z_angle_axis_rot.toRotationMatrix() * current_in_base_M_gripper.rotation(),
                     z_angle_axis_rot.toRotationMatrix() * current_in_base_M_gripper.translation());
  std::optional<Eigen::VectorXd> q_above_object =
      motion_planner.get_inverse_kinematic_at_pose(current_q_, des_in_base_M_gripper);

  return {q_above_object.value_or(current_q_)};
}

std::vector<Eigen::VectorXd> PlannerManager::get_grasping_q_waypoints() const {
  // Compute grasp coniguration.
  pinocchio::SE3 in_base_M_grasp =
      get_in_base_M_grasp(object_detection_to_sort_.in_base_M_frame.value(), object_detection_to_sort_.frame);
  std::optional<Eigen::VectorXd> q_grasp = motion_planner.get_inverse_kinematic_at_pose(current_q_, in_base_M_grasp);

  // Compute translation from grasp frame to pre-grasp frame.
  Eigen::VectorXd in_base_translation_grasp_to_pre_grasp;
  bool grasp_from_top = objects_to_grasp_from_top_map_.at(object_detection_to_sort_.frame);
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
  std::optional<Eigen::VectorXd> q_pre_grasp =
      motion_planner.get_inverse_kinematic_at_pose(current_q_, in_base_M_pre_grasp);

  return {q_pre_grasp.value_or(current_q_), q_grasp.value_or(current_q_)};
}

std::vector<Eigen::VectorXd> PlannerManager::get_placing_q_waypoints() const {
  // Compute pre-placing configuration.
  pinocchio::SE3 in_base_M_ee = motion_planner.get_in_base_M_gripper_at_q(current_q_, ee_frame_name_);
  pinocchio::SE3 in_base_M_pre_placing =
      pinocchio::SE3(in_base_M_ee.rotation(), in_base_M_ee.translation() + in_base_translation_gripper_to_pre_placing_);
  std::optional<Eigen::VectorXd> q_pre_placing =
      motion_planner.get_inverse_kinematic_at_pose(current_q_, in_base_M_pre_placing);

  // Compute placing configuration.
  pinocchio::SE3 in_box_M_compartment = in_box_M_compartment_map_.at(object_detection_to_sort_.frame);
  pinocchio::SE3 in_base_M_object_placing = in_base_M_box_ * in_box_M_compartment;
  pinocchio::SE3 in_base_M_placing = get_in_base_M_grasp(in_base_M_object_placing, object_detection_to_sort_.frame);
  std::optional<Eigen::VectorXd> q_placing =
      motion_planner.get_inverse_kinematic_at_pose(current_q_, in_base_M_placing);

  return {q_pre_placing.value_or(current_q_), q_placing.value_or(current_q_)};
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, bool> PlannerManager::get_traj_value_at_t() const {
  return motion_planner.get_traj_value_at_t(action_time_);
}

pinocchio::SE3 PlannerManager::get_in_base_M_grasp(const pinocchio::SE3 &in_base_M_object,
                                                   const std::string &object_frame) const {
  // Compute desired gripper orientation.
  bool grasp_from_top = objects_to_grasp_from_top_map_.at(object_frame);
  Eigen::Matrix3d in_base_rot_grasp;
  double yaw_angle = std::atan2(in_base_M_object.translation()[1], in_base_M_object.translation()[0]);
  if (grasp_from_top) {
    Eigen::AngleAxisd yaw_rot_angle_axis = Eigen::AngleAxisd(-yaw_angle, Eigen::Vector3d::UnitZ());
    in_base_rot_grasp = in_world_rot_top_grasp_quat_.toRotationMatrix() * yaw_rot_angle_axis.toRotationMatrix();
  } else {
    Eigen::AngleAxisd yaw_rot_angle_axis = Eigen::AngleAxisd(-yaw_angle, Eigen::Vector3d::UnitY());
    in_base_rot_grasp = in_world_rot_front_grasp_quat_.toRotationMatrix() * yaw_rot_angle_axis.toRotationMatrix();
  }

  // Compute translation from object frame to gripper frame at grasping position.
  Eigen::Vector3d in_base_translation_object_to_grasp =
      in_base_rot_grasp * in_grasp_translation_object_to_grasp_map_.at(object_frame);

  return pinocchio::SE3(in_base_rot_grasp, in_base_M_object.translation() + in_base_translation_object_to_grasp);
}

bool PlannerManager::goal_pose_achieved() const {
  if (!trajectory_ready_ || action_time_ < motion_planner.get_traj_duration())
    return false;
  for (int joint_idx = 0; joint_idx < nq_; joint_idx++) {
    if (std::abs(q_waypoints.back()[joint_idx] - current_q_[joint_idx]) > joints_des_precision_[joint_idx])
      return false;
  }
  return true;
}

bool PlannerManager::trajectory_ready() const { return trajectory_ready_; }

StateMachine PlannerManager::get_state() const { return state_; }

} // namespace sorting_bot
