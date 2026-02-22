#include "sorting_bot/joint_trajectory_publisher.hpp"

namespace sorting_bot {

JointTrajectoryPublisher::JointTrajectoryPublisher() : Node("joint_trajectory_publisher") {
  // Load ROS parameters.
  if (!load_params()) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't load the parameters, stopping the node.");
    throw std::runtime_error("Failed to load parameters");
  }
  initialize_params_related_attributes();

  // QoS definitions.
  rclcpp::QoS qos_joint_trajectory = rclcpp::QoS(rclcpp::KeepLast(10))
                                         .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                                         .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  rclcpp::QoS qos_gripper = rclcpp::QoS(rclcpp::KeepLast(10))
                                .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                                .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  rclcpp::QoS qos_joint_states = rclcpp::QoS(rclcpp::KeepLast(10))
                                     .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                     .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  rclcpp::QoS qos_robot_description = rclcpp::QoS(rclcpp::KeepLast(10))
                                          .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                          .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  rclcpp::QoS qos_odometry = rclcpp::QoS(rclcpp::KeepLast(10))
                                 .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                 .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // Joint states subscriber.
  joint_states_subscriber_ = this->create_subscription<JointState>(
      "/joint_states", qos_joint_states,
      std::bind(&JointTrajectoryPublisher::joint_states_callback, this, std::placeholders::_1));

  // Robot description subscriber
  robot_description_subscriber_ = this->create_subscription<String>(
      "/robot_description", qos_robot_description,
      std::bind(&JointTrajectoryPublisher::robot_description_callback, this, std::placeholders::_1));

  // Odometry subscriber
  odom_subscriber_ = this->create_subscription<Odometry>(
      "/odom", qos_odometry, std::bind(&JointTrajectoryPublisher::odom_callback, this, std::placeholders::_1));

  // Joint trajectory publisher
  joint_trajectory_publisher_ =
      this->create_publisher<JointTrajectory>(params_.joint_trajectory_topic, qos_joint_trajectory);

  // Gripper position publisher
  if (robot_name_ == "SO-101") {
    so_101_gripper_publisher_ = this->create_publisher<Float64MultiArray>(params_.gripper_command_topic, qos_gripper);
    lekiwi_gripper_publisher_ = nullptr;
  } else if (robot_name_ == "LeKiwi") {
    so_101_gripper_publisher_ = nullptr;
    lekiwi_gripper_publisher_ = this->create_publisher<JointTrajectory>(params_.gripper_command_topic, qos_gripper);
  }

  // Action client for navigation.
  navigation_action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "/navigate_through_poses");

  // Timer to publish joint trajectory.
  joint_traj_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / rate_)),
                                                  std::bind(&JointTrajectoryPublisher::joint_traj_pub_callback, this));

  // Timer to update detections status.
  detections_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&JointTrajectoryPublisher::detections_update_callback, this));

  // TF related attributes.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

bool JointTrajectoryPublisher::load_params() {
  try {
    parameter_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception thrown during the loading of the parameters: %s \n", e.what());
    return false;
  }
  params_ = parameter_listener_->get_params();
  return true;
}

void JointTrajectoryPublisher::initialize_params_related_attributes() {
  // Fill detections status map.
  for (std::string &object_frame : params_.objects_frame)
    detection_map_.insert({object_frame, Detection(params_.hand_camera_frame, object_frame)});
  for (std::string &box_frame : params_.box_frames)
    detection_map_.insert({box_frame, Detection(params_.base_camera_frame, box_frame)});

  // Initialize remaining attributes from parameters.
  rate_ = params_.common.rate;
  robot_name_ = params_.common.robot_name;
  world_frame_ = params_.common.world_frame;
  base_frame_ = params_.common.base_frame;
  joint_names_ = params_.joint_names;
  integration_coeffs_during_traj_ = params_.integration_coeffs_during_traj;
  integration_coeffs_after_traj_ = params_.integration_coeffs_after_traj;
  friction_compensation_ = params_.friction_compensation;
  nq_ = params_.common.nq;
  q_dot_traj_ = Eigen::VectorXd::Zero(nq_);
  integrated_q_err_ = Eigen::VectorXd::Zero(nq_);
}

void JointTrajectoryPublisher::joint_states_callback(const JointState &msg) {
  std::vector<double> current_q_vec = {};
  for (std::string &joint_name : joint_names_) {
    auto joint_name_it = std::find(msg.name.begin(), msg.name.end(), joint_name);
    int joint_idx = static_cast<int>(std::distance(msg.name.begin(), joint_name_it));
    current_q_vec.push_back(msg.position[joint_idx]);
  }
  std::lock_guard<std::mutex> guard(current_q_mutex_);
  current_q_ = Eigen::Map<Eigen::VectorXd>(current_q_vec.data(), current_q_vec.size());
  joint_states_callback_ready_ = true;
}

void JointTrajectoryPublisher::robot_description_callback(const String &msg) {
  planner_manager_.initialize(tf_buffer_, tf_broadcaster_, msg.data, params_);
  robot_description_ready_ = true;
}

void JointTrajectoryPublisher::nav_goal_response_callback(
    const rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::SharedPtr &goal_handle) const {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by navigation server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by navigation server");
  }
}

void JointTrajectoryPublisher::nav_result_callback(
    const rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::WrappedResult &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Navigation succeed");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_INFO(this->get_logger(), "Navigation was aborted");
    break;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_INFO(this->get_logger(), "Navigation was canceled");
    break;
  default:
    RCLCPP_INFO(this->get_logger(), "Unknown navigation result");
  }
  nav_result_ = result.code;
}

void JointTrajectoryPublisher::odom_callback(const Odometry &msg) { base_pose_ = pose_msg_to_base_pose(msg.pose.pose); }

void JointTrajectoryPublisher::publish_nav_goal(const std::vector<Eigen::VectorXd> &base_poses) {
  if (!navigation_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "navigation server not available");
    planner_manager_.set_base_waypoints_published(false);
    return;
  }

  // Create navigation's waypoints msg.
  auto goal_msg = NavigateThroughPoses::Goal();
  for (const auto &base_pose : base_poses)
    goal_msg.poses.push_back(base_pose_to_pose_msg(base_pose, world_frame_, this->get_clock()->now()));

  // Add callbacks to track navigation progress.
  auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&JointTrajectoryPublisher::nav_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
      std::bind(&JointTrajectoryPublisher::nav_result_callback, this, std::placeholders::_1);
  navigation_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void JointTrajectoryPublisher::send_gripper_pose_msg(const double &gripper_pose) const {
  if (robot_name_ == "SO-101") {
    Float64MultiArray gripper_command_msg;
    MultiArrayDimension dim_sol;
    dim_sol.label = "position";
    dim_sol.size = 1;
    dim_sol.stride = 1;
    gripper_command_msg.layout.dim.push_back(dim_sol);
    gripper_command_msg.data = {gripper_pose};
    so_101_gripper_publisher_->publish(gripper_command_msg);
  } else if (robot_name_ == "LeKiwi") {
    JointTrajectory joint_trajectory_msg;
    joint_trajectory_msg.joint_names = {"gripper"};
    JointTrajectoryPoint curr_point;
    curr_point.positions = {gripper_pose};
    joint_trajectory_msg.points = {curr_point};
    lekiwi_gripper_publisher_->publish(joint_trajectory_msg);
  }
}

void JointTrajectoryPublisher::send_joint_trajectory_msg(const Eigen::VectorXd &q_ref,
                                                         const Eigen::VectorXd &q_dot_ref) const {
  // Switch input trajectory point to std::vector.
  std::vector<double> q_ref_vec(q_ref.data(), q_ref.data() + q_ref.size());
  std::vector<double> q_dot_ref_vec(q_dot_ref.data(), q_dot_ref.data() + q_dot_ref.size());

  // Create joint trajectory point msg.
  JointTrajectoryPoint joint_trajectory_point_msg;
  joint_trajectory_point_msg.positions = q_ref_vec;
  joint_trajectory_point_msg.velocities = q_dot_ref_vec;

  // Create and send joint trajectory msg.
  JointTrajectory joint_trajectory_msg;
  joint_trajectory_msg.joint_names = joint_names_;
  joint_trajectory_msg.points = {joint_trajectory_point_msg};
  joint_trajectory_publisher_->publish(joint_trajectory_msg);
}

void JointTrajectoryPublisher::handle_nav_goal_publication() {
  if (!planner_manager_.base_waypoints_published()) {
    std::vector<Eigen::VectorXd> base_waypoints = planner_manager_.get_base_waypoints();
    planner_manager_.set_base_waypoints_published(true);

    // If there hasn't been any update for the base goal, don't start navigation.
    if (last_sent_base_waypoints[0] == base_waypoints[0] && planner_manager_.get_last_nav_succeed())
      nav_result_ = rclcpp_action::ResultCode::SUCCEEDED;

    // If we got a new goal start navigation.
    else {
      for (Eigen::VectorXd &base_waypoint : base_waypoints)
        RCLCPP_INFO(this->get_logger(), "publishing base waypoints x: %f y: %f yaw: %f", base_waypoint[0],
                    base_waypoint[1], base_waypoint[2]);
      last_sent_base_waypoints = base_waypoints;
      publish_nav_goal(base_waypoints);
    }
  }
}

void JointTrajectoryPublisher::update_traj_references() {
  if (planner_manager_.trajectory_ready()) {
    std::tuple<Eigen::VectorXd, Eigen::VectorXd, bool> traj_value = planner_manager_.get_traj_value_at_t();
    q_traj_ = std::get<0>(traj_value);
    q_dot_traj_ = std::get<1>(traj_value);
    over_traj_total_duration_ = std::get<2>(traj_value);
    traj_ready_ = true;
  }
}

void JointTrajectoryPublisher::do_actions() {
  Action action = planner_manager_.get_current_action();
  StateMachine state = planner_manager_.get_state();
  RCLCPP_DEBUG(this->get_logger(), "state : %s action : %s", get_state_as_string(state).c_str(),
               get_action_type_as_string(action.type).c_str());
  switch (action.type) {
  case MOVE_GRIPPER: {
    if (const double *value = std::get_if<double>(&action.value)) {
      send_gripper_pose_msg(*value);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Stored wrong variant value type %s for Move Jaw action.",
                   action.get_value_type_name());
      throw std::runtime_error("Got wrong type for move jaw action value");
    }

    break;
  }
  case MOVE_BASE: {
    handle_nav_goal_publication();
    break;
  }
  case MOVE_ARM: {
    update_traj_references();
    break;
  }
  }

  // Add log if we started an action.
  if (action.type != last_action_) {
    last_action_ = action.type;
    RCLCPP_INFO(this->get_logger(), "start new action : %s", get_action_type_as_string(action.type).c_str());
  }

  // Update actions status.
  planner_manager_.update_actions_status(nav_result_);
  if (nav_result_ != rclcpp_action::ResultCode::UNKNOWN)
    nav_result_ = rclcpp_action::ResultCode::UNKNOWN;
}

void JointTrajectoryPublisher::update_integrated_q_err(const Eigen::VectorXd &q_err) {
  Action action = planner_manager_.get_current_action();
  // If we are in a state that requires to follow a trajectory that is ready, integrate error.
  if (action.type == MOVE_ARM && traj_ready_)
    for (int joint_idx = 0; joint_idx < nq_; joint_idx++) {
      if (over_traj_total_duration_)
        integrated_q_err_[joint_idx] += integration_coeffs_after_traj_[joint_idx] / rate_ * q_err[joint_idx];
      else
        integrated_q_err_[joint_idx] += integration_coeffs_during_traj_[joint_idx] / rate_ * q_err[joint_idx];
    }
  else {
    // Needed to handle motors delays to reach desired configuration.
    integrated_q_err_ += last_q_ - current_q_;
    traj_ready_ = false;
  }
}

Eigen::VectorXd JointTrajectoryPublisher::get_q_ref(const Eigen::VectorXd &q_err) const {
  Eigen::VectorXd friction_compensation = Eigen::VectorXd::Zero(nq_);
  for (int joint_idx = 0; joint_idx < nq_; joint_idx++) {
    if (q_err[joint_idx] > 0)
      friction_compensation[joint_idx] = friction_compensation_[joint_idx];
    else if (q_err[joint_idx] < 0)
      friction_compensation[joint_idx] = -friction_compensation_[joint_idx];
  }
  return q_traj_ + friction_compensation + integrated_q_err_;
}

void JointTrajectoryPublisher::joint_traj_pub_callback() {
  // Wait for joint states callback and robot description.
  if (joint_states_callback_ready_ == false || robot_description_ready_ == false)
    return;

  // Initialize attributes while waiting for first trajectory to be set.
  std::lock_guard<std::mutex> guard(current_q_mutex_);
  if (first_joint_traj_pub_callback_iter_) {
    last_q_ = current_q_;
    q_traj_ = current_q_;
    first_joint_traj_pub_callback_iter_ = false;
  }

  // Update planner manager and perform needed actions.
  planner_manager_.update_state(current_q_, base_pose_, nav_result_, this->get_clock()->now());
  do_actions();

  // Compute and send joint trajectory message.
  Eigen::VectorXd q_err = q_traj_ - current_q_;
  update_integrated_q_err(q_err);
  Eigen::VectorXd q_ref = get_q_ref(q_err);
  send_joint_trajectory_msg(q_ref, q_dot_traj_);

  last_q_ = current_q_;
}

void JointTrajectoryPublisher::detections_update_callback() {
  for (auto &[frame, detection] : detection_map_) {
    try {
      // Determine if detection is active, ie is in fov (field of view).
      TransformStamped in_parent_M_child_stamped =
          tf_buffer_->lookupTransform(detection.parent_frame, detection.child_frame, tf2::TimePointZero);
      bool object_in_fov = in_parent_M_child_stamped.header.stamp != detection.last_stamp.value_or(rclcpp::Time(0, 0));

      // If in fov update detection in map.
      if (object_in_fov) {
        detection.is_in_fov = true;
        detection.last_stamp = in_parent_M_child_stamped.header.stamp;
        TransformStamped in_base_M_frame_stamped =
            tf_buffer_->lookupTransform(base_frame_, detection.child_frame, tf2::TimePointZero);
        detection.in_base_M_child_frame = transform_msg_to_SE3(in_base_M_frame_stamped);
      } else
        detection.is_in_fov = false;

    } catch (const tf2::TransformException &ex) {
      RCLCPP_DEBUG(this->get_logger(), "Didn't found tranform for frame %s , error : %s", frame.c_str(), ex.what());
    }
  }
  planner_manager_.update_detection_map(detection_map_);
}

} // namespace sorting_bot

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sorting_bot::JointTrajectoryPublisher>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(node->get_logger(), "Exception during node runtime : %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
