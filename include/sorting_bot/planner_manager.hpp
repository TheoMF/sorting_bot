#include <vector>

#include "tf2_ros/buffer.h"

#include "sorting_bot/motion_planner.hpp"

enum StateMachine
{
  GOING_TO_QINIT,
  SEARCHING_OBJECTS,
  GOING_TO_GRASP_POSE,
  PLACING
};
enum ActionType
{
  MOVE_JAW,
  WAIT,
  FOLLOW_TRAJ,
  SEARCH_TRANSFORM
};

class PlannerManager
{
public:
  PlannerManager()
  {
    q_ = Eigen::VectorXd::Zero(5);
    q_waypoint_above_object_ = Eigen::VectorXd::Zero(5);
    tf_buffer_ = nullptr;
    robot_name_ = "SO-101";
  }

  void initialize(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer, std::string urdf, std::string ee_frame_name, joint_trajectory_publisher::Params params)
  {
    tf_buffer_ = tf_buffer;
    motion_planner.initialize(urdf, ee_frame_name, params);
    des_precision_ = params.des_precision;
    robot_name_ = params.robot_name;
    q_waypoint_up_ = Eigen::Map<Eigen::VectorXd>(params.q_waypoint_up.data(), params.q_waypoint_up.size());
    q_init_ = Eigen::Map<Eigen::VectorXd>(params.q_init.data(), params.q_init.size());
    objects_frame_ = params.objects_frame;
    for (std::string &object_frame : params.objects_frame)
      q_types.push_back(params.objects_frame_map.at(object_frame).q_placing);
  }

  pinocchio::SE3 transform_msg_to_SE3(const geometry_msgs::msg::Transform &transform)
  {
    Eigen::Quaterniond q(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
    Eigen::Vector3d t(transform.translation.x, transform.translation.y, transform.translation.z);
    pinocchio::SE3 se3(q, t);
    return se3;
  }
  /*
    int evaluate_vision_delay(const std::string &child_frame)
    {
      int detection_count = 0;
      int nanosec_delay = -1;
      geometry_msgs::msg::TransformStamped previous_t = tf_buffer_->lookupTransform("camera", child_frame, tf2::TimePointZero);
      for (int i = 0; i < 5; i++)
      {
        usleep(100000);
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("camera", child_frame, tf2::TimePointZero);
        if (t.header.stamp != previous_t.header.stamp)
          detection_count++;
        previous_t = t;
      }
      if (detection_count > 2)
        nanosec_delay = (this->get_clock()->now() - previous_t.header.stamp).nanoseconds();
      return nanosec_delay;
    }*/

  pinocchio::SE3 get_in_base_M_object(const std::string &parent_frame, const std::string &child_frame)
  {
    std::string cameraFrameRel = "camera";
    // rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(cameraFrameRel, child_frame, tf2::TimePointZero); // now,100ms
    pinocchio::SE3 in_camera_M_cardboard = transform_msg_to_SE3(t.transform);
    // auto current_stamp = this->get_clock()->now();
    /*int nanosec_delay = evaluate_vision_delay(child_frame);
    if (nanosec_delay == -1)
    {
      RCLCPP_WARN(this->get_logger(), "object %s transform is old, transform stamp seconds %d, current stamp seconds %.6f ", child_frame.c_str(), t.header.stamp.sec, current_stamp.seconds());
      throw tf2::TransformException("object transform is too old");
    }*/
    auto camera_transform_stamp = t.header.stamp;
    // acamera_transform_stamp.nanosec += nanosec_delay;
    geometry_msgs::msg::TransformStamped other_t = tf_buffer_->lookupTransform(parent_frame, cameraFrameRel, camera_transform_stamp);
    pinocchio::SE3 in_base_M_camera = transform_msg_to_SE3(other_t.transform);
    return in_base_M_camera * in_camera_M_cardboard;
  }

  bool find_next_object_to_grasp_transform()
  {

    std::string parent_frame = "base_link";
    bool found_transform = false;
    for (auto &object_frame : objects_frame_)
    {
      if (std::find(objects_done_.begin(), objects_done_.end(), object_frame) != objects_done_.end())
        continue;
      try
      {
        pinocchio::SE3 des_transform = get_in_base_M_object(parent_frame, object_frame);
        found_transform = true;
        Eigen::Quaterniond rot_x_axis_pi_quat(0., 1., 0., 0.);
        const pinocchio::SE3 rot_pi_x_axis(rot_x_axis_pi_quat, Eigen::Vector3d(0., 0., 0.));
        des_transform = des_transform * rot_pi_x_axis;
        const Eigen::Vector3d in_base_trans = Eigen::Vector3d(0.04, -0.005, 0.02);
        Eigen::Vector3d in_des_trans = des_transform.rotation().inverse() * in_base_trans;
        des_transform.translation() += in_des_trans;
        // publish_transform(des_transform, "base_link", "desired_pose");
        current_target_ = make_tuple(object_frame, des_transform);
        objects_done_.push_back(object_frame);
        std::cout << "found object  " << object_frame.c_str() << std::endl;
        break;
      }
      catch (const tf2::TransformException &ex)
      {
        std::cout << "didn't found transform for object " << object_frame.c_str() << " error : " << ex.what() << std::endl;
      }
    }
    return found_transform;
  }

  bool goal_pose_achieved(const Eigen::VectorXd &q)
  {
    if (!trajectory_ready_)
      return false;
    return (q_waypoints.back() - q).norm() < des_precision_;
  }

  Eigen::VectorXd get_current_goal()
  {
    return q_waypoints.back();
  }

  bool action_is_finished(Eigen::VectorXd q, double time, std::tuple<ActionType, double> &current_action)
  {
    ActionType action_type = std::get<0>(current_action);
    if ((action_type == WAIT && time >= std::get<1>(current_action)) || (action_type == MOVE_JAW) || (action_type == FOLLOW_TRAJ && trajectory_ready_ && goal_pose_achieved(q)))
      return true;
    return false;
  }

  std::vector<std::tuple<ActionType, double>> get_state_actions()
  {
    std::vector<std::tuple<ActionType, double>> actions = {};
    switch (state_)
    {
    case GOING_TO_QINIT:
      actions.push_back(std::make_tuple(MOVE_JAW, 1.5));
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 1.));
      actions.push_back(std::make_tuple(WAIT, 2.0));
      break;
    case GOING_TO_GRASP_POSE:
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 2.));
      actions.push_back(std::make_tuple(WAIT, 0.7));
      actions.push_back(std::make_tuple(MOVE_JAW, 0.2));
      actions.push_back(std::make_tuple(WAIT, 0.3));
      break;
    case PLACING:
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 3.));
      actions.push_back(std::make_tuple(MOVE_JAW, 1.5));
      actions.push_back(std::make_tuple(WAIT, 0.5));
      break;
    }
    return actions;
  }

  std::vector<std::tuple<ActionType, double>> update_state(Eigen::VectorXd q)
  {
    q_ = q;
    std::vector<std::tuple<ActionType, double>> actions = {};
    if (std::find(moving_state_.begin(), moving_state_.end(), state_) != moving_state_.end() && goal_pose_achieved(q))
    {
      trajectory_ready_ = false;
      current_state_action_were_sent_ = false;
      int new_state_idx = (int(state_) + 1) % 4;
      state_ = state_order_[new_state_idx];
    }
    if (!current_state_action_were_sent_)
    {
      actions = get_state_actions();
      if (std::find(moving_state_.begin(), moving_state_.end(), state_) != moving_state_.end())
      {
        if (trajectory_computing_thread_.joinable())
          trajectory_computing_thread_.join();
        trajectory_computing_thread_ = std::thread(&PlannerManager::compute_trajectory, this);
      }
      current_state_action_were_sent_ = true;
    }

    if (state_ == SEARCHING_OBJECTS)
    {
      bool object_found = find_next_object_to_grasp_transform();
      if (object_found)
      {
        state_ = GOING_TO_GRASP_POSE;
        trajectory_ready_ = false;
        current_state_action_were_sent_ = false;
      }
    }
    return actions;
  }
  void compute_trajectory()
  {
    if (state_ == GOING_TO_QINIT)
    {
      q_waypoints = {q_init_};
    }
    else if (state_ == GOING_TO_GRASP_POSE)
    {
      pinocchio::SE3 des_transform = std::get<1>(current_target_);
      Eigen::VectorXd q_goal = motion_planner.get_inverse_kinematic_at_pose(q_, des_transform);
      pinocchio::SE3 above_object_des_transform = des_transform;
      above_object_des_transform.translation().z() += 0.06;
      q_waypoint_above_object_ = motion_planner.get_inverse_kinematic_at_pose(q_, above_object_des_transform);
      q_waypoints = {q_waypoint_above_object_, q_goal};
      Eigen::Vector3d des_trans = des_transform.translation();
    }
    else if (state_ == PLACING)
    {
      state_ = PLACING;
      std::string object = std::get<0>(current_target_);
      auto it = find(objects_frame_.begin(), objects_frame_.end(), object);
      int object_idx = it - objects_frame_.begin();
      std::vector<double> q_placing_vec = q_types[object_idx];
      Eigen::VectorXd q_goal = Eigen::Map<Eigen::VectorXd>(q_placing_vec.data(), q_placing_vec.size());
      q_waypoints = {q_waypoint_above_object_, q_waypoint_up_, q_goal};
    }
    motion_planner.set_plan(q_, q_waypoints);
    trajectory_ready_ = true;
    std::cout << "finished setting traj" << std::endl;
  }

  Eigen::VectorXd get_configuration_at_t(const double &time)
  {
    Eigen::VectorXd q = motion_planner.get_configuration_at_t(time);
    return q;
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

private:
  Eigen::VectorXd q_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  enum StateMachine state_ = GOING_TO_QINIT;
  std::vector<StateMachine> moving_state_ = {GOING_TO_QINIT, GOING_TO_GRASP_POSE, PLACING},
                            state_order_ = {GOING_TO_QINIT, SEARCHING_OBJECTS, GOING_TO_GRASP_POSE, PLACING};
  std::string robot_name_;
  bool first_time_ = true, trajectory_ready_ = false, first_time_reach_q_init = true, current_state_action_were_sent_ = false;
  double des_precision_;
  MotionPlanner motion_planner;
  std::vector<Eigen::VectorXd> q_waypoints = {};
  std::vector<std::vector<double>> q_inits = {{0.322136, 0.018408, -0.678020, 1.65, 1.684311}, {-0.222136, 0.018408, -0.678020, 1.65, 1.684311}, {0.822136, 0.018408, -0.678020, 1.65, 1.684311}};
  std::vector<std::vector<double>> pose_2d_to_try_ = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  int pose_2d_idx_ = 0;
  std::thread trajectory_computing_thread_;
  std::vector<std::string> objects_frame_, objects_done_ = {};
  std::tuple<std::string, pinocchio::SE3> current_target_;
  Eigen::VectorXd q_waypoint_above_object_, q_waypoint_up_, q_init_;
  std::vector<std::vector<double>> q_types = {};
};