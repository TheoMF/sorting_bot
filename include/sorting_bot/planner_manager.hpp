#include <vector>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
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
  MOVE_BASE,
  WAIT,
  FOLLOW_TRAJ,
  SEARCH_OBJECT,
  SEARCH_BOX
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
    double box_width = 0.255, box_length = 0.315;
    Eigen::Quaterniond rot_x_axis_pi_quat(0., 1., 0., 0.);
    for (double i = -1.; i == -1. || i == 1.; i += 2.)
    {
      for (double j = -1.; j == -1. || j == 1.; j += 2.)
      {
        Eigen::Vector3d box_to_compartment_trans(i * box_length / 4.0, 0.1, -box_width / 2.0 + j * box_width / 4.0);
        pinocchio::SE3 in_box_M_compartment(Eigen::Quaterniond(0., 1., 0., 0.), box_to_compartment_trans);
        in_box_M_compartments_.push_back(in_box_M_compartment);
      }
    }
  }

  void initialize(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer, const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster, std::string urdf, std::string ee_frame_name, joint_trajectory_publisher::Params params)
  {
    tf_buffer_ = tf_buffer;
    tf_broadcaster_ = tf_broadcaster;
    motion_planner.initialize(urdf, ee_frame_name, params);
    des_precision_ = params.des_precision;
    robot_name_ = params.robot_name;
    grasping_object_distance_ = params.grasping_object_distance;
    in_object_translation_grasp_ = Eigen::Map<Eigen::VectorXd>(params.in_object_translation_grasp.data(), params.in_object_translation_grasp.size());
    q_waypoint_up_ = Eigen::Map<Eigen::VectorXd>(params.q_waypoint_up.data(), params.q_waypoint_up.size());
    q_init_ = Eigen::Map<Eigen::VectorXd>(params.q_init.data(), params.q_init.size());
    for (std::vector<double> &q_init_vec : q_inits_vec_)
      q_inits_.push_back(Eigen::Map<Eigen::VectorXd>(q_init_vec.data(), q_init_vec.size()));
    for (std::vector<double> &searching_box_q_vec_ : searching_box_qs_vec_)
      searching_box_qs_.push_back(Eigen::Map<Eigen::VectorXd>(searching_box_q_vec_.data(), searching_box_q_vec_.size()));

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

  pinocchio::SE3 get_most_recent_transform(const std::string &parent_frame, const std::string &child_frame)
  {
    geometry_msgs::msg::TransformStamped stamped_transform = tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
    return transform_msg_to_SE3(stamped_transform.transform);
  }

  pinocchio::SE3 get_in_base_M_object(const std::string &parent_frame, const std::string &child_frame)
  {
    std::string cameraFrameRel = "camera";
    geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(cameraFrameRel, child_frame, tf2::TimePointZero);
    pinocchio::SE3 in_camera_M_cardboard = transform_msg_to_SE3(t.transform);
    auto camera_transform_stamp = t.header.stamp;
    geometry_msgs::msg::TransformStamped other_t = tf_buffer_->lookupTransform(parent_frame, cameraFrameRel, camera_transform_stamp);
    pinocchio::SE3 in_base_M_camera = transform_msg_to_SE3(other_t.transform);
    return in_base_M_camera * in_camera_M_cardboard;
  }

  void publish_transform(const pinocchio::SE3 &transform, const std::string parent_frame, std::string child_frame)
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.frame_id = parent_frame;
    transform_msg.child_frame_id = child_frame;

    // set transform msg translation and rotation
    transform_msg.transform.translation.x = transform.translation().x();
    transform_msg.transform.translation.y = transform.translation().y();
    transform_msg.transform.translation.z = transform.translation().z();
    Eigen::Quaterniond q(transform.rotation());
    transform_msg.transform.rotation.x = q.x();
    transform_msg.transform.rotation.y = q.y();
    transform_msg.transform.rotation.z = q.z();
    transform_msg.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(transform_msg);
  }

  bool find_next_object_to_grasp_transform()
  {
    std::string parent_frame = "base_footprint";
    bool found_transform = false;
    std::vector<std::string> objects_frame = {};
    if (state_ == SEARCHING_OBJECTS)
    {
      for (auto &object_frame : objects_frame_)
      {
        if (std::find(objects_done_.begin(), objects_done_.end(), object_frame) == objects_done_.end())
          objects_frame.push_back(object_frame);
      }
    }
    else
    {
      objects_frame.push_back(std::get<0>(current_target_));
    }
    for (auto &object_frame : objects_frame)
    {
      try
      {
        pinocchio::SE3 des_transform = get_most_recent_transform(parent_frame, object_frame);
        des_transform.rotation() = ideal_rot_quat_.toRotationMatrix();
        found_transform = true;
        first_transform = des_transform;

        double y_angle = std::atan2(des_transform.translation()[1], des_transform.translation()[0]);
        Eigen::AngleAxisd y_angle_axis_rot = Eigen::AngleAxisd(-y_angle, Eigen::Vector3d::UnitY());
        des_transform.rotation() = des_transform.rotation() * y_angle_axis_rot.toRotationMatrix();
        sec_transform = des_transform;

        const Eigen::Vector3d in_base_trans = Eigen::Vector3d(in_object_translation_grasp_);
        Eigen::Vector3d in_des_trans = des_transform.rotation() * in_base_trans;
        des_transform.translation() += in_des_trans;

        // publish_transform(des_transform, "base_link", "desired_pose");
        current_target_ = make_tuple(object_frame, des_transform);
        auto val = get_base_goal_waypoints();
        objects_done_.push_back(object_frame);
        std::cout << "found object  " << object_frame.c_str() << std::endl;
        break;
      }
      catch (const tf2::TransformException &ex)
      {
        // std::cout << "didn't found transform for object " << object_frame.c_str() << " error : " << ex.what() << std::endl;
      }
    }
    return found_transform;
  }

  bool find_box_transform()
  {

    std::string parent_frame = "odom", box_frame = "box";
    bool found_transform = false;
    try
    {
      in_world_M_box_ = get_in_base_M_object(parent_frame, box_frame);
      found_transform = true;
      std::cout << "found box  " << std::endl;
    }
    catch (const tf2::TransformException &ex)
    {
      std::cout << "didn't found box transform " << " error : " << ex.what() << std::endl;
    }
    return found_transform;
  }

  void do_search_object_action()
  {
    if (!find_next_object_to_grasp_transform())
    {
      std::cout << "didn't found object, restart state  " << std::endl;
      current_state_action_were_sent_ = false;
    }
  }

  void do_search_box_action()
  {
    // If we didn't found the box, restart state actions.
    if (!find_box_transform())
      current_state_action_were_sent_ = false;
  }

  bool goal_pose_achieved(const Eigen::VectorXd &q)
  {
    if (!trajectory_ready_)
      return false;
    for (int angle_idx = 0; angle_idx < nq_; angle_idx++)
    {
      if (std::abs(q_waypoints.back()[angle_idx] - q[angle_idx]) > des_precision_)
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
    Eigen::Vector3d last_waypoint(0.5, 0., M_PI);
    Eigen::Vector3d dist_to_last_waypoint = last_waypoint - base_pose_;
    double angle_to_last_waypoint = std::atan2(dist_to_last_waypoint[1], dist_to_last_waypoint[0]);
    Eigen::Vector3d first_waypoint(base_pose_[0], base_pose_[1], angle_to_last_waypoint);
    Eigen::Vector3d second_waypoint(last_waypoint[0], last_waypoint[1], angle_to_last_waypoint);
    searching_box_base_waypoints_ = {first_waypoint, second_waypoint, last_waypoint};
    searching_box_base_waypoints_ = {last_waypoint};
  }

  std::vector<Eigen::VectorXd> get_base_goal_waypoints()
  {
    if (state_ == SEARCHING_OBJECTS)
      return base_poses_waypoints_vec_[search_obj_base_waypoints_vec_idx_];
    else if (state_ == GOING_TO_GRASP_POSE)
    {
      pinocchio::SE3 in_base_M_object = std::get<1>(current_target_);
      pinocchio::SE3 in_world_M_base = get_most_recent_transform("odom", "base_footprint");
      pinocchio::SE3 in_world_M_object = in_world_M_base * in_base_M_object;

      double yaw_angle = std::atan2(in_base_M_object.translation()[1], in_base_M_object.translation()[0]);
      std::cout << " current pose " << in_world_M_base.translation() << std::endl;
      std::cout << " object pose " << in_world_M_object.translation() << std::endl;
      std::cout << "yaw angle " << yaw_angle << std::endl;
      Eigen::Vector3d des_in_world_M_base(in_world_M_object.translation()[0] + std::cos(M_PI + yaw_angle) * grasping_object_distance_,
                                          in_world_M_object.translation()[1] + std::sin(M_PI + yaw_angle) * grasping_object_distance_,
                                          base_pose_[2] + yaw_angle);
      return {des_in_world_M_base};
    }
    else if (state_ == SEARCHING_BOX)
      return searching_box_base_waypoints_;
    else if (state_ == PLACING)
      return {Eigen::Vector3d(0.22 + in_world_M_box_.translation()[0], in_world_M_box_.translation()[1], M_PI)};
    std::cout << " should never happen" << std::endl;
    return {Eigen::Vector3d(0.0, 0., 0.0)};
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
        std::cout << "navigation failed, restarting it" << std::endl;
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
        (action_type == SEARCH_OBJECT) || (action_type == SEARCH_BOX) || (action_type == MOVE_BASE && nav_result == rclcpp_action::ResultCode::SUCCEEDED))
      return true;
    if (action_type == FOLLOW_TRAJ)
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
      actions.push_back(std::make_tuple(MOVE_JAW, -0.4));
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 1.));
      // if (robot_name_ == "LeKiwi")
      //   actions.push_back(std::make_tuple(MOVE_BASE, 1.0));
      actions.push_back(std::make_tuple(WAIT, 1.0));
      actions.push_back(std::make_tuple(SEARCH_OBJECT, 1.0));
      break;
    case GOING_TO_GRASP_POSE:
      // actions.push_back(std::make_tuple(MOVE_BASE, 2.0));
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 2.));
      actions.push_back(std::make_tuple(WAIT, 2.0));
      actions.push_back(std::make_tuple(SEARCH_OBJECT, 2.0));
      actions.push_back(std::make_tuple(MOVE_JAW, 1.0));
      break;
    case GRASPING:
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 2.));
      actions.push_back(std::make_tuple(WAIT, 0.5));
      actions.push_back(std::make_tuple(MOVE_JAW, -0.4));
      actions.push_back(std::make_tuple(WAIT, 10000.));
      break;
    case SEARCHING_BOX:
      // actions.push_back(std::make_tuple(MOVE_BASE, 3.0));
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 3.));
      actions.push_back(std::make_tuple(SEARCH_BOX, 2.0));
      break;
    case PLACING:
      // actions.push_back(std::make_tuple(MOVE_BASE, 4.0));
      actions.push_back(std::make_tuple(FOLLOW_TRAJ, 4.));
      actions.push_back(std::make_tuple(MOVE_JAW, 1.0));
      actions.push_back(std::make_tuple(WAIT, 10000.));
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
        if (action_type == SEARCH_OBJECT)
        {
          do_search_object_action();
        }
        if (action_type == SEARCH_BOX)
        {
          do_search_box_action();
        }
        if (action_type == FOLLOW_TRAJ)
        {
          if (trajectory_computing_thread_.joinable())
            trajectory_computing_thread_.join();
          trajectory_computing_thread_ = std::thread(&PlannerManager::compute_trajectory, this);
          std::cout << "new traj will start soon" << std::endl;
        }
        start_new_action_ = false;
      }
    }
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
      if (action_is_finished(q_, time_, action, nav_result))
      {
        actions_.erase(actions_.begin());
        time_ = 0.;
        goal_base_pose_published_ = false;
        start_new_action_ = true;
        trajectory_ready_ = false;
      }
    }
  }

  void update_state(Eigen::VectorXd q, const Eigen::VectorXd &base_pose, const rclcpp_action::ResultCode &nav_result)
  {
    q_ = q;
    base_pose_ = base_pose;
    update_actions(nav_result);
    if (new_transform != pinocchio::SE3::Identity())
    {
      publish_transform(first_transform, "base_footprint", "first_transform");
      publish_transform(sec_transform, "base_footprint", "sec_transform");
      publish_transform(new_transform, "base_footprint", "new_transform");
    }

    //  Change state if we it has a goal and we reach it.
    if (actions_.size() == 0 && current_state_action_were_sent_)
    {
      current_state_action_were_sent_ = false;
      int new_state_idx = (int(state_) + 1) % state_order_.size();
      if (state_order_[new_state_idx] == SEARCHING_BOX && robot_name_ == "SO-101")
        new_state_idx++;
      state_ = state_order_[new_state_idx];
      std::cout << "new state idx " << new_state_idx << std::endl;
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
    if (action_type != FOLLOW_TRAJ || trajectory_ready_)
      time_ += 0.01;
  }

  std::tuple<ActionType, double> get_current_action()
  {
    if (actions_.size() == 0)
      return std::make_tuple(NONE, 0.);
    return actions_[0];
  }

  void compute_trajectory()
  {
    if (state_ == SEARCHING_OBJECTS)
    {
      if (robot_name_ == "SO-101")
        q_waypoints = {q_init_};
      else
      {
        q_waypoints = {q_inits_[q_init_idx_]};
        q_init_idx_ = (q_init_idx_ + 1) % q_inits_.size();
        if (q_init_idx_ == 0)
          search_obj_base_waypoints_vec_idx_ = (search_obj_base_waypoints_vec_idx_ + 1) % base_poses_waypoints_vec_.size();
      }
    }
    else if (state_ == GOING_TO_GRASP_POSE)
    {
      pinocchio::SE3 in_base_M_object = std::get<1>(current_target_);
      double yaw_angle = std::atan2(in_base_M_object.translation()[1], in_base_M_object.translation()[0] - 0.067);
      Eigen::VectorXd des_q = q_;
      des_q[0] = -yaw_angle;
      std::cout << "object_translation \n " << in_base_M_object.translation() << "object at yaw " << yaw_angle << " set q0 at " << des_q[0] << std::endl;
      q_waypoints = {des_q};
    }
    else if (state_ == GRASPING)
    {
      pinocchio::SE3 des_transform = std::get<1>(current_target_);

      new_transform = des_transform;

      Eigen::VectorXd q_goal = motion_planner.get_inverse_kinematic_at_pose(q_, des_transform);
      std::cout << "Object IK res : \n"
                << q_goal << std::endl;
      pinocchio::SE3 above_object_des_transform = des_transform;
      above_object_des_transform.translation().z() += 0.06;
      q_waypoint_above_object_ = motion_planner.get_inverse_kinematic_at_pose(q_, above_object_des_transform);
      q_waypoints = {q_waypoint_above_object_, q_goal};
      Eigen::Vector3d des_trans = des_transform.translation();
    }
    else if (state_ == SEARCHING_BOX)
    {
      q_waypoints = {searching_box_qs_[searching_box_q_idx_]};
      searching_box_q_idx_ = (searching_box_q_idx_ + 1) % searching_box_qs_.size();
      std::cout << " searching box traj will start" << std::endl;
    }
    else if (state_ == PLACING)
    {
      std::cout << "start computing placing traj" << std::endl;
      std::string object = std::get<0>(current_target_);
      auto it = find(objects_frame_.begin(), objects_frame_.end(), object);
      int object_idx = it - objects_frame_.begin();
      std::cout << "object idx " << object_idx << std::endl;
      pinocchio::SE3 in_box_M_compartment = in_box_M_compartments_[object_idx];
      pinocchio::SE3 in_base_link_M_world = get_most_recent_transform("base_footprint", "odom");
      pinocchio::SE3 base_link_M_compartment = in_base_link_M_world * in_world_M_box_ * in_box_M_compartment;
      std::cout << "in_base_link_M_world \n"
                << in_base_link_M_world << "in_world_M_box_ \n"
                << in_world_M_box_ << " y_anglein_box_M_compartment \n"
                << in_box_M_compartment << " base_link_M_compartment\n"
                << base_link_M_compartment << std::endl;
      first_transform = in_base_link_M_world * in_world_M_box_;
      sec_transform = base_link_M_compartment;
      base_link_M_compartment.rotation() = ideal_rot_quat_.toRotationMatrix();
      double y_angle = std::atan2(base_link_M_compartment.translation()[1], base_link_M_compartment.translation()[0]);
      Eigen::AngleAxisd y_angle_axis_rot = Eigen::AngleAxisd(y_angle, Eigen::Vector3d::UnitZ());
      base_link_M_compartment.rotation() *= y_angle_axis_rot.toRotationMatrix();
      new_transform = base_link_M_compartment;
      Eigen::VectorXd q_above_compartment = motion_planner.get_inverse_kinematic_at_pose(q_, base_link_M_compartment);
      // std::vector<double> q_placing_vec = q_types[object_idx];
      // Eigen::VectorXd q_goal = Eigen::Map<Eigen::VectorXd>(q_placing_vec.data(), q_placing_vec.size());
      q_waypoints = {q_waypoint_up_, q_above_compartment};
    }
    else
    {
      std::cout << "try to follow traj in unknown state, state idx is " << static_cast<int>(state_) << std::endl;
      q_waypoints = {q_};
    }
    motion_planner.set_plan(q_, q_waypoints);
    trajectory_ready_ = true;
    std::cout << "finished setting traj" << std::endl;
  }

  std::tuple<Eigen::VectorXd, Eigen::VectorXd> get_traj_value_at_t()
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
  int nq_ = 5;
  Eigen::VectorXd q_, in_object_translation_grasp_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  enum StateMachine state_ = SEARCHING_OBJECTS;
  std::vector<StateMachine> state_order_ = {SEARCHING_OBJECTS, GOING_TO_GRASP_POSE, GRASPING, SEARCHING_BOX, PLACING};
  std::vector<std::tuple<ActionType, double>> actions_ = {};
  std::string robot_name_;
  bool trajectory_ready_ = false, current_state_action_were_sent_ = false, goal_base_pose_published_ = false, start_new_action_ = true;
  double des_precision_, grasping_object_distance_, time_ = 0.;
  MotionPlanner motion_planner;
  std::vector<Eigen::VectorXd> q_waypoints = {}, q_inits_ = {}, searching_box_qs_ = {}, searching_box_base_waypoints_ = {};
  std::vector<pinocchio::SE3> in_box_M_compartments_ = {};
  pinocchio::SE3 in_world_M_box_ = pinocchio::SE3::Identity(), new_transform = pinocchio::SE3::Identity(), first_transform = pinocchio::SE3::Identity(), sec_transform = pinocchio::SE3::Identity();
  std::vector<std::vector<double>> q_inits_vec_ = {{0.0, -0.776194, 0.170272, 1.475690, 1.262466}, {-0.8, -0.776194, 0.170272, 1.475690, 1.262466}, {0.8, -0.776194, 0.170272, 1.475690, 1.262466}, {0.0, -1.814699, 1.656699, 1.313088, 1.262466}};
  std::vector<std::vector<double>> searching_box_qs_vec_ = {{-0.0, -1.796292, 1.561592, 0.368155, 1.475690}, {-0.8, -1.796292, 1.561592, 0.368155, 1.475690}, {0.8, -1.796292, 1.561592, 0.368155, 1.475690}};
  pinocchio::SE3::Quaternion ideal_rot_quat_ = Eigen::Quaternion(0.50762351, -0.56451828, 0.48617564, -0.43581315); // 0.583, -0.500, 0.549, -0.330
  std::vector<std::vector<double>> q_types = {};
  // std::vector<std::vector<Eigen::VectorXd>> base_poses_waypoints_vec_ = {{Eigen::Vector3d(0.0, 0.0, 0.0)}, {Eigen::Vector3d(0.5, 0.0, 0.0)}, {Eigen::Vector3d(0.5, 0.0, M_PI_2), Eigen::Vector3d(0.5, 0.5, M_PI_2)}, {Eigen::Vector3d(0.5, 0.5, M_PI), Eigen::Vector3d(0.0, 0.5, M_PI)}};
  std::vector<std::vector<Eigen::VectorXd>> base_poses_waypoints_vec_ = {{Eigen::Vector3d(0.0, 0.0, 0.0)}, {Eigen::Vector3d(0.5, 0.0, 0.0)}, {Eigen::Vector3d(0.5, 0.5, M_PI_2)}, {Eigen::Vector3d(0.0, 0.5, M_PI)}};
  int search_obj_base_waypoints_vec_idx_ = 0, q_init_idx_ = 0, searching_box_q_idx_ = 0;
  std::thread trajectory_computing_thread_;
  std::vector<std::string> objects_frame_, objects_done_ = {};
  std::tuple<std::string, pinocchio::SE3> current_target_;
  Eigen::VectorXd q_waypoint_above_object_, base_pose_ = Eigen::VectorXd::Zero(3), q_waypoint_up_, q_init_;
};
//-0.56471753,  0.4825263 , -0.37284455,  0.55646449
// 0.53676005, -0.58853859,  0.45401563, -0.40043172
