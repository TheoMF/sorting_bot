#include <vector>

#include "tf2_ros/buffer.h"

#include "sorting_bot/motion_planner.hpp"

enum StateMachine
{
  GOING_TO_QINIT,
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
  }

  void initialize(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer, std::string urdf, std::string ee_frame_name, joint_trajectory_publisher::Params params)
  {
    tf_buffer_ = tf_buffer;
    motion_planner.initialize(urdf, ee_frame_name, params);
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
        // std::cout << "didn't found transform for object "<< object_frame.c_str()<< " error : " <<ex.what()<<std::endl;
      }
    }
    return found_transform;
  }

  bool goal_pose_achieved(const Eigen::VectorXd &q, Eigen::VectorXd q_goal, double des_precision)
  {
    return (q_goal - q).norm() < des_precision;
  }

  Eigen::VectorXd get_current_goal()
  {
    return q_waypoints.back();
  }

  bool action_is_finished(Eigen::VectorXd q, double time, std::tuple<ActionType, double> &current_action)
  {
    ActionType action_type = std::get<0>(current_action);
    if ((action_type == WAIT && time >= std::get<1>(current_action)) || (action_type == MOVE_JAW) || (action_type == FOLLOW_TRAJ && trajectory_ready_ && goal_pose_achieved(q, get_current_goal(), 5e-3)))
      return true;
    return false;
  }

  std::vector<std::tuple<ActionType, double>> update_state(Eigen::VectorXd q)
  {
    q_ = q;
    std::vector<std::tuple<ActionType, double>> action = {};
    if (state_ == GOING_TO_QINIT)
    {

      if (first_time_)
      {
        action.push_back(std::make_tuple(MOVE_JAW, 1.5));
        trajectory_computing_thread_ = std::thread(&PlannerManager::compute_trajectory, this);
        action.push_back(std::make_tuple(FOLLOW_TRAJ, 2.));
      }
      first_time_ = false;
      if (trajectory_ready_)
      {
        if (goal_pose_achieved(q, q_waypoints.back(), 5e-3))
        {
          if (first_time_reach_q_init)
          {
            action.push_back(std::make_tuple(WAIT, 2.0));
            first_time_reach_q_init = false;
          }
          else
          {
            bool object_found = find_next_object_to_grasp_transform();
            if (object_found)
            {
              std::cout << "set GOING_TO_GRASP_POSE state" << std::endl;
              state_ = GOING_TO_GRASP_POSE;
              trajectory_ready_ = false;
              trajectory_computing_thread_.join();
              trajectory_computing_thread_ = std::thread(&PlannerManager::compute_trajectory, this);
              action.push_back(std::make_tuple(FOLLOW_TRAJ, 3.));
            }
          }
        }
      }
    }
    else if (state_ == GOING_TO_GRASP_POSE)
    {
      if (trajectory_ready_)
      {
        if (goal_pose_achieved(q, q_waypoints.back(), 5e-3))
        {
          std::cout << "set PLACING state" << std::endl;
          state_ = PLACING;
          trajectory_ready_ = false;
          trajectory_computing_thread_.join();
          trajectory_computing_thread_ = std::thread(&PlannerManager::compute_trajectory, this);
          action.push_back(std::make_tuple(WAIT, 0.7));
          action.push_back(std::make_tuple(MOVE_JAW, 0.2));
          action.push_back(std::make_tuple(WAIT, 0.3));
          action.push_back(std::make_tuple(FOLLOW_TRAJ, 1.));
        }
      }
    }
    else if (state_ == PLACING)
    {
      if (trajectory_ready_)
      {
        if (goal_pose_achieved(q, q_waypoints.back(), 5e-3))
        {
          std::cout << "set GOING_TO_QINIT state" << std::endl;
          state_ = GOING_TO_QINIT;
          first_time_reach_q_init = true;
          trajectory_ready_ = false;
          trajectory_computing_thread_.join();
          trajectory_computing_thread_ = std::thread(&PlannerManager::compute_trajectory, this);
          action.push_back(std::make_tuple(WAIT, 0.6));
          action.push_back(std::make_tuple(MOVE_JAW, 1.5));
          action.push_back(std::make_tuple(FOLLOW_TRAJ, 2.));
        }
      }
    }
    return action;
  }
  void compute_trajectory()
  {
    if (state_ == GOING_TO_GRASP_POSE)
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
    else if (state_ == GOING_TO_QINIT)
    {
      q_waypoints = {q_init_};
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

  double get_traj_duration()
  {
    return motion_planner.get_traj_duration();
  }

private:
  Eigen::VectorXd q_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  enum StateMachine state_ = GOING_TO_QINIT;
  bool first_time_ = true, trajectory_ready_ = false, first_time_reach_q_init = true;
  MotionPlanner motion_planner;
  std::vector<Eigen::VectorXd> q_waypoints = {};
  std::thread trajectory_computing_thread_;
  std::vector<std::string> objects_frame_, objects_done_ = {};
  std::tuple<std::string, pinocchio::SE3> current_target_;
  Eigen::VectorXd q_waypoint_above_object_, q_waypoint_up_, q_init_;
  std::vector<std::vector<double>> q_types = {};
};