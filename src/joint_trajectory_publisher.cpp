#include <vector>
#include <string>
#include <chrono>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "sorting_bot/planner_manager.hpp"

using namespace std::chrono_literals;
namespace joint_trajectory_publisher
{
  class JointTrajectoryPublisher : public rclcpp::Node
  {
  public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher")
    {
      if (!load_parameters())
      {
        RCLCPP_ERROR(this->get_logger(), "Got issues loading the parameters.");
        return;
      }

      // Joint trajectory publisher
      rclcpp::QoS joint_trajectory_qos_profile(10);
      joint_trajectory_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      joint_trajectory_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(parameters_.joint_trajectory_topic, joint_trajectory_qos_profile);

      // Action client for navigation
      navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
          this, "/navigate_through_poses");

      // Gripper angle publisher
      if (parameters_.robot_name == "SO-101")
      {
        so_101_gripper_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(parameters_.gripper_command_topic, rclcpp::SystemDefaultsQoS());
        lekiwi_gripper_publisher_ = nullptr;
      }
      else if (parameters_.robot_name == "LeKiwi")
      {
        so_101_gripper_publisher_ = nullptr;
        lekiwi_gripper_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(parameters_.gripper_command_topic, rclcpp::SystemDefaultsQoS());
      }

      // Joint states subscriber
      rclcpp::QoS joint_states_qos_profile(10);
      joint_states_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      joint_states_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", joint_states_qos_profile, std::bind(&JointTrajectoryPublisher::topic_callback, this, std::placeholders::_1));

      // Odometry subscriber
      rclcpp::QoS odometry_qos_profile(10);
      odometry_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      odometry_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom", odometry_qos_profile, std::bind(&JointTrajectoryPublisher::odom_callback, this, std::placeholders::_1));

      // Robot description subscriber
      robot_description_subscriber_ = this->create_subscription<std_msgs::msg::String>(
          "/robot_description", joint_states_qos_profile, std::bind(&JointTrajectoryPublisher::robot_description_callback, this, std::placeholders::_1));

      // Timers to publish joint trajectory
      timer_ = this->create_wall_timer(
          10ms, std::bind(&JointTrajectoryPublisher::timer_callback, this));
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
      RCLCPP_INFO(this->get_logger(), "finished setup");
    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState &msg)
    {
      std::vector<double> q_vec = {};
      for (std::string &joint_name : parameters_.joint_names)
      {
        int joint_idx = static_cast<int>(std::find(msg.name.begin(), msg.name.end(), joint_name) - msg.name.begin());
        q_vec.push_back(msg.position[joint_idx]);
      }
      current_q = Eigen::Map<Eigen::VectorXd>(q_vec.data(), q_vec.size());
      ready = true;
      RCLCPP_DEBUG(this->get_logger(), "current q %f %f %f %f %f", q_vec[0], q_vec[1], q_vec[2], q_vec[3], q_vec[4]);
    }

    void odom_callback(const nav_msgs::msg::Odometry &msg)
    {
      base_pose_ = pose_msg_to_base_pose(msg.pose.pose);
    }

    void robot_description_callback(const std_msgs::msg::String &msg)
    {
      planner_manager.initialize(tf_buffer_, tf_broadcaster_, msg.data, end_effector_name_, parameters_);
      planner_ready_ = true;
    }

    Eigen::VectorXd pose_msg_to_base_pose(const geometry_msgs::msg::Pose &msg)
    {
      pinocchio::SE3::Quaternion quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
      double yaw = quat.toRotationMatrix().eulerAngles(0, 1, 2)[2];
      std::vector<double> pose_vec = {msg.position.x, msg.position.y, yaw};
      return Eigen::Map<Eigen::VectorXd>(pose_vec.data(), pose_vec.size());
    }

    geometry_msgs::msg::PoseStamped base_pose_to_pose_msg(const Eigen::VectorXd &base_pose)
    {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = this->get_clock()->now();
      pose_msg.header.frame_id = "odom";
      pose_msg.pose.position.x = base_pose[0];
      pose_msg.pose.position.y = base_pose[1];
      pose_msg.pose.position.z = 0.0;
      Eigen::AngleAxisd angle_axis_rot = Eigen::AngleAxisd(base_pose[2], Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond quat = Eigen::Quaterniond(angle_axis_rot.toRotationMatrix());
      pose_msg.pose.orientation.x = quat.x();
      pose_msg.pose.orientation.y = quat.y();
      pose_msg.pose.orientation.z = quat.z();
      pose_msg.pose.orientation.w = quat.w();
      return pose_msg;
    }

    void send_gripper_pose_msg(const double &angle)
    {
      if (parameters_.robot_name == "SO-101")
      {
        std_msgs::msg::Float64MultiArray gripper_command_msg;
        std_msgs::msg::MultiArrayDimension dim_sol;
        dim_sol.label = "position";
        dim_sol.size = 1;
        dim_sol.stride = 1;
        gripper_command_msg.layout.dim.push_back(dim_sol);
        std_msgs::msg::MultiArrayDimension max_effort_dim;
        gripper_command_msg.data = {angle};
        so_101_gripper_publisher_->publish(gripper_command_msg);
      }
      else if (parameters_.robot_name == "LeKiwi")
      {
        trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
        joint_trajectory_msg.joint_names = {"gripper"};
        trajectory_msgs::msg::JointTrajectoryPoint curr_point;
        curr_point.positions = {angle};
        joint_trajectory_msg.points = {curr_point};
        lekiwi_gripper_publisher_->publish(joint_trajectory_msg);
      }
    }

    bool load_parameters()
    {
      try
      {
        parameter_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Exception thrown during the loading of the parameters: %s \n",
                     e.what());
        return false;
      }
      parameters_ = parameter_listener_->get_params();
      return true;
    }

    void publish_transform(const pinocchio::SE3 &transform, const std::string parent_frame, std::string child_frame)
    {
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header.stamp = this->get_clock()->now();
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

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr &goal_handle)
    {
      if (!goal_handle)
      {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected by navigation server");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by navigation server");
      }
    }

    void feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr,
                           const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
    {
      RCLCPP_DEBUG(this->get_logger(), "Navigation distance remaining: %f", feedback->distance_remaining);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result)
    {
      switch (result.code)
      {
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

    void update_planning()
    {
      // Update state
      bool action_is_finished = false;
      planner_manager.update_state(current_q, base_pose_, nav_result_);
      std::tuple<ActionType, double> action = planner_manager.get_current_action();
      StateMachine planner_state = planner_manager.get_state();
      RCLCPP_DEBUG(this->get_logger(), "Planner state : %s", std::to_string(planner_state).c_str());
      std::string current_action_str;
      ActionType action_type = std::get<0>(action);
      switch (action_type)
      {
      case NONE:
      {
        current_action_str = "none";
        break;
      }
      case MOVE_JAW:
      {
        current_action_str = "move_jaw";
        send_gripper_pose_msg(std::get<1>(action));
        break;
      }
      case MOVE_BASE:
      {

        current_action_str = "move_base";
        if (!planner_manager.goal_base_pose_published())
        {
          std::vector<Eigen::VectorXd> base_poses = planner_manager.get_base_goal_waypoints();
          planner_manager.set_goal_base_pose_published(true);
          // last_sent_base_waypoints[0][0] == base_poses[0][0] && last_sent_base_waypoints[0][1] == base_poses[0][1] && last_sent_base_waypoints[0][2] == base_poses[0][2]
          if (last_sent_base_waypoints[0] == base_poses[0] && planner_manager.last_nav_succeed)
            nav_result_ = rclcpp_action::ResultCode::SUCCEEDED;
          else
          {
            for (Eigen::VectorXd &base_pose : base_poses)
              RCLCPP_INFO(this->get_logger(), "publishing base waypoints x: %f y: %f yaw: %f", base_pose[0], base_pose[1], base_pose[2]);
            last_sent_base_waypoints = base_poses;
            publish_goal_base_poses(base_poses);
          }
        }
        break;
      }
      case WAIT:
      {
        current_action_str = "wait " + std::to_string(std::get<1>(action)) + "s";
        break;
      }

      case FOLLOW_TRAJ:
      {
        if (planner_manager.trajectory_ready())
        {
          current_action_str = "traj ready id " + std::to_string(std::get<1>(action));
          Eigen::VectorXd q_plan = planner_manager.get_configuration_at_t();
          q_traj_ = q_plan;
        }
        else
        {
          current_action_str = "traj not ready id " + std::to_string(std::get<1>(action));
        }
        break;
      }
      default:
        current_action_str = "vision action";
      }
      RCLCPP_DEBUG(this->get_logger(), "Current action %s", current_action_str.c_str());
      if (action_type != last_action_)
      {
        last_action_ = action_type;
        RCLCPP_INFO(this->get_logger(), "start new action %s", current_action_str.c_str());
      }
      planner_manager.update_actions_status(nav_result_);
      if (nav_result_ != rclcpp_action::ResultCode::UNKNOWN)
        nav_result_ = rclcpp_action::ResultCode::UNKNOWN;
      // if configuration trajectory point hasn't been initialized, use current configuration.
      if (q_traj_ == Eigen::VectorXd::Zero(nq_))
        q_traj_ = current_q;
    }

    Eigen::VectorXd compute_q_ref()
    {
      // Get current state
      std::tuple<ActionType, double> action = planner_manager.get_current_action();
      ActionType action_type = std::get<0>(action);

      // Add an integrator on every axis lacking precision if we follow a trajectory.
      if (action_type == FOLLOW_TRAJ)
      {
        Eigen::VectorXd q_err = q_traj_ - current_q;
        for (int idx = 0; idx < nq_; idx++)
        {
          if (std::abs(q_err[idx]) < parameters_.des_precision * 0.7)
          {
            q_err[idx] = 0.0;
          }
        }
        integrated_q_err_ += 1.0 / parameters_.rate * q_err;
        return q_traj_ + parameters_.integration_coeff * integrated_q_err_;
      }
      else
      {
        return q_traj_;
      }
    }

    void publish_goal_base_poses(const std::vector<Eigen::VectorXd> &base_poses)
    {
      if (!navigation_action_client_->wait_for_action_server(std::chrono::seconds(1)))
      {
        RCLCPP_WARN(this->get_logger(), "navigation server not available");
        return;
      }

      // Create navigation's waypoints msg.
      auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
      for (const auto &base_pose : base_poses)
      {
        goal_msg.poses.push_back(base_pose_to_pose_msg(base_pose));
      }

      // Add callback to track navigation progress.
      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
      send_goal_options.goal_response_callback =
          std::bind(&JointTrajectoryPublisher::goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback =
          std::bind(&JointTrajectoryPublisher::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback =
          std::bind(&JointTrajectoryPublisher::result_callback, this, std::placeholders::_1);
      navigation_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void timer_callback()
    {
      // wait for robot configuration and robot description
      if (ready == false || planner_ready_ == false)
        return;
      update_planning();
      Eigen::VectorXd q_ref = compute_q_ref();

      trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
      joint_trajectory_msg.joint_names = parameters_.joint_names;
      trajectory_msgs::msg::JointTrajectoryPoint curr_point;
      std::vector<double> q_ref_vec(q_ref.data(), q_ref.data() + q_ref.size());
      curr_point.positions = q_ref_vec;
      curr_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0};

      joint_trajectory_msg.points = {curr_point};
      publisher_->publish(joint_trajectory_msg);
    }

    int nq_ = 5;
    Eigen::VectorXd current_q, q_traj_ = Eigen::VectorXd::Zero(nq_),
                               base_pose_ = Eigen::VectorXd::Zero(3), integrated_q_err_ = Eigen::VectorXd::Zero(nq_);

    // ROS params.
    std::shared_ptr<joint_trajectory_publisher::ParamListener>
        parameter_listener_;
    joint_trajectory_publisher::Params parameters_;

    std::vector<Eigen::VectorXd> q_waypoints, last_sent_base_waypoints = {Eigen::Vector3d(-10., -10., 0.)};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigation_action_client_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr so_101_gripper_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lekiwi_gripper_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
    rclcpp_action::ResultCode nav_result_ = rclcpp_action::ResultCode::UNKNOWN;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    PlannerManager planner_manager;
    ActionType last_action_ = NONE;
    std::string end_effector_name_ = "gripper_frame_link";
    std::thread trajectory_computing_thread_;
    bool ready = false, planner_ready_ = false;
  };
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joint_trajectory_publisher::JointTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}