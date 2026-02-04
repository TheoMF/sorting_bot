#include <vector>
#include <string>
#include <chrono>
#include <unistd.h>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
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
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(params_.joint_trajectory_topic, joint_trajectory_qos_profile);

      // Action client for navigation
      navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
          this, "/navigate_through_poses");

      // Gripper angle publisher
      if (params_.robot_name == "SO-101")
      {
        so_101_gripper_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(params_.gripper_command_topic, rclcpp::SystemDefaultsQoS());
        lekiwi_gripper_publisher_ = nullptr;
      }
      else if (params_.robot_name == "LeKiwi")
      {
        so_101_gripper_publisher_ = nullptr;
        lekiwi_gripper_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(params_.gripper_command_topic, rclcpp::SystemDefaultsQoS());
      }

      // Joint states subscriber
      rclcpp::QoS joint_states_qos_profile(10);
      joint_states_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      joint_states_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", joint_states_qos_profile, std::bind(&JointTrajectoryPublisher::joint_states_callback, this, std::placeholders::_1));

      // Odometry subscriber
      rclcpp::QoS odometry_qos_profile(10);
      odometry_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      odometry_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom", odometry_qos_profile, std::bind(&JointTrajectoryPublisher::odom_callback, this, std::placeholders::_1));

      // Robot description subscriber
      robot_description_subscriber_ = this->create_subscription<std_msgs::msg::String>(
          "/robot_description", joint_states_qos_profile, std::bind(&JointTrajectoryPublisher::robot_description_callback, this, std::placeholders::_1));

      // Timer to publish joint trajectory
      joint_traj_pub_timer_ = this->create_wall_timer(
          10ms, std::bind(&JointTrajectoryPublisher::joint_traj_pub_callback, this));

      // TF related attributes.
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

      RCLCPP_INFO(this->get_logger(), "finished setup");
    }

  private:
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
      params_ = parameter_listener_->get_params();
      return true;
    }

    void joint_states_callback(const sensor_msgs::msg::JointState &msg)
    {
      std::vector<double> current_q_vec = {};
      for (std::string &joint_name : params_.joint_names)
      {
        int joint_idx = static_cast<int>(std::find(msg.name.begin(), msg.name.end(), joint_name) - msg.name.begin());
        current_q_vec.push_back(msg.position[joint_idx]);
      }
      current_q_ = Eigen::Map<Eigen::VectorXd>(current_q_vec.data(), current_q_vec.size());
      joint_states_callback_ready_ = true;
      RCLCPP_DEBUG(this->get_logger(), "current q %f %f %f %f %f", current_q_vec[0], current_q_vec[1], current_q_vec[2], current_q_vec[3], current_q_vec[4]);
    }

    void robot_description_callback(const std_msgs::msg::String &msg)
    {
      planner_manager_.initialize(tf_buffer_, tf_broadcaster_, msg.data, end_effector_name_, params_);
      robot_description_ready_ = true;
    }

    void nav_goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr &goal_handle)
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

    void nav_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr,
                               const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
    {
      RCLCPP_DEBUG(this->get_logger(), "Nav distance to goal: %f", feedback->distance_remaining);
    }

    void nav_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result)
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

    void odom_callback(const nav_msgs::msg::Odometry &msg)
    {
      base_pose_ = pose_msg_to_base_pose(msg.pose.pose);
    }

    void publish_nav_goal(const std::vector<Eigen::VectorXd> &base_poses)
    {
      if (!navigation_action_client_->wait_for_action_server(std::chrono::seconds(1)))
      {
        RCLCPP_WARN(this->get_logger(), "navigation server not available");
        planner_manager_.set_goal_base_pose_published(false);
        return;
      }

      // Create navigation's waypoints msg.
      auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
      for (const auto &base_pose : base_poses)
      {
        goal_msg.poses.push_back(base_pose_to_pose_msg(base_pose));
      }

      // Add callbacks to track navigation progress.
      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
      send_goal_options.goal_response_callback =
          std::bind(&JointTrajectoryPublisher::nav_goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback =
          std::bind(&JointTrajectoryPublisher::nav_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback =
          std::bind(&JointTrajectoryPublisher::nav_result_callback, this, std::placeholders::_1);
      navigation_action_client_->async_send_goal(goal_msg, send_goal_options);
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
      pose_msg.header.frame_id = params_.world_frame;
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
      if (params_.robot_name == "SO-101")
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
      else if (params_.robot_name == "LeKiwi")
      {
        trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
        joint_trajectory_msg.joint_names = {"gripper"};
        trajectory_msgs::msg::JointTrajectoryPoint curr_point;
        curr_point.positions = {angle};
        joint_trajectory_msg.points = {curr_point};
        lekiwi_gripper_publisher_->publish(joint_trajectory_msg);
      }
    }

    void handle_nav_goal_publication()
    {
      if (!planner_manager_.goal_base_pose_published())
      {
        std::vector<Eigen::VectorXd> base_poses = planner_manager_.get_base_goal_waypoints();
        planner_manager_.set_goal_base_pose_published(true);
        if (last_sent_base_waypoints[0] == base_poses[0] && planner_manager_.last_nav_succeed)
          nav_result_ = rclcpp_action::ResultCode::SUCCEEDED;
        else
        {
          for (Eigen::VectorXd &base_pose : base_poses)
            RCLCPP_INFO(this->get_logger(), "publishing base waypoints x: %f y: %f yaw: %f", base_pose[0], base_pose[1], base_pose[2]);
          last_sent_base_waypoints = base_poses;
          publish_nav_goal(base_poses);
        }
      }
    }

    void update_traj_references()
    {
      if (planner_manager_.trajectory_ready())
      {
        std::tuple<Eigen::VectorXd, Eigen::VectorXd, bool> traj_value = planner_manager_.get_traj_value_at_t();
        q_traj_ = std::get<0>(traj_value);
        q_dot_traj_ = std::get<1>(traj_value);
        over_traj_total_duration_ = std::get<2>(traj_value);
        traj_ready_ = true;
      }
    }

    void do_actions()
    {
      std::tuple<ActionType, double> action = planner_manager_.get_current_action();
      StateMachine state = planner_manager_.get_state();
      ActionType action_type = std::get<0>(action);
      RCLCPP_DEBUG(this->get_logger(), "state idx : %d action idx : %d", static_cast<int>(state), static_cast<int>(action_type));
      switch (action_type)
      {
      case MOVE_JAW:
      {
        send_gripper_pose_msg(std::get<1>(action));
        break;
      }
      case MOVE_BASE:
      {
        handle_nav_goal_publication();
        break;
      }
      case SET_MOVE_BASE_Q:
      {
        update_traj_references();
        break;
      }
      case FOLLOW_TRAJ:
      {
        update_traj_references();
        break;
      }
      }

      // Add log if we started an action.
      if (action_type != last_action_)
      {
        if (last_action_ == SET_MOVE_BASE_Q || last_action_ == FOLLOW_TRAJ)
        {
          std::cout << "finished traj with curr q " << current_q_ << " q_ref_ " << q_ref_ << " integ " << integrated_q_err_ << std::endl;
          RCLCPP_INFO(this->get_logger(), "add_integration_in_err %d %d %d %d %d", add_integration_in_err[0], add_integration_in_err[1], add_integration_in_err[2], add_integration_in_err[3], add_integration_in_err[4]);
        }

        last_action_ = action_type;
        RCLCPP_INFO(this->get_logger(), "start new action index %d", static_cast<int>(action_type));
      }

      // Update actions status.
      planner_manager_.update_actions_status(nav_result_);
      if (nav_result_ != rclcpp_action::ResultCode::UNKNOWN)
        nav_result_ = rclcpp_action::ResultCode::UNKNOWN;
    }

    double sign(const double &val)
    {
      if (val > 0.0)
        return 1.0;
      else if (val < 0.0)
        return -1.0;
      return 0.0;
    }

    Eigen::VectorXd compute_q_ref()
    {
      Eigen::VectorXd q_ref = q_traj_;
      Eigen::VectorXd q_err = q_ref - current_q_;
      std::vector<int> curr_add_integration_in_err = {0, 0, 0, 0, 0};
      bool changed_values = false;
      for (int idx = 0; idx < nq_; idx++)
      {
        q_ref[idx] += sign(q_err[idx]) * params_.friction_compensation[idx];
        if (std::abs(q_err[idx]) > params_.joints_des_precision[idx] * 0.1)
        {
          curr_add_integration_in_err[idx] = 1;
          changed_values = true;
          if (over_traj_total_duration_)
            integrated_q_err_[idx] += params_.integration_coeffs_after_traj[idx] / params_.rate * q_err[idx];
          else
            integrated_q_err_[idx] += params_.integration_coeffs_during_traj[idx] / params_.rate * q_err[idx];
        }
      }
      if (changed_values)
        add_integration_in_err = curr_add_integration_in_err;
      return q_ref + integrated_q_err_;
    }

    void joint_traj_pub_callback()
    {
      // wait for joint states callback and robot description
      if (joint_states_callback_ready_ == false || robot_description_ready_ == false)
        return;
      planner_manager_.update_state(current_q_, base_pose_, nav_result_, this->get_clock()->now());
      do_actions();

      // Get current state
      std::tuple<ActionType, double> action = planner_manager_.get_current_action();
      ActionType action_type = std::get<0>(action);
      if (last_q_ == Eigen::VectorXd::Zero(nq_))
      {
        last_q_ = current_q_;
        q_ref_ = current_q_;
      }
      if ((action_type == FOLLOW_TRAJ || action_type == SET_MOVE_BASE_Q) && traj_ready_)
      {
        last_q_ref_ = q_ref_;
        q_ref_ = compute_q_ref();
      }
      else
      {
        integrated_q_err_ += last_q_ - current_q_;
        traj_ready_ = false;
      }
      last_q_ = current_q_;

      trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
      joint_trajectory_msg.joint_names = params_.joint_names;
      trajectory_msgs::msg::JointTrajectoryPoint curr_point;
      std::vector<double> q_ref_vec(q_ref_.data(), q_ref_.data() + q_ref_.size());
      std::vector<double> q_dot_ref_vec(q_dot_traj_.data(), q_dot_traj_.data() + q_dot_traj_.size());
      curr_point.positions = q_ref_vec;
      curr_point.velocities = q_dot_ref_vec;

      joint_trajectory_msg.points = {curr_point};
      publisher_->publish(joint_trajectory_msg);
    }

    int nq_ = 5;
    Eigen::VectorXd current_q_, q_ref_ = Eigen::VectorXd::Zero(nq_), last_q_ref_ = Eigen::VectorXd::Zero(nq_), q_traj_ = Eigen::VectorXd::Zero(nq_), q_dot_traj_ = Eigen::VectorXd::Zero(nq_),
                                base_pose_ = Eigen::VectorXd::Zero(3), integrated_q_err_ = Eigen::VectorXd::Zero(nq_), last_q_ = Eigen::VectorXd::Zero(nq_);

    // ROS params.
    std::shared_ptr<joint_trajectory_publisher::ParamListener>
        parameter_listener_;
    joint_trajectory_publisher::Params params_;
    std::vector<int> add_integration_in_err = {0, 0, 0, 0, 0};

    std::vector<Eigen::VectorXd> q_waypoints, last_sent_base_waypoints = {Eigen::Vector3d(-10., -10., 0.)};
    rclcpp::TimerBase::SharedPtr joint_traj_pub_timer_;
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

    PlannerManager planner_manager_;
    bool over_traj_total_duration_ = false;
    ActionType last_action_ = NONE;
    std::string end_effector_name_ = "gripper_frame_link";
    std::thread trajectory_computing_thread_;
    bool joint_states_callback_ready_ = false, robot_description_ready_ = false, traj_ready_ = false;
  };
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joint_trajectory_publisher::JointTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}