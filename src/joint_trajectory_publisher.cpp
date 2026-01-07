#include <vector>
#include <string>
#include <chrono>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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

      // 3d pose goal publisher for mobile robots
      rclcpp::QoS pose_qos_profile(10);
      pose_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      pose_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", pose_qos_profile);

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
          "/joint_states", pose_qos_profile, std::bind(&JointTrajectoryPublisher::topic_callback, this, std::placeholders::_1));

      // Robot description subscriber
      robot_description_subscriber_ = this->create_subscription<std_msgs::msg::String>(
          "/robot_description", joint_states_qos_profile, std::bind(&JointTrajectoryPublisher::robot_description_callback, this, std::placeholders::_1));

      // Timers to publish joint trajectory
      timer_ = this->create_wall_timer(
          10ms, std::bind(&JointTrajectoryPublisher::timer_callback, this));
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
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

    void robot_description_callback(const std_msgs::msg::String &msg)
    {
      planner_manager.initialize(tf_buffer_, msg.data, end_effector_name_, parameters_); //
      planner_ready_ = true;
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

    bool goal_pose_achieved(const Eigen::VectorXd &q, Eigen::VectorXd q_goal, double des_precision)
    {
      return (q_goal - q).norm() < des_precision;
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

    void update_planning()
    {
      // Update state
      bool action_is_finished = false;
      if (actions_.size() > 0)
        action_is_finished = planner_manager.action_is_finished(current_q, time, actions_[0]);

      if (actions_.size() == 0)
      {
        std::vector<std::tuple<ActionType, double>> action = planner_manager.update_state(current_q);
        actions_.insert(actions_.end(), action.begin(), action.end());
      }
      StateMachine planner_state = planner_manager.get_state();
      RCLCPP_DEBUG(this->get_logger(), "Planner state : %s", std::to_string(planner_state).c_str());

      // GET TRANSFORM
      // for (int joint_idx = 0; joint_idx < 5; joint_idx++)
      //  RCLCPP_INFO(this->get_logger(), "q%d %.6f", joint_idx, q[joint_idx]);

      std::string current_action_str;
      if (actions_.size() == 0)
      {
        RCLCPP_DEBUG(this->get_logger(), "No action currently");
      }
      else
      {
        std::tuple<ActionType, double> action = actions_[0];
        ActionType action_type = std::get<0>(action);
        switch (action_type)
        {
        case MOVE_JAW:
          current_action_str = "move_jaw";
          send_gripper_pose_msg(std::get<1>(action));
          break;
        case WAIT:
          current_action_str = "wait " + std::to_string(std::get<1>(action)) + "s";
          break;
        case FOLLOW_TRAJ:
          if (planner_manager.trajectory_ready())
          {
            current_action_str = "traj ready id " + std::to_string(std::get<1>(action));
            Eigen::VectorXd q_plan = planner_manager.get_configuration_at_t(time);
            q_traj_ = q_plan;
          }
          else
          {
            current_action_str = "traj not ready id " + std::to_string(std::get<1>(action));
            time = .0;
          }
          break;
        }
        time += .01;
        RCLCPP_DEBUG(this->get_logger(), "Current action %s", current_action_str.c_str());
        if (action_is_finished)
        {
          time = 0.;
          integrated_q_err_ = Eigen::VectorXd::Zero(nq_);
          publish_random_goal_pose();
          actions_.erase(actions_.begin());
        }
      }
      // if configuration trajectory point hasn't been initialized, use current configuration.
      if (q_traj_ == Eigen::VectorXd::Zero(nq_))
        q_traj_ = current_q;
    }

    Eigen::VectorXd compute_q_ref()
    {
      if (actions_.size() == 0)
        return q_traj_;

      // Get current state
      std::tuple<ActionType, double> action = actions_[0];
      ActionType action_type = std::get<0>(action);

      // Add an integrator on every axis lacking precision if we follow a trajectory.
      if (action_type == FOLLOW_TRAJ)
      {
        Eigen::VectorXd q_err = q_traj_ - current_q;
        for (int idx = 0; idx < nq_; idx++)
        {
          if (std::abs(q_err[idx]) < parameters_.des_precision / static_cast<double>(nq_))
          {
            integrated_q_err_[idx] = 0.0;
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

    void
    publish_random_goal_pose()
    {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.frame_id = "odom";
      pose_msg.pose.position.x = 0.5;
      pose_msg.pose.position.y = 0.3;
      pose_msg.pose.position.z = 0.0;
      pose_msg.pose.orientation.x = 0.0;
      pose_msg.pose.orientation.y = 0.0;
      pose_msg.pose.orientation.z = 0.0;
      pose_msg.pose.orientation.w = 1.0;
      pose_publisher_->publish(pose_msg);
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

    double time = 0.;
    int nq_ = 5;
    Eigen::VectorXd current_q, q_traj_ = Eigen::VectorXd::Zero(nq_), integrated_q_err_ = Eigen::VectorXd::Zero(nq_);

    // ROS params.
    std::shared_ptr<joint_trajectory_publisher::ParamListener>
        parameter_listener_;
    joint_trajectory_publisher::Params parameters_;

    std::vector<Eigen::VectorXd> q_waypoints;
    bool first_time_ = true, first_time_reach_q_init = true;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr so_101_gripper_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lekiwi_gripper_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    PlannerManager planner_manager;
    std::vector<std::tuple<ActionType, double>> actions_ = {};
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