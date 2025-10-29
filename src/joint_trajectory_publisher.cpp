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

      rclcpp::QoS joint_trajectory_qos_profile(10);
      joint_trajectory_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      joint_trajectory_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/so_100_arm_controller/joint_trajectory", joint_trajectory_qos_profile);
      gripper_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/so_100_arm_gripper_controller/commands", rclcpp::SystemDefaultsQoS());
      rclcpp::QoS joint_states_qos_profile(10);
      joint_states_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      joint_states_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", joint_states_qos_profile, std::bind(&JointTrajectoryPublisher::topic_callback, this, std::placeholders::_1));
      robot_description_subscriber_ = this->create_subscription<std_msgs::msg::String>(
          "/robot_description", joint_states_qos_profile, std::bind(&JointTrajectoryPublisher::robot_description_callback, this, std::placeholders::_1));
      timer_ = this->create_wall_timer(
          10ms, std::bind(&JointTrajectoryPublisher::timer_callback, this));
      start_time = this->get_clock()->now();
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      const std::string urdf_filename = std::string("/home/gepetto/ros2_ws/src/repos/SO-100-arm/urdf/so101_new_calib.urdf");
      RCLCPP_INFO(this->get_logger(), "finished setup");
    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState &msg)
    {
      std::vector<double> q_vec = msg.position;
      q_vec.pop_back();
      current_q = Eigen::Map<Eigen::VectorXd>(q_vec.data(), q_vec.size());
      ready = true;
    }

    void robot_description_callback(const std_msgs::msg::String &msg)
    {
      planner_manager.initialize(tf_buffer_, msg.data, end_effector_name_, parameters_); //
      planner_ready_ = true;
    }

    void send_gripper_pose_msg(const double &angle)
    {
      std_msgs::msg::Float64MultiArray gripper_command_msg;
      std_msgs::msg::MultiArrayDimension dim_sol;
      dim_sol.label = "position";
      dim_sol.size = 1;
      dim_sol.stride = 1;
      gripper_command_msg.layout.dim.push_back(dim_sol);
      std_msgs::msg::MultiArrayDimension max_effort_dim;
      gripper_command_msg.data = {angle};
      gripper_publisher_->publish(gripper_command_msg);
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

    std::vector<double> get_trajectory_joint_value()
    {
      std::vector<double> q_traj;
      // Update state
      bool action_is_finished = false;
      if (actions_.size() > 0)
        action_is_finished = planner_manager.action_is_finished(current_q, time, actions_[0]);
      std::vector<std::tuple<ActionType, double>> action = planner_manager.update_state(current_q);
      actions_.insert(actions_.end(), action.begin(), action.end());

      // GET TRANSFORM
      // for (int joint_idx = 0; joint_idx < 5; joint_idx++)
      //  RCLCPP_INFO(this->get_logger(), "q%d %.6f", joint_idx, q[joint_idx]);

      std::string current_action_str;
      std::vector<double> current_q_vec(current_q.data(), current_q.data() + current_q.size());
      if (actions_.size() == 0)
      {
        q_traj = current_q_vec;
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

          q_traj = current_q_vec;
          break;
        case WAIT:
          current_action_str = "wait " + std::to_string(std::get<1>(action)) + "s";
          q_traj = current_q_vec;
          break;
        case FOLLOW_TRAJ:
          if (planner_manager.trajectory_ready())
          {
            current_action_str = "traj ready id " + std::to_string(std::get<1>(action));
            Eigen::VectorXd q_plan = planner_manager.get_configuration_at_t(time);
            std::vector<double> q_plan_vec(q_plan.data(), q_plan.data() + q_plan.size());
            q_traj = q_plan_vec;
          }
          else
          {
            current_action_str = "traj not ready id " + std::to_string(std::get<1>(action));
            q_traj = current_q_vec;
            time = .0;
          }
          break;
        default:
          q_traj = current_q_vec;
        }
        time += .01;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 80, "Current action %s", current_action_str.c_str());
        if (action_is_finished)
        {
          time = 0.;
          actions_.erase(actions_.begin());
        }
      }
      return q_traj;
    }

    void
    timer_callback()
    {
      // wait for robot configuration and robot description
      if (ready == false || planner_ready_ == false)
        return;
      std::vector<double> q_traj = get_trajectory_joint_value();
      trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
      joint_trajectory_msg.joint_names = parameters_.joint_names;
      trajectory_msgs::msg::JointTrajectoryPoint curr_point;
      curr_point.positions = q_traj;
      curr_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0};

      joint_trajectory_msg.points = {curr_point};
      publisher_->publish(joint_trajectory_msg);
    }

    enum StateMachine state_ = GOING_TO_QINIT;
    double time = 0.;
    Eigen::VectorXd current_q;

    // ROS params.
    std::shared_ptr<joint_trajectory_publisher::ParamListener>
        parameter_listener_;
    joint_trajectory_publisher::Params parameters_;

    std::vector<Eigen::VectorXd> q_waypoints;
    bool first_time_ = true, first_time_reach_q_init = true;
    rclcpp::Time start_time;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_publisher_;
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