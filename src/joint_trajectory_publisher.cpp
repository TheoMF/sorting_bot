#include <string>
#include <chrono>
#include <unistd.h>
#include <stdexcept>
#include <cstdlib>

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

namespace joint_trajectory_publisher
{
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
  using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
  using Float64MultiArray = std_msgs::msg::Float64MultiArray;
  using MultiArrayDimension = std_msgs::msg::MultiArrayDimension;
  using JointState = sensor_msgs::msg::JointState;
  using Odometry = nav_msgs::msg::Odometry;
  using String = std_msgs::msg::String;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  class JointTrajectoryPublisher : public rclcpp::Node
  {
  public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher")
    {
      if (!load_parameters())
      {
        RCLCPP_ERROR(this->get_logger(), "Couldn't load the parameters, stopping the node.");
        throw std::runtime_error("Failed to load parameters");
      }

      // Joint trajectory publisher
      rclcpp::QoS joint_trajectory_qos_profile(10);
      joint_trajectory_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      joint_trajectory_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      joint_trajectory_publisher_ = this->create_publisher<JointTrajectory>(
          params_.joint_trajectory_topic,
          joint_trajectory_qos_profile);

      // Action client for navigation
      navigation_action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
          this,
          "/navigate_through_poses");

      // Gripper position publisher
      if (params_.robot_name == "SO-101")
      {
        so_101_gripper_publisher_ = this->create_publisher<Float64MultiArray>(
            params_.gripper_command_topic, rclcpp::SystemDefaultsQoS());
        lekiwi_gripper_publisher_ = nullptr;
      }
      else if (params_.robot_name == "LeKiwi")
      {
        so_101_gripper_publisher_ = nullptr;
        lekiwi_gripper_publisher_ = this->create_publisher<JointTrajectory>(
            params_.gripper_command_topic,
            rclcpp::SystemDefaultsQoS());
      }

      // Joint states subscriber
      rclcpp::QoS joint_states_qos_profile(10);
      joint_states_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      joint_states_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      state_subscriber_ = this->create_subscription<JointState>(
          "/joint_states", joint_states_qos_profile,
          std::bind(&JointTrajectoryPublisher::joint_states_callback, this, std::placeholders::_1));

      // Odometry subscriber
      rclcpp::QoS odometry_qos_profile(10);
      odometry_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      odometry_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      odom_subscriber_ = this->create_subscription<Odometry>(
          "/odom", odometry_qos_profile,
          std::bind(&JointTrajectoryPublisher::odom_callback, this,
                    std::placeholders::_1));

      // Robot description subscriber
      rclcpp::QoS robot_description_qos_profile(10);
      robot_description_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      robot_description_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      robot_description_subscriber_ = this->create_subscription<String>(
          "/robot_description", robot_description_qos_profile,
          std::bind(&JointTrajectoryPublisher::robot_description_callback, this,
                    std::placeholders::_1));

      // Timer to publish joint trajectory
      joint_traj_pub_timer_ = this->create_wall_timer(
          std::chrono::milliseconds((int)(1000.0 / params_.rate)),
          std::bind(&JointTrajectoryPublisher::joint_traj_pub_callback, this));

      // Timer to update detections status.
      detections_update_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500),
          std::bind(&JointTrajectoryPublisher::detections_update_callback, this));

      // TF related attributes.
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

      // Fill detections status map.
      for (std::string &object_frame : params_.objects_frame)
        det_status_map_.insert({object_frame, DetectionStatus(params_.hand_camera_frame, object_frame)});
      for (std::string &box_frame : params_.box_frames)
        det_status_map_.insert({box_frame, DetectionStatus(params_.base_camera_frame, box_frame)});

      // Initialize non-ROS-related attributes.
      nq_ = params_.nq;
      q_dot_traj_ = Eigen::VectorXd::Zero(nq_);
      integrated_q_err_ = Eigen::VectorXd::Zero(nq_);
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

    void joint_states_callback(const JointState &msg)
    {
      std::vector<double> current_q_vec = {};
      for (std::string &joint_name : params_.joint_names)
      {
        auto joint_name_it = std::find(msg.name.begin(), msg.name.end(), joint_name);
        int joint_idx = static_cast<int>(joint_name_it - msg.name.begin());
        current_q_vec.push_back(msg.position[joint_idx]);
      }
      std::lock_guard<std::mutex> guard(current_q_mutex_);
      current_q_ = Eigen::Map<Eigen::VectorXd>(current_q_vec.data(), current_q_vec.size());
      joint_states_callback_ready_ = true;
    }

    void robot_description_callback(const String &msg)
    {
      planner_manager_.initialize(tf_buffer_, tf_broadcaster_, msg.data, params_);
      robot_description_ready_ = true;
    }

    void nav_goal_response_callback(
        const rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::SharedPtr &goal_handle)
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

    void nav_result_callback(const rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::WrappedResult &result)
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

    void odom_callback(const Odometry &msg)
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
      auto goal_msg = NavigateThroughPoses::Goal();
      for (const auto &base_pose : base_poses)
        goal_msg.poses.push_back(base_pose_to_pose_msg(base_pose));

      // Add callbacks to track navigation progress.
      auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
      send_goal_options.goal_response_callback =
          std::bind(&JointTrajectoryPublisher::nav_goal_response_callback, this, std::placeholders::_1);
      send_goal_options.result_callback =
          std::bind(&JointTrajectoryPublisher::nav_result_callback, this, std::placeholders::_1);
      navigation_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    Eigen::VectorXd pose_msg_to_base_pose(const Pose &msg)
    {
      pinocchio::SE3::Quaternion quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
      double yaw = quat.toRotationMatrix().eulerAngles(0, 1, 2)[2];
      std::vector<double> pose_vec = {msg.position.x, msg.position.y, yaw};
      return Eigen::Map<Eigen::VectorXd>(pose_vec.data(), pose_vec.size());
    }

    PoseStamped base_pose_to_pose_msg(const Eigen::VectorXd &base_pose)
    {
      PoseStamped pose_msg;
      pose_msg.header.stamp = this->get_clock()->now();
      pose_msg.header.frame_id = params_.world_frame;
      pose_msg.pose.position.x = base_pose[0];
      pose_msg.pose.position.y = base_pose[1];
      pose_msg.pose.position.z = 0.0;
      Eigen::AngleAxisd angle_axis_rot = Eigen::AngleAxisd(base_pose[2], Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond quat_rot = Eigen::Quaterniond(angle_axis_rot.toRotationMatrix());
      pose_msg.pose.orientation.x = quat_rot.x();
      pose_msg.pose.orientation.y = quat_rot.y();
      pose_msg.pose.orientation.z = quat_rot.z();
      pose_msg.pose.orientation.w = quat_rot.w();
      return pose_msg;
    }

    void send_gripper_pose_msg(const double &gripper_pose)
    {
      if (params_.robot_name == "SO-101")
      {
        Float64MultiArray gripper_command_msg;
        MultiArrayDimension dim_sol;
        dim_sol.label = "position";
        dim_sol.size = 1;
        dim_sol.stride = 1;
        gripper_command_msg.layout.dim.push_back(dim_sol);
        gripper_command_msg.data = {gripper_pose};
        so_101_gripper_publisher_->publish(gripper_command_msg);
      }
      else if (params_.robot_name == "LeKiwi")
      {
        JointTrajectory joint_trajectory_msg;
        joint_trajectory_msg.joint_names = {"gripper"};
        JointTrajectoryPoint curr_point;
        curr_point.positions = {gripper_pose};
        joint_trajectory_msg.points = {curr_point};
        lekiwi_gripper_publisher_->publish(joint_trajectory_msg);
      }
    }

    void send_joint_trajectory_msg(const Eigen::VectorXd &q_ref, const Eigen::VectorXd &q_dot_ref)
    {
      JointTrajectory joint_trajectory_msg;
      joint_trajectory_msg.joint_names = params_.joint_names;
      JointTrajectoryPoint joint_trajectory_point_msg;
      std::vector<double> q_ref_vec(q_ref.data(), q_ref.data() + q_ref.size());
      std::vector<double> q_dot_ref_vec(q_dot_ref.data(), q_dot_ref.data() + q_dot_ref.size());
      joint_trajectory_point_msg.positions = q_ref_vec;
      joint_trajectory_point_msg.velocities = q_dot_ref_vec;

      joint_trajectory_msg.points = {joint_trajectory_point_msg};
      joint_trajectory_publisher_->publish(joint_trajectory_msg);
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
      Action action = planner_manager_.get_current_action();
      StateMachine state = planner_manager_.get_state();
      RCLCPP_DEBUG(this->get_logger(), "state idx : %d action idx : %d", static_cast<int>(state), static_cast<int>(action.type));
      switch (action.type)
      {
      case MOVE_JAW:
      {
        // MoveJawAction *move_jaw_action = dynamic_cast<MoveJawAction *>(&action);
        // double gripper_angle = move_jaw_action->value();
        if (const double *value = std::get_if<double>(&action.value))
        {
          send_gripper_pose_msg(*value);
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Stored wrong variant value type %s for Move Jaw action.", action.get_value_type_name());
          throw std::runtime_error("Got wrong type for move jaw action value");
        }

        break;
      }
      case MOVE_BASE:
      {
        handle_nav_goal_publication();
        break;
      }
      case MOVE_ARM:
      {
        update_traj_references();
        break;
      }
      }

      // Add log if we started an action.
      if (action.type != last_action_)
      {
        last_action_ = action.type;
        RCLCPP_INFO(this->get_logger(), "start new action index %d", static_cast<int>(action.type));
      }

      // Update actions status.
      planner_manager_.update_actions_status(nav_result_);
      if (nav_result_ != rclcpp_action::ResultCode::UNKNOWN)
        nav_result_ = rclcpp_action::ResultCode::UNKNOWN;
    }

    void update_integrated_q_err(const Eigen::VectorXd &q_err)
    {
      Action action = planner_manager_.get_current_action();
      // If we are in a state that requires to follow a trajectory that is ready, integrate error.
      if (action.type == MOVE_ARM && traj_ready_)
        for (int joint_idx = 0; joint_idx < nq_; joint_idx++)
        {
          if (over_traj_total_duration_)
            integrated_q_err_[joint_idx] += params_.integration_coeffs_after_traj[joint_idx] / params_.rate * q_err[joint_idx];
          else
            integrated_q_err_[joint_idx] += params_.integration_coeffs_during_traj[joint_idx] / params_.rate * q_err[joint_idx];
        }
      else
      {
        // Needed to handle motors delays to reach desired configuration.
        integrated_q_err_ += last_q_ - current_q_;
        traj_ready_ = false;
      }
    }

    Eigen::VectorXd get_q_ref(const Eigen::VectorXd &q_err)
    {
      Eigen::VectorXd friction_compensation = Eigen::VectorXd::Zero(nq_);
      for (int joint_idx = 0; joint_idx < nq_; joint_idx++)
      {
        if (q_err[joint_idx] > 0)
          friction_compensation[joint_idx] = params_.friction_compensation[joint_idx];
        else if (q_err[joint_idx] < 0)
          friction_compensation[joint_idx] = -params_.friction_compensation[joint_idx];
      }
      return q_traj_ + friction_compensation + integrated_q_err_;
    }

    void joint_traj_pub_callback()
    {
      // Wait for joint states callback and robot description.
      if (joint_states_callback_ready_ == false || robot_description_ready_ == false)
        return;

      // Initialize attributes while waiting for first trajectory to be set.
      std::lock_guard<std::mutex> guard(current_q_mutex_);
      if (first_joint_traj_pub_callback_iter_)
      {
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

    void detections_update_callback()
    {
      for (auto &[frame, det_status] : det_status_map_)
      {
        try
        {
          TransformStamped stamped_transform = tf_buffer_->lookupTransform(
              det_status.camera_frame, det_status.frame, tf2::TimePointZero);
          if (!det_status.last_stamp.has_value())
          {
            det_status.is_in_fov = true;
            det_status.last_stamp = stamped_transform.header.stamp;
            TransformStamped in_base_M_frame_stamped = tf_buffer_->lookupTransform(
                params_.base_frame, det_status.frame, tf2::TimePointZero);
            det_status.in_base_M_frame = transform_msg_to_SE3(in_base_M_frame_stamped.transform);
          }
          else
          {
            if (stamped_transform.header.stamp != det_status.last_stamp.value())
            {
              det_status.is_in_fov = true;
              det_status.last_stamp = stamped_transform.header.stamp;
              TransformStamped in_base_M_frame_stamped = tf_buffer_->lookupTransform(
                  params_.base_frame, det_status.frame, tf2::TimePointZero);
              det_status.in_base_M_frame = transform_msg_to_SE3(in_base_M_frame_stamped.transform);
            }
            else
            {
              det_status.is_in_fov = false;
            }
          }
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_DEBUG(this->get_logger(), "Didn't found tranform for frame %s , error : %s", frame.c_str(), ex.what());
        }
      }
      planner_manager_.update_det_status_map(det_status_map_);
    }

    // ROS params.
    std::shared_ptr<joint_trajectory_publisher::ParamListener>
        parameter_listener_;
    joint_trajectory_publisher::Params params_;

    // ROS publishers, subscribers and timers.
    rclcpp::TimerBase::SharedPtr joint_traj_pub_timer_, detections_update_timer_;
    rclcpp::Publisher<JointTrajectory>::SharedPtr joint_trajectory_publisher_;
    rclcpp::Publisher<Float64MultiArray>::SharedPtr so_101_gripper_publisher_;
    rclcpp::Publisher<JointTrajectory>::SharedPtr lekiwi_gripper_publisher_;
    rclcpp::Subscription<JointState>::SharedPtr state_subscriber_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<String>::SharedPtr robot_description_subscriber_;

    // Navigation attributes.
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr navigation_action_client_;
    rclcpp_action::ResultCode nav_result_ = rclcpp_action::ResultCode::UNKNOWN;

    // tf attributes.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Non-ROS-Related attributes.
    std::map<std::string, DetectionStatus> det_status_map_;
    PlannerManager planner_manager_;
    ActionType last_action_ = NONE;
    std::mutex current_q_mutex_;
    int nq_;
    Eigen::VectorXd current_q_, q_traj_, q_dot_traj_,
        integrated_q_err_, last_q_, base_pose_ = Eigen::VectorXd::Zero(3);
    std::vector<Eigen::VectorXd> last_sent_base_waypoints = {Eigen::Vector3d(-10., -10., 0.)};
    bool over_traj_total_duration_ = false, first_joint_traj_pub_callback_iter_ = true, traj_ready_ = false;
    std::atomic<bool> joint_states_callback_ready_ = false, robot_description_ready_ = false;
  };
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<joint_trajectory_publisher::JointTrajectoryPublisher>();
  try
  {
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(node->get_logger(), "Exception during node runtime : %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}