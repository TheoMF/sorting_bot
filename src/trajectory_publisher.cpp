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

#include "sorting_bot/motion_planner.hpp"

using namespace std::chrono_literals;

enum StateMachine
{
  GOING_TO_QINIT,
  GOING_TO_GRASP_POSE,
  PLACING
};

class TrajectoryPublisher : public rclcpp::Node
{
public:
  TrajectoryPublisher()
      : Node("trajectory_publisher")
  {
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/so_100_arm_controller/joint_trajectory", 10);
    gripper_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/so_100_arm_gripper_controller/commands", 10);
    state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/command", 10, std::bind(&TrajectoryPublisher::topic_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        10ms, std::bind(&TrajectoryPublisher::timer_callback, this));
    start_time = this->get_clock()->now();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const std::string urdf_filename = std::string("/home/tmartinez/ros2_ws/src/SO-100-arm/urdf/so101_new_calib.urdf");
    std::string end_effector_name = "gripper_frame_link";
    motion_planner.initialize(urdf_filename, end_effector_name);
    RCLCPP_INFO(this->get_logger(), "finished setup");
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState &msg)
  {
    std::vector<double> q_vec = msg.position;
    q_vec.pop_back();
    q = Eigen::Map<Eigen::VectorXd>(q_vec.data(), q_vec.size());
    ready = true;
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

  pinocchio::SE3 transform_msg_to_SE3(const geometry_msgs::msg::Transform &transform)
  {
    Eigen::Quaterniond q(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
    Eigen::Vector3d t(transform.translation.x, transform.translation.y, transform.translation.z);
    pinocchio::SE3 se3(q, t);
    return se3;
  }

  pinocchio::SE3 get_in_base_M_object(const std::string &parent_frame, const std::string &child_frame)
  {
    std::string cameraFrameRel = "camera";
    geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(cameraFrameRel, child_frame, tf2::TimePointZero);
    pinocchio::SE3 in_camera_M_cardboard = transform_msg_to_SE3(t.transform);
    auto current_stamp = this->get_clock()->now();
    int nanosec_delay = evaluate_vision_delay(child_frame);
    if (nanosec_delay == -1)
    {
      RCLCPP_WARN(this->get_logger(), "object %s transform is old, transform stamp seconds %d, current stamp seconds %.6f ", child_frame.c_str(), t.header.stamp.sec, current_stamp.seconds());
      throw tf2::TransformException("object transform is too old");
    }
    auto camera_transform_stamp = t.header.stamp;
    camera_transform_stamp.nanosec += nanosec_delay;
    geometry_msgs::msg::TransformStamped other_t = tf_buffer_->lookupTransform(parent_frame, cameraFrameRel, camera_transform_stamp);
    pinocchio::SE3 in_base_M_camera = transform_msg_to_SE3(other_t.transform);
    return in_base_M_camera * in_camera_M_cardboard;
  }

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
        publish_transform(des_transform, "base_link", "desired_pose");
        current_target_ = make_tuple(object_frame, des_transform);
        objects_done_.push_back(object_frame);
        RCLCPP_INFO(this->get_logger(), "found object %s", object_frame.c_str());
        break;
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_INFO(this->get_logger(), "didn't found transform for object  %s error : %s", object_frame.c_str(), ex.what());
      }
    }
    return found_transform;
  }

  Eigen::VectorXd get_inverse_kinematic_at_pose(const pinocchio::SE3 &des_transform)
  {
    Eigen::VectorXd q_inv_kin;
    std::tuple<Eigen::VectorXd, double> inv_kin_res = motion_planner.get_inverse_kinematics_for_des_pose(q, des_transform);
    double pose_err = std::get<1>(inv_kin_res);
    if (pose_err > 0.01)
    {
      RCLCPP_WARN(this->get_logger(), "IK didn't converge, Run genetic algorithm");
      Individual best = motion_planner.run_gen_algo(des_transform);
      q_inv_kin = best.q();
    }
    else
    {
      q_inv_kin = std::get<0>(inv_kin_res);
    }
    std::string ee_frame_name = "gripper_frame_link";
    pinocchio::SE3 pose_inv_kin = motion_planner.get_frame_pose_at_q(q_inv_kin, ee_frame_name);
    Eigen::Matrix<double, 6, 1> err = pinocchio::log6(pose_inv_kin.actInv(des_transform)).toVector();
    Eigen::Matrix<double, 5, 1> err_5d;
    err_5d.head<3>() = err.head<3>();
    err_5d.tail<2>() = err.tail<2>();
    RCLCPP_INFO(this->get_logger(), "inverse kinematic pose error norm %.6f base IK err %.6f", err_5d.norm(), pose_err);
    return q_inv_kin;
  }

  void compute_trajectory()
  {
    if (state_ == GOING_TO_GRASP_POSE)
    {
      RCLCPP_INFO(this->get_logger(), "computing inverse kinematics for grasping");
      pinocchio::SE3 des_transform = std::get<1>(current_target_);
      Eigen::VectorXd q_goal = get_inverse_kinematic_at_pose(des_transform);
      pinocchio::SE3 above_object_des_transform = des_transform;
      above_object_des_transform.translation().z() += 0.06;
      q_waypoint_above_object = get_inverse_kinematic_at_pose(above_object_des_transform);
      q_waypoints = {q_waypoint_above_object, q_goal};
      Eigen::Vector3d des_trans = des_transform.translation();
      RCLCPP_INFO(this->get_logger(), "object pose x %.6f y %.6f z %.6f", des_trans.x(), des_trans.y(), des_trans.z());
    }
    else if (state_ == PLACING)
    {
      state_ = PLACING;
      std::string object = std::get<0>(current_target_);
      auto it = find(objects_frame_.begin(), objects_frame_.end(), object);
      int object_idx = it - objects_frame_.begin();
      std::vector<double> q_placing_vec = q_types[object_idx];
      Eigen::VectorXd q_goal = Eigen::Map<Eigen::VectorXd>(q_placing_vec.data(), q_placing_vec.size());
      q_waypoints = {q_waypoint_above_object, q_waypoint_up_, q_goal};
    }
    else if (state_ == GOING_TO_QINIT)
    {
      RCLCPP_INFO(this->get_logger(), "SETTING QINIT traj");
      q_waypoints = {q_init_};
    }
    double traj_duration = 3.0 * q_waypoints.size();
    motion_planner.set_plan(q, q_waypoints, traj_duration);
    time = .0;
    trajectory_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "finished setting traj");
  }
  void
  timer_callback()
  {
    // wait for robot configuration
    if (ready == false)
      return;

    // Update state
    if (state_ == GOING_TO_QINIT)
    {

      if (first_time_)
      {
        send_gripper_pose_msg(1.5);
        trajectory_computing_thread_ = std::thread(&TrajectoryPublisher::compute_trajectory, this);
      }
      first_time_ = false;
      if (trajectory_ready_)
      {
        if (goal_pose_achieved(q, q_waypoints.back(), 5e-3))
        {
          if (first_time_reach_q_init)
          {

            first_time_reach_q_init = false;
          }
          usleep(2000000);
          bool object_found = find_next_object_to_grasp_transform();

          if (object_found)
          {
            state_ = GOING_TO_GRASP_POSE;
            trajectory_ready_ = false;
            trajectory_computing_thread_.join();
            trajectory_computing_thread_ = std::thread(&TrajectoryPublisher::compute_trajectory, this);
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
          RCLCPP_INFO(this->get_logger(), "SETTING PLACING state");
          state_ = PLACING;
          trajectory_ready_ = false;
          trajectory_computing_thread_.join();
          trajectory_computing_thread_ = std::thread(&TrajectoryPublisher::compute_trajectory, this);
          usleep(700000);
          send_gripper_pose_msg(0.2);
          usleep(300000);
        }
      }
    }
    else if (state_ == PLACING)
    {
      if (trajectory_ready_)
      {
        if (goal_pose_achieved(q, q_waypoints.back(), 5e-3))
        {
          RCLCPP_INFO(this->get_logger(), "SETTING GOING_TO_QINIT state");
          state_ = GOING_TO_QINIT;
          first_time_reach_q_init = true;
          trajectory_ready_ = false;
          trajectory_computing_thread_.join();
          trajectory_computing_thread_ = std::thread(&TrajectoryPublisher::compute_trajectory, this);
          usleep(600000);
          send_gripper_pose_msg(1.5);
        }
      }
    }

    // GET TRANSFORM
    // for (int joint_idx = 0; joint_idx < 5; joint_idx++)
    //  RCLCPP_INFO(this->get_logger(), "q%d %.6f", joint_idx, q[joint_idx]);

    // set then send joint control msg
    trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
    joint_trajectory_msg.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint curr_point;
    if (trajectory_ready_)
    {
      Eigen::VectorXd q_plan = motion_planner.get_configuration_at_t(time);
      std::vector<double> q_plan_vec(q_plan.data(), q_plan.data() + q_plan.size());
      curr_point.positions = q_plan_vec;
    }
    else
    {
      std::vector<double> current_q(q.data(), q.data() + q.size());
      curr_point.positions = current_q;
    }
    curr_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0};
    joint_trajectory_msg.points = {curr_point};
    publisher_->publish(joint_trajectory_msg);
    time += .01;
  }
  enum StateMachine state_ = GOING_TO_QINIT;
  double time = 0., waiting_time = 0.;
  Eigen::VectorXd q;
  std::vector<Eigen::VectorXd> q_waypoints;
  MotionPlanner motion_planner;
  bool first_time_ = true, first_time_reach_q_init = true;
  std::vector<std::string> objects_frame_{"glass", "cardboard", "metal", "plastic"};
  std::vector<std::string> objects_done_{};
  std::tuple<std::string, pinocchio::SE3> current_target_;
  rclcpp::Time start_time;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Eigen::VectorXd q_waypoint_above_object;
  std::vector<double> q_waypoint_up_vec = {0.256175, -0.573709, -0.335942, 1.032369, 1.679709};
  Eigen::VectorXd q_waypoint_up_ = Eigen::Map<Eigen::VectorXd>(q_waypoint_up_vec.data(), q_waypoint_up_vec.size());
  std::vector<double> q_init_vec = {0.322136, 0.018408, -0.678020, 1.794758, 1.684311};
  Eigen::VectorXd q_init_ = Eigen::Map<Eigen::VectorXd>(q_init_vec.data(), q_init_vec.size());
  std::vector<std::vector<double>> q_types{
      {-0.569107, -0.408039, -0.110447, 1.207243, 1.684311},
      {-0.929592, -0.078233, -0.033748, 0.362019, 1.684311},
      {-1.222583, -0.891243, 0.041417, 1.764078, 1.679709},
      {-1.497165, -0.055223, -0.504680, 1.204175, 1.670505},

  };

  std::vector<std::string> joint_names_ = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"};
  std::thread trajectory_computing_thread_;
  bool ready = false, trajectory_ready_ = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}