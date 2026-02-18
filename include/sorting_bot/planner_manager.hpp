#include <atomic>
#include <mutex>
#include <variant>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include "sorting_bot/motion_planner.hpp"

enum StateMachine {
  SEARCHING_OBJECTS,
  GOING_ABOVE_OBJECT_POSE, // Useful state to improve object detection precision.
  GRASPING,
  SEARCHING_BOX,
  PLACING
};

enum ActionType { NONE, WAIT, MOVE_GRIPPER, MOVE_BASE, MOVE_ARM, SEARCH_OBJECT, SEARCH_BOX };

struct Action {
  ActionType type;
  int id;
  std::variant<std::monostate, double, Eigen::VectorXd> value;

  const char *get_value_type_name() const {
    return std::visit([](auto &v) -> std::type_index { return typeid(v); }, value).name();
  }
};

std::string get_state_as_string(const StateMachine &state);

std::string get_action_type_as_string(const ActionType &action_type);

struct Detection {
  std::string camera_frame, frame;
  bool is_in_fov;
  std::optional<builtin_interfaces::msg::Time> last_stamp;
  std::optional<pinocchio::SE3> in_base_M_frame;

  Detection() {
    camera_frame = "";
    frame = "";
    is_in_fov = false;
    last_stamp = std::nullopt;
    in_base_M_frame = std::nullopt;
  }

  Detection(const std::string &_camera_frame, const std::string &_frame) : camera_frame(_camera_frame), frame(_frame) {
    is_in_fov = false;
    last_stamp = std::nullopt;
    in_base_M_frame = std::nullopt;
  }
};

pinocchio::SE3 transform_msg_to_SE3(const geometry_msgs::msg::Transform &transform);

class PlannerManager {
public:
  PlannerManager() {}

  void initialize(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
                  const std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_broadcaster, const std::string &urdf,
                  joint_trajectory_publisher::Params &params);

  void initialize_in_box_M_compartment_map(const double &box_width, const double &box_length,
                                           const double &object_height_while_placing_in_box);

  void publish_transform(const pinocchio::SE3 &transform, const std::string parent_frame,
                         std::string child_frame) const;

  pinocchio::SE3 get_most_recent_in_parent_M_child(const std::string &parent_frame,
                                                   const std::string &child_frame) const;

  void update_detection_map(const std::map<std::string, Detection> &det_status_map);

  std::optional<Detection> get_first_detection_currently_in_fov(const std::vector<std::string> &frames);

  void do_search_object_action();

  void do_search_box_action();

  bool base_waypoints_published() const;

  std::vector<Eigen::VectorXd> get_base_waypoints() const;

  void update_nav_action(const rclcpp_action::ResultCode &nav_result);

  void set_base_waypoints_published(const bool &base_waypoints_published);

  bool action_is_finished(const Action &action, const rclcpp_action::ResultCode &nav_result) const;

  void do_action_first_time_in_steps();

  void reset_actions_attributes();

  void update_actions_status(const rclcpp_action::ResultCode &nav_result);

  std::vector<Action> get_state_actions() const;

  void restart_state_actions();

  void update_state(const Eigen::VectorXd &current_q, const Eigen::VectorXd &base_pose,
                    const rclcpp_action::ResultCode &nav_result, const rclcpp::Time ros_time);

  Action get_current_action() const;

  void compute_trajectory();

  void set_object_search_to_next_location();

  std::vector<Eigen::VectorXd> get_going_above_object_pose_q_waypoints() const;

  std::vector<Eigen::VectorXd> get_grasping_q_waypoints() const;

  std::vector<Eigen::VectorXd> get_placing_q_waypoints() const;

  std::tuple<Eigen::VectorXd, Eigen::VectorXd, bool> get_traj_value_at_t() const;

  pinocchio::SE3 get_in_base_M_grasp(const pinocchio::SE3 &in_base_M_object, const std::string &object_frame) const;

  bool goal_pose_achieved() const;

  bool trajectory_ready() const;

  StateMachine get_state() const;

  bool last_nav_succeed = false;

private:
  // ROS related attributes.
  rclcpp::Time ros_time_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Logger logger_ = rclcpp::get_logger("joint_trajectory_publisher");

  // Perception related attributes.
  std::string world_frame_, base_frame_, ee_frame_name_;
  std::vector<std::string> objects_frame_, box_frames_;
  Detection object_detection_to_sort_;
  std::mutex detection_map_mutex_;
  std::map<std::string, Detection> detection_map_;
  pinocchio::SE3 in_world_M_box_, in_base_M_box_;

  // Navigation Related attributes.
  Eigen::VectorXd base_pose_ = Eigen::VectorXd::Zero(3);
  std::vector<Eigen::VectorXd> base_waypoint_vec_;
  int base_waypoint_vec_idx_ = 0;
  bool base_waypoints_published_ = false;
  double box_dist_while_placing_, search_box_y_dist_threshold_;

  // Motion planning related attributes.
  MotionPlanner motion_planner;
  std::atomic<bool> trajectory_ready_ = false;
  std::thread trajectory_computing_thread_;
  Eigen::VectorXd current_q_, q_base_moving_;
  std::vector<double> joints_des_precision_;
  std::vector<Eigen::VectorXd> q_waypoints, qs_searching_objects_;
  int qs_searching_object_idx_ = 0, nq_;
  double closed_gripper_position_, open_gripper_position_;
  Eigen::Vector3d in_grasp_translation_grasp_to_pre_grasp_, in_base_translation_gripper_to_pre_placing_;
  pinocchio::SE3::Quaternion in_world_rot_front_grasp_quat_, in_world_rot_top_grasp_quat_;
  std::map<std::string, pinocchio::SE3> in_box_M_compartment_map_;
  std::map<std::string, Eigen::Vector3d> in_grasp_translation_object_to_grasp_map_,
      in_box_frame_translation_box_frame_to_box_center_map_;
  std::map<std::string, bool> objects_to_grasp_from_top_map_;

  // Finite-state machine related attributes.
  enum StateMachine state_ = SEARCHING_OBJECTS;
  std::vector<StateMachine> state_order_ = {SEARCHING_OBJECTS, GOING_ABOVE_OBJECT_POSE, GRASPING, SEARCHING_BOX,
                                            PLACING};
  std::vector<Action> actions_ = {};
  bool current_state_action_were_retrieved_ = false, start_new_action_ = true;
  double wait_duration_before_vision_action_, wait_duration_after_gripper_moved_, action_time_ = 0.,
                                                                                  state_update_period_;
  std::string robot_name_;
};
