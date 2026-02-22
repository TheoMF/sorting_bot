
#include "sorting_bot/motion_planner.hpp"

namespace sorting_bot {

void MotionPlanner::initialize(const std::string &urdf, const std::string &robot_name, MotionPlannerParams &params) {
  // Build pinocchio reduced model.
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(urdf, model);
  std::vector<std::string> joints_to_lock_names;
  if (robot_name == "LeKiwi")
    joints_to_lock_names = {"gripper", "rear_wheel_drive", "left_wheel_drive", "right_wheel_drive"};
  else if (robot_name == "SO-101")
    joints_to_lock_names = {"gripper"};
  std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id = {};
  for (std::string &joint_to_lock_name : joints_to_lock_names)
    list_of_joints_to_lock_by_id.push_back(model.getJointId(joint_to_lock_name));
  Eigen::VectorXd q_rand = pinocchio::randomConfiguration(model);
  model_ =
      std::make_shared<pinocchio::Model>(pinocchio::buildReducedModel(model, list_of_joints_to_lock_by_id, q_rand));

  // Initialize remaining attributes.
  pinocchio::Data data(*model_);
  data_ = std::make_shared<pinocchio::Data>(data);
  gripper_frame_ = params.gripper_frame;
  nq_ = model_->nq;
  gripper_frame_id_ = model_->getFrameId(gripper_frame_);
  use_genetic_algo_ = params.use_genetic_algo;
  inverse_kin_.initialize(model_, data_, gripper_frame_id_, params.inverse_kin_base, params.inverse_kin);
  genetic_algo_inverse_kin_.initialize(model_, data_, gripper_frame_id_, params.inverse_kin_base,
                                       params.genetic_algo_inverse_kin);
  quintic_polynom_.set_motion_planning_time_coeff(params.motion_planning_time_coeff);
  ik_min_precision_threshold_ = params.ik_min_precision_threshold;
}

std::optional<Eigen::VectorXd>
MotionPlanner::get_inverse_kinematic_at_pose(const Eigen::VectorXd &q_init,
                                             const pinocchio::SE3 &des_in_base_M_gripper) const {
  // Run Inverse kinematics.
  Eigen::VectorXd q_inv_kin;
  std::tuple<Eigen::VectorXd, double> inv_kin_res = inverse_kin_.get_inverse_kinematics(q_init, des_in_base_M_gripper);
  double pose_norm_err = std::get<1>(inv_kin_res);

  // If it failed, we can as an option use a genetic algorithm to improve result.
  if (pose_norm_err > ik_min_precision_threshold_ && use_genetic_algo_) {
    RCLCPP_INFO(logger_, "IK wasn't precise enough, run genetic algorithm.");
    Individual best_individual = genetic_algo_inverse_kin_.run_gen_algo(des_in_base_M_gripper);
    q_inv_kin = best_individual.get_q();
  } else
    q_inv_kin = std::get<0>(inv_kin_res);

  // Return Inverse kinematics result if it's precise enough.
  if (pose_norm_err <= ik_min_precision_threshold_)
    return q_inv_kin;

  RCLCPP_WARN(logger_, "IK Failed.");
  return std::nullopt;
}

pinocchio::SE3 MotionPlanner::get_in_base_M_gripper_at_q(const Eigen::VectorXd &q) const {
  pinocchio::framesForwardKinematics(*model_, *data_, q);
  auto frame_id = model_->getFrameId(gripper_frame_);
  return data_->oMf[frame_id];
}

void MotionPlanner::set_motion_planning(const Eigen::VectorXd &q_start,
                                        const std::vector<Eigen::VectorXd> &q_waypoints) {
  quintic_polynom_.set_motion_planning(q_start, q_waypoints);
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, bool> MotionPlanner::get_traj_value_at_t(const double &time) const {
  return quintic_polynom_.get_traj_value_at_t(time);
}

double MotionPlanner::get_traj_duration() const { return quintic_polynom_.traj_duration(); }

} // namespace sorting_bot
