#include <fstream>
#include <iostream>

#include "pinocchio/parsers/urdf.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sorting_bot/inverse_kin/genetic_algo_inverse_kin.hpp"
#include "sorting_bot/inverse_kin/inverse_kin.hpp"
#include "sorting_bot/quintic_polynom.hpp"

class MotionPlanner {
public:
  MotionPlanner() {}
  void initialize(std::string urdf, joint_trajectory_publisher::Params params);

  std::optional<Eigen::VectorXd> get_inverse_kinematic_at_pose(const Eigen::VectorXd &q_init,
                                                               const pinocchio::SE3 &des_in_base_M_gripper) const;

  pinocchio::SE3 get_in_base_M_gripper_at_q(const Eigen::VectorXd &q, const std::string &frame_name) const;

  void set_motion_planning(const Eigen::VectorXd &q_start, const std::vector<Eigen::VectorXd> &q_waypoints);

  std::tuple<Eigen::VectorXd, Eigen::VectorXd, bool> get_traj_value_at_t(const double &time) const;

  double get_traj_duration() const;

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("joint_trajectory_publisher");
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  std::string ee_frame_name_;
  int ee_frame_id_, nq_;
  double min_precision_threshold_;
  bool use_genetic_algo_;
  QuinticPolynom quintic_polynom_;
  GeneticAlgoInverseKin genetic_algo_inverse_kin_;
  InverseKin inverse_kin_;
};