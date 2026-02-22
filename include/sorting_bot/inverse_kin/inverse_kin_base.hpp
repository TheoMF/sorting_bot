#ifndef SORTING_BOT_INVERSE_KIN_BASE_
#define SORTING_BOT_INVERSE_KIN_BASE_

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "sorting_bot/joint_trajectory_publisher_parameters.hpp"

namespace sorting_bot {

class InverseKinBase {
public:
  using InverseKinBaseParams = joint_trajectory_publisher::Params::PlannerManager::MotionPlanner::InverseKinBase;

  void initialize(const std::shared_ptr<pinocchio::Model> &model, const std::shared_ptr<pinocchio::Data> &data,
                  const int &ee_frame_id, InverseKinBaseParams &params);

  void initialize(const std::shared_ptr<pinocchio::Model> &model, const std::shared_ptr<pinocchio::Data> &data,
                  const int &ee_frame_id, const Eigen::Matrix<double, 6, 6> &error_weights,
                  const double &convergence_threshold);

  void set_q_in_joint_limits(Eigen::VectorXd &q) const;

protected:
  // Pinocchio model attributes
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  int gripper_frame_id_;
  int nq_;

  Eigen::Matrix<double, 6, 6> error_weights_; // square matrix with weights in the diagonal
  double convergence_threshold_, limit_margin = 0.01;
};

} // namespace sorting_bot

#endif