#include "sorting_bot/inverse_kin/inverse_kin_base.hpp"

namespace sorting_bot {

void InverseKinBase::initialize(const std::shared_ptr<pinocchio::Model> &model,
                                const std::shared_ptr<pinocchio::Data> &data, const int &ee_frame_id,
                                InverseKinBaseParams &params) {
  model_ = model;
  data_ = data;
  ee_frame_id_ = ee_frame_id;
  nq_ = model_->nq;
  convergence_threshold_ = params.convergence_threshold;
  error_weights_ = Eigen::Matrix<double, 6, 6>::Identity();
  for (int idx = 0; idx < 6; idx++)
    error_weights_(idx, idx) = params.error_weights[idx];
}

void InverseKinBase::initialize(const std::shared_ptr<pinocchio::Model> &model,
                                const std::shared_ptr<pinocchio::Data> &data, const int &ee_frame_id,
                                const Eigen::Matrix<double, 6, 6> &error_weights, const double &convergence_threshold) {
  model_ = model;
  data_ = data;
  ee_frame_id_ = ee_frame_id;
  nq_ = model_->nq;
  error_weights_ = error_weights;
  convergence_threshold_ = convergence_threshold;
}

void InverseKinBase::set_q_in_joint_limits(Eigen::VectorXd &q) const {
  for (int joint_idx = 0; joint_idx < model_->nq; joint_idx++) {
    if (q[joint_idx] < model_->lowerPositionLimit[joint_idx] + limit_margin)
      q[joint_idx] = model_->lowerPositionLimit[joint_idx];
    if (q[joint_idx] > model_->upperPositionLimit[joint_idx] - limit_margin)
      q[joint_idx] = model_->upperPositionLimit[joint_idx];
  }
}

} // namespace sorting_bot
