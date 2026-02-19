#include "sorting_bot/inverse_kin/inverse_kin_base.hpp"

void InverseKinBase::initialize_model(std::shared_ptr<pinocchio::Model> model, std::shared_ptr<pinocchio::Data> data,
                                      int ee_frame_id) {
  model_ = model;
  data_ = data;
  ee_frame_id_ = ee_frame_id;
  nq_ = model_->nq;
}

void InverseKinBase::set_q_in_joint_limits(Eigen::VectorXd &q) const {
  for (int joint_idx = 0; joint_idx < model_->nq; joint_idx++) {
    if (q[joint_idx] < model_->lowerPositionLimit[joint_idx] + limit_margin)
      q[joint_idx] = model_->lowerPositionLimit[joint_idx];
    if (q[joint_idx] > model_->upperPositionLimit[joint_idx] - limit_margin)
      q[joint_idx] = model_->upperPositionLimit[joint_idx];
  }
}