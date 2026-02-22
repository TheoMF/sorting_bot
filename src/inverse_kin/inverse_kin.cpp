#include "sorting_bot/inverse_kin/inverse_kin.hpp"

namespace sorting_bot {

InverseKin::InverseKin() {
  // Inverse kinematic parameters default values.
  max_iter_ = 1000;
  dt_ = 1e-1;
  damp_ = 1e-6;
}

void InverseKin::initialize(const std::shared_ptr<pinocchio::Model> &model,
                            const std::shared_ptr<pinocchio::Data> &data, const int &ee_frame_id,
                            InverseKinBaseParams &base_params, InverseKinParams &params) {
  InverseKinBase::initialize(model, data, ee_frame_id, base_params);
  max_iter_ = params.max_iter;
  dt_ = params.dt;
  damp_ = params.damp;
}

std::tuple<Eigen::VectorXd, double>
InverseKin::get_inverse_kinematics(Eigen::VectorXd q_init, const pinocchio::SE3 &in_world_M_des_pose) const {
  // Initialize IK variables;
  Eigen::VectorXd q = q_init, speed_dir(model_->nv), error;
  pinocchio::Data::Matrix6x frame_jacobian(6, model_->nv), task_jacobian(6, model_->nv);
  pinocchio::Data::Matrix6 task_jacobian_log, gramian_matrix;
  frame_jacobian.setZero();
  task_jacobian.setZero();

  for (int iter = 0; iter < max_iter_; iter++) {
    // Compute current error.
    pinocchio::framesForwardKinematics(*model_, *data_, q);
    const pinocchio::SE3 in_current_gripper_M_des_gripper = data_->oMf[ee_frame_id_].actInv(in_world_M_des_pose);
    error = error_weights_ * pinocchio::log6(in_current_gripper_M_des_gripper).toVector();
    if (error.norm() < convergence_threshold_)
      break;

    // Compute inverse kinematics using the jacobian matrix.
    pinocchio::computeFrameJacobian(*model_, *data_, q, ee_frame_id_, pinocchio::ReferenceFrame::LOCAL, frame_jacobian);
    pinocchio::Jlog6(in_current_gripper_M_des_gripper.inverse(), task_jacobian_log);
    task_jacobian = -task_jacobian_log * frame_jacobian;
    gramian_matrix.noalias() = task_jacobian * task_jacobian.transpose();
    gramian_matrix.diagonal().array() += damp_;
    speed_dir.noalias() = -task_jacobian.transpose() * gramian_matrix.ldlt().solve(error);

    // Compute new configuration.
    q = pinocchio::integrate(*model_, q, speed_dir * dt_);
    set_q_in_joint_limits(q);
  }

  return std::make_tuple(q, error.norm());
}

} // namespace sorting_bot
