#include "sorting_bot/inverse_kin_base.hpp"

#include "sorting_bot/joint_trajectory_publisher_parameters.hpp"

class InverseKin : public InverseKinBase {

public:
  InverseKin() {
    // Inverse kinematic parameters default values.
    error_weights_ = Eigen::Matrix<double, 6, 6>::Identity();
    convergence_threshold_ = 1e-3;
    max_iter_ = 20000;
    dt_ = 1e-1;
    damp_ = 1e-6;
  }

  void initialize(joint_trajectory_publisher::Params::InverseKin params) {
    error_weights_ = Eigen::Matrix<double, 6, 6>::Identity();
    for (int idx = 0; idx < 6; idx++)
      error_weights_(idx, idx) = params.error_weights[idx];
    convergence_threshold_ = params.convergence_threshold;
    max_iter_ = params.max_iter;
    dt_ = params.dt;
    damp_ = params.damp;
  }

  std::tuple<Eigen::VectorXd, double> get_inverse_kinematics(Eigen::VectorXd q_init,
                                                             const pinocchio::SE3 &in_world_M_des_pose) const {

    // Initialize IK variables;
    Eigen::VectorXd q = q_init;
    pinocchio::Data::Matrix6x J(6, model_->nv);
    J.setZero();
    Eigen::VectorXd v(model_->nv), err;

    for (int i = 0; i < max_iter_; i++) {
      // Compute current error.
      pinocchio::framesForwardKinematics(*model_, *data_, q);
      const pinocchio::SE3 iMd = data_->oMf[ee_frame_id_].actInv(in_world_M_des_pose);
      err = error_weights_ * pinocchio::log6(iMd).toVector();
      if (err.norm() < convergence_threshold_)
        break;

      // Compute inverse kinematics using the jacobian matrix.
      pinocchio::computeFrameJacobian(*model_, *data_, q, ee_frame_id_, pinocchio::ReferenceFrame::LOCAL, J);
      pinocchio::Data::Matrix6 Jlog;
      pinocchio::Jlog6(iMd.inverse(), Jlog);
      J = -Jlog * J;
      pinocchio::Data::Matrix6 JJt;
      JJt.noalias() = J * J.transpose();
      JJt.diagonal().array() += damp_;
      v.noalias() = -J.transpose() * JJt.ldlt().solve(err);

      // Compute new configuration.
      q = pinocchio::integrate(*model_, q, v * dt_);
      set_q_in_joint_limits(q);
    }

    return std::make_tuple(q, err.norm());
  }

private:
  // Inverse kinematics parameters
  Eigen::Matrix<double, 6, 6> error_weights_; // square matrix with weights in the diagonal
  double convergence_threshold_, dt_, damp_, min_precision_threshold_;
  int max_iter_;
};