#include "sorting_bot/inverse_kin_base.hpp"

#include "sorting_bot/joint_trajectory_publisher_parameters.hpp"

class InverseKin : public InverseKinBase {

public:
  InverseKin() {
    // Inverse kinematic parameters default values
    error_weights = Eigen::Matrix<double, 6, 6>::Identity();
    eps = 1e-3;
    IT_MAX = 20000;
    DT = 1e-1;
    damp = 1e-6;
  }

  void initialize(joint_trajectory_publisher::Params::InverseKin params) {
    error_weights = Eigen::Matrix<double, 6, 6>::Identity();
    for (int idx = 0; idx < 6; idx++)
      error_weights(idx, idx) = params.error_weights[idx];
    eps = params.eps;
    IT_MAX = params.max_iter;
    DT = params.dt;
    damp = params.damp;
  }

  std::tuple<Eigen::VectorXd, double> get_inverse_kinematics_for_des_pose(Eigen::VectorXd q_init,
                                                                          const pinocchio::SE3 &in_world_M_des_pose) {
    Eigen::VectorXd q = q_init;
    pinocchio::Data::Matrix6x J(6, model_ptr_->nv);
    J.setZero();

    bool success = false;
    Eigen::Matrix<double, 6, 1> err;
    Eigen::VectorXd v(model_ptr_->nv);
    for (int i = 0;; i++) {
      pinocchio::framesForwardKinematics(*model_ptr_, *data_ptr_, q);
      const pinocchio::SE3 iMd = data_ptr_->oMf[ee_frame_id_].actInv(in_world_M_des_pose);
      err = pinocchio::log6(iMd).toVector();
      err = error_weights * err;
      if (i == 0) {
        std::cout << "start IK, des transform :\n" << in_world_M_des_pose << std::endl;
      }
      if (err.norm() < eps) {
        success = true;
        break;
      }
      if (i >= IT_MAX) {
        success = false;
        break;
      }
      pinocchio::computeFrameJacobian(*model_ptr_, *data_ptr_, q, ee_frame_id_, pinocchio::ReferenceFrame::LOCAL, J);
      pinocchio::Data::Matrix6 Jlog;
      pinocchio::Jlog6(iMd.inverse(), Jlog);
      J = -Jlog * J;
      pinocchio::Data::Matrix6 JJt;
      JJt.noalias() = J * J.transpose();
      JJt.diagonal().array() += damp;
      v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
      q = pinocchio::integrate(*model_ptr_, q, v * DT);
      set_q_in_joint_limits(q);
    }

    return std::make_tuple(q, err.norm());
  }

private:
  // Inverse kinematics parameters
  Eigen::Matrix<double, 6, 6> error_weights; // square matrix with weights in the diagonal
  double eps, DT, damp;
  int IT_MAX;
};