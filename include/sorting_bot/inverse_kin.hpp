#include "sorting_bot/inverse_kin_base.hpp"

#include "sorting_bot/joint_trajectory_publisher_parameters.hpp"

class InverseKin : public InverseKinBase {

public:
  InverseKin();

  void initialize(joint_trajectory_publisher::Params::InverseKin params);

  std::tuple<Eigen::VectorXd, double> get_inverse_kinematics(Eigen::VectorXd q_init,
                                                             const pinocchio::SE3 &in_world_M_des_pose) const;

private:
  // Inverse kinematics parameters
  Eigen::Matrix<double, 6, 6> error_weights_; // square matrix with weights in the diagonal
  double convergence_threshold_, dt_, damp_, min_precision_threshold_;
  int max_iter_;
};