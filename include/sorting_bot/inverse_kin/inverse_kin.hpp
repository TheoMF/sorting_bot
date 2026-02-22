#ifndef SORTING_BOT_INVERSE_KIN_
#define SORTING_BOT_INVERSE_KIN_

#include "sorting_bot/inverse_kin/inverse_kin_base.hpp"

namespace sorting_bot {

using InverseKinParams = joint_trajectory_publisher::Params::InverseKin;

class InverseKin : public InverseKinBase {

public:
  InverseKin();

  void initialize(const std::shared_ptr<pinocchio::Model> &model, const std::shared_ptr<pinocchio::Data> &data,
                  const int &ee_frame_id, InverseKinBaseParams &base_params, InverseKinParams &params);

  std::tuple<Eigen::VectorXd, double> get_inverse_kinematics(Eigen::VectorXd q_init,
                                                             const pinocchio::SE3 &in_world_M_des_pose) const;

private:
  // Inverse kinematics parameters
  double dt_, damp_, min_precision_threshold_;
  int max_iter_;
};

} // namespace sorting_bot

#endif