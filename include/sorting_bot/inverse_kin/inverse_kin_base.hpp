#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/model.hpp"

#ifndef __sorting_bot_inverse_kin_base__
#define __sorting_bot_inverse_kin_base__
class InverseKinBase {
public:
  void initialize_model(std::shared_ptr<pinocchio::Model> model, std::shared_ptr<pinocchio::Data> data,
                        int ee_frame_id);

  void set_q_in_joint_limits(Eigen::VectorXd &q) const;

protected:
  // Pinocchio model attributes
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  int ee_frame_id_;
  int nq_;

  double limit_margin = 0.01;
};
#endif