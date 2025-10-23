#include <iostream>

#include "pinocchio/parsers/urdf.hpp"

#include "sorting_bot/quintic_polynom.hpp"
#include "sorting_bot/genetic_algo_inverse_kin.hpp"
#include "sorting_bot/inverse_kin.hpp"

class MotionPlanner
{
public:
  MotionPlanner()
  {
  }
  void initialize(std::string filename, std::string ee_frame_name)
  {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(filename, model);
    std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id{model.getJointId("gripper")};
    Eigen::VectorXd q_rand = pinocchio::randomConfiguration(model);
    model = pinocchio::buildReducedModel(model, list_of_joints_to_lock_by_id, q_rand);
    model_ptr_ = std::make_shared<pinocchio::Model>(model);
    ee_frame_name_ = ee_frame_name;
    pinocchio::Data data(*model_ptr_);
    nq_ = model_ptr_->nq;
    data_ptr_ = std::make_shared<pinocchio::Data>(data);
    ee_frame_id_ = model_ptr_->getFrameId(ee_frame_name_);
    genetic_algo_inverse_kin_.initialize(model_ptr_, data_ptr_, ee_frame_id_);
    inverse_kin_.initialize(model_ptr_, data_ptr_, ee_frame_id_);
  }

  Eigen::VectorXd get_inverse_kinematic_at_pose(const Eigen::VectorXd &q_init, const pinocchio::SE3 &des_transform)
  {
    Eigen::VectorXd q_inv_kin;
    std::tuple<Eigen::VectorXd, double> inv_kin_res = inverse_kin_.get_inverse_kinematics_for_des_pose(q_init, des_transform);
    double pose_err = std::get<1>(inv_kin_res);
    if (pose_err > 0.01)
    {
      std::cout << "IK didn't converge, Run genetic algorithm" << std::endl;
      Individual best = genetic_algo_inverse_kin_.run_gen_algo(des_transform);
      q_inv_kin = best.q();
    }
    else
    {
      q_inv_kin = std::get<0>(inv_kin_res);
    }
    std::string ee_frame_name = "gripper_frame_link";
    pinocchio::SE3 pose_inv_kin = get_frame_pose_at_q(q_inv_kin, ee_frame_name);
    Eigen::Matrix<double, 6, 1> err = pinocchio::log6(pose_inv_kin.actInv(des_transform)).toVector();
    Eigen::Matrix<double, 5, 1> err_5d;
    err_5d.head<3>() = err.head<3>();
    err_5d.tail<2>() = err.tail<2>();
    std::cout << "inverse kinematic pose error norm " << err_5d.norm() << "base IK err " << pose_err << std::endl;
    return q_inv_kin;
  }

  pinocchio::SE3 get_frame_pose_at_q(Eigen::VectorXd &q, std::string &frame_name)
  {
    pinocchio::framesForwardKinematics(*model_ptr_, *data_ptr_, q);
    auto frame_id = model_ptr_->getFrameId(frame_name);
    return data_ptr_->oMf[frame_id];
  }

  void set_plan(const Eigen::VectorXd &q_start, const std::vector<Eigen::VectorXd> &q_waypoints)
  {
    quintic_polynom_.set_plan(q_start, q_waypoints);
  }

  Eigen::VectorXd get_configuration_at_t(const double &time)
  {
    return quintic_polynom_.get_configuration_at_t(time);
  }

  double get_traj_duration()
  {
    return quintic_polynom_.traj_duration();
  }

private:
  std::shared_ptr<pinocchio::Model> model_ptr_;
  std::shared_ptr<pinocchio::Data> data_ptr_;
  std::string ee_frame_name_;
  int ee_frame_id_;
  int nq_;
  QuinticPolynom quintic_polynom_;
  GeneticAlgoInverseKin genetic_algo_inverse_kin_;
  InverseKin inverse_kin_;
};