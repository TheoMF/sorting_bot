#include <iostream>
#include <fstream>

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
  void initialize(std::string urdf, std::string ee_frame_name, joint_trajectory_publisher::Params params)
  {
    // Build pinocchio reduced model.
    pinocchio::Model model;
    pinocchio::urdf::buildModelFromXML(urdf, model);
    std::vector<std::string> joints_to_lock_names;
    if (params.robot_name == "LeKiwi")
      joints_to_lock_names = {"gripper", "rear_wheel_drive", "left_wheel_drive", "right_wheel_drive"};
    else if (params.robot_name == "SO-101")
      joints_to_lock_names = {"gripper"};
    std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id = {};
    for (std::string &joint_to_lock_name : joints_to_lock_names)
    {
      list_of_joints_to_lock_by_id.push_back(model.getJointId(joint_to_lock_name));
      std::cout << "joint to lock id " << model.getJointId(joint_to_lock_name) << std::endl;
    }

    Eigen::VectorXd q_rand = pinocchio::randomConfiguration(model);
    model = pinocchio::buildReducedModel(model, list_of_joints_to_lock_by_id, q_rand);
    std::cout << "new nq " << model.nq << std::endl;

    // Initialize remaining attributes.
    model_ptr_ = std::make_shared<pinocchio::Model>(model);
    ee_frame_name_ = ee_frame_name;
    pinocchio::Data data(*model_ptr_);
    nq_ = model_ptr_->nq;
    data_ptr_ = std::make_shared<pinocchio::Data>(data);
    ee_frame_id_ = model_ptr_->getFrameId(ee_frame_name_);
    use_genetic_algo_ = params.use_genetic_algo;
    genetic_algo_inverse_kin_.initialize_model(model_ptr_, data_ptr_, ee_frame_id_);
    inverse_kin_.initialize_model(model_ptr_, data_ptr_, ee_frame_id_);
    inverse_kin_.initialize(params.inverse_kin);
    genetic_algo_inverse_kin_.initialize(params.genetic_algo_inverse_kin);
    quintic_polynom_.set_motion_planning_time_coeff(params.motion_planning_time_coeff);
  }

  Eigen::VectorXd get_inverse_kinematic_at_pose(const Eigen::VectorXd &q_init, const pinocchio::SE3 &des_transform)
  {
    Eigen::VectorXd q_inv_kin;
    std::tuple<Eigen::VectorXd, double> inv_kin_res = inverse_kin_.get_inverse_kinematics_for_des_pose(q_init, des_transform);
    double pose_err = std::get<1>(inv_kin_res);
    if (pose_err > 0.01 && use_genetic_algo_)
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
    std::cout << "inverse kinematic pose error norm " << err_5d.norm() << "base IK err " << pose_err << " q inv \n"
              << q_inv_kin
              << std::endl;

    if (pose_err > 0.05)
    {
      std::cout << "didnt converge, returning q_init" << std::endl;
      return q_init;
    }
    return q_inv_kin;
  }

  void print_ee_pose(const Eigen::VectorXd &q)
  {
    pinocchio::framesForwardKinematics(*model_ptr_, *data_ptr_, q);
    const pinocchio::SE3 iMd = data_ptr_->oMf[ee_frame_id_];
    std::cout << "in_base_M_ee " << iMd << std::endl;
    auto quat = pinocchio::SE3::Quaternion(iMd.rotation());
    std::cout << "in_base_M_ee quat " << quat << std::endl;
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

  std::tuple<Eigen::VectorXd, Eigen::VectorXd, double> get_traj_value_at_t(const double &time)
  {
    return quintic_polynom_.get_traj_value_at_t(time);
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
  bool use_genetic_algo_;
  QuinticPolynom quintic_polynom_;
  GeneticAlgoInverseKin genetic_algo_inverse_kin_;
  InverseKin inverse_kin_;
};