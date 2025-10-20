#include <iostream>
#include <vector>
#include <math.h>
#include <memory>
#include <random>
#include <ranges>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "sorting_bot/quintic_polynom.hpp"
#include "sorting_bot/genetic_algo_inv_kin.hpp"

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
        genetic_algo_inv_kin_.initialize(model_ptr_,data_ptr_,ee_frame_id_);
    }

    void initialize_population(const pinocchio::SE3 &in_world_M_des_pose)
    {
        genetic_algo_inv_kin_.initialize_population(in_world_M_des_pose);
    }

    Individual run_gen_algo(const pinocchio::SE3 &in_world_M_des_pose)
    {
        return genetic_algo_inv_kin_.run_gen_algo(in_world_M_des_pose);
    }

    pinocchio::SE3 get_frame_pose_at_q(Eigen::VectorXd &q, std::string &frame_name)
    {
        pinocchio::framesForwardKinematics(*model_ptr_, *data_ptr_, q);
        auto frame_id = model_ptr_->getFrameId(frame_name);
        return data_ptr_->oMf[frame_id];
    }

    void set_q_in_joint_limits(Eigen::VectorXd &q)
    {
        for (int joint_idx = 0; joint_idx < model_ptr_->nq; joint_idx++)
        {
            if (q[joint_idx] < model_ptr_->lowerPositionLimit[joint_idx])
                q[joint_idx] = model_ptr_->lowerPositionLimit[joint_idx];
            if (q[joint_idx] > model_ptr_->upperPositionLimit[joint_idx])
                q[joint_idx] = model_ptr_->upperPositionLimit[joint_idx];
        }
    }

    bool is_initialized()
    {
        return quintic_polynom_.is_initialized();
    }

    std::tuple<Eigen::VectorXd, double> get_inverse_kinematics_for_des_pose(Eigen::VectorXd q_init, const pinocchio::SE3 &in_world_M_des_pose)
    {
        Eigen::VectorXd q = q_init;
        pinocchio::Data::Matrix6x J(6, model_ptr_->nv);
        J.setZero();

        bool success = false;
        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        typedef Eigen::Matrix<double, 5, 1> Vector4d;
        Vector6d err;
        Eigen::VectorXd v(model_ptr_->nv);
        for (int i = 0;; i++)
        {
            pinocchio::framesForwardKinematics(*model_ptr_, *data_ptr_, q);
            const pinocchio::SE3 iMd = data_ptr_->oMf[ee_frame_id_].actInv(in_world_M_des_pose);
            err = pinocchio::log6(iMd).toVector();
            err(3) *= 0.001;
            err(5) *= 0.2;
            if (err.norm() < eps)
            {
                success = true;
                break;
            }
            if (i >= IT_MAX)
            {
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
        }
        set_q_in_joint_limits(q);
        return std::make_tuple(q, err.norm());
    }

    void set_plan(const Eigen::VectorXd &q_start, const std::vector<Eigen::VectorXd> &q_waypoints)
    {
        quintic_polynom_.set_plan(q_start,q_waypoints);
    }

    Eigen::VectorXd get_configuration_at_t(const double &time)
    {
        return quintic_polynom_.get_configuration_at_t(time);
    }

private:
    std::shared_ptr<pinocchio::Model> model_ptr_;
    std::shared_ptr<pinocchio::Data> data_ptr_;
    std::string ee_frame_name_;
    int ee_frame_id_;
    int nq_;
    QuinticPolynom quintic_polynom_;
    GeneticAlgoInvKin  genetic_algo_inv_kin_;

    

    // Inverse kinematics parameters
    const double eps = 1e-3;
    const int IT_MAX = 20000;
    const double DT = 1e-1;
    const double damp = 1e-6;

};