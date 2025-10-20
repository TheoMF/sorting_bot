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
        genetic_algo_inverse_kin_.initialize(model_ptr_,data_ptr_,ee_frame_id_);
        inverse_kin_.initialize(model_ptr_,data_ptr_,ee_frame_id_);
    }

    void initialize_population(const pinocchio::SE3 &in_world_M_des_pose)
    {
        genetic_algo_inverse_kin_.initialize_population(in_world_M_des_pose);
    }

    Individual run_gen_algo(const pinocchio::SE3 &in_world_M_des_pose)
    {
        return genetic_algo_inverse_kin_.run_gen_algo(in_world_M_des_pose);
    }

    pinocchio::SE3 get_frame_pose_at_q(Eigen::VectorXd &q, std::string &frame_name)
    {
        pinocchio::framesForwardKinematics(*model_ptr_, *data_ptr_, q);
        auto frame_id = model_ptr_->getFrameId(frame_name);
        return data_ptr_->oMf[frame_id];
    }

    bool is_initialized()
    {
        return quintic_polynom_.is_initialized();
    }

    std::tuple<Eigen::VectorXd, double> get_inverse_kinematics_for_des_pose(Eigen::VectorXd q_init, const pinocchio::SE3 &in_world_M_des_pose)
    {
        return inverse_kin_.get_inverse_kinematics_for_des_pose(q_init,in_world_M_des_pose);
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
    GeneticAlgoInverseKin  genetic_algo_inverse_kin_;
    InverseKin inverse_kin_;
};