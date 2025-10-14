#include <iostream>
#include <vector>
#include <math.h>
#include <memory>
#include <random>
#include <ranges>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"

class Individual
{
public:
    typedef Eigen::Matrix<double, 5, 1> Vector5d;
    Individual(Eigen::VectorXd q, std::shared_ptr<pinocchio::Model> model_ptr, std::shared_ptr<pinocchio::Data> data_ptr, int ee_frame_id, const pinocchio::SE3 &in_world_M_des_pose)
        : ee_frame_id_(ee_frame_id), in_world_M_des_pose_(in_world_M_des_pose)
    {
        q_ = q;
        model_ptr_ = model_ptr;
        data_ptr_ = data_ptr;
        set_score(q);
    }

    Eigen::VectorXd q() const
    {
        return q_;
    }
    double score() const
    {
        return score_;
    }
    void set_score(const Eigen::VectorXd &q)
    {
        pinocchio::framesForwardKinematics(*model_ptr_, *data_ptr_, q);
        const pinocchio::SE3 iMd = data_ptr_->oMf[ee_frame_id_].actInv(in_world_M_des_pose_);
        Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();
        Eigen::Matrix<double, 5, 1> err_5d;
        err_5d.head<3>() = err.head<3>();
        err_5d.tail<2>() = err.tail<2>();
        score_ = err_5d.norm();
    }

    Individual cross(const Individual &other_indiv)
    {
        Eigen::VectorXd cross_q = (q_ + other_indiv.q()) / 2.0;
        return Individual(cross_q, model_ptr_, data_ptr_, ee_frame_id_, in_world_M_des_pose_);
    }

    void mutate()
    {
        double lower_bound = -0.05;
        double upper_bound = 0.05;

        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);

        static std::default_random_engine re;

        // Getting a random double value

        for (int joint_idx = 0; joint_idx < model_ptr_->nq; joint_idx++)
        {
            double rand_value = unif(re);
            q_[joint_idx] += rand_value;
        }
        set_q_in_joint_limits();
        set_score(q_);
    }

    void set_q_in_joint_limits()
    {
        for (int joint_idx = 0; joint_idx < model_ptr_->nq; joint_idx++)
        {
            if (q_[joint_idx] < model_ptr_->lowerPositionLimit[joint_idx])
                q_[joint_idx] = model_ptr_->lowerPositionLimit[joint_idx];
            if (q_[joint_idx] > model_ptr_->upperPositionLimit[joint_idx])
                q_[joint_idx] = model_ptr_->upperPositionLimit[joint_idx];
        }
    }

private:
    std::vector<double> joint_amplitudes;
    int ee_frame_id_;
    pinocchio::SE3 in_world_M_des_pose_;
    std::shared_ptr<pinocchio::Model> model_ptr_;
    std::shared_ptr<pinocchio::Data> data_ptr_;
    Eigen::VectorXd q_;
    double score_;
};

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
    }

    void initialize_population(const pinocchio::SE3 &in_world_M_des_pose)
    {
        double lower_bound = 0.;
        double upper_bound = 1.;
        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
        std::default_random_engine re;
        population_.clear();
        for (int indiv_idx = 0; indiv_idx < population_size_; indiv_idx++)
        {
            Eigen::Matrix<double, 5, 1> random_q = Eigen::Matrix<double, 5, 1>::Zero();
            for (int joint_idx = 0; joint_idx < nq_; joint_idx++)
            {
                double rand_value = unif(re);
                random_q[joint_idx] = model_ptr_->lowerPositionLimit[joint_idx] + rand_value * (model_ptr_->upperPositionLimit[joint_idx] - model_ptr_->lowerPositionLimit[joint_idx]);
            }
            population_.push_back(Individual(random_q, model_ptr_, data_ptr_, ee_frame_id_, in_world_M_des_pose));
        }
        std::sort(population_.begin(), population_.end(), [](const Individual &a, const Individual &b)
                  { return a.score() < b.score(); });
    }

    Individual run_gen_algo(const pinocchio::SE3 &in_world_M_des_pose)
    {
        double best_score = 100.0;
        double lower_bound = 0;
        double upper_bound = population_size_ - 1;
        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
        std::default_random_engine re;
        initialize_population(in_world_M_des_pose);
        int iter = 0;
        while (iter < max_iter_ && best_score > eps_)
        {
            Individual best_indiv = population_[0];
            new_population_.clear();
            for (int i = 0; i < nb_keep_ind; i++)
                new_population_.push_back(population_[i]);
            for (int idx = nb_keep_ind; idx < population_size_ / 2; idx++)
            {
                int indiv_idx1 = int(unif(re));
                int indiv_idx2 = int(unif(re));
                while (indiv_idx2 == indiv_idx1)
                    indiv_idx2 = unif(re);
                if (population_[indiv_idx1].score() < population_[indiv_idx2].score())
                    new_population_.push_back(population_[indiv_idx1]);
                else
                    new_population_.push_back(population_[indiv_idx2]);
            }
            for (int idx = 0; idx < population_size_ / 2; idx++)
            {
                int indiv_idx1 = int(unif(re));
                int indiv_idx2 = int(unif(re));
                while (indiv_idx2 == indiv_idx1)
                    indiv_idx2 = unif(re);
                Individual new_indiv = population_[indiv_idx1].cross(population_[indiv_idx2]);
                new_indiv.mutate();
                new_population_.push_back(new_indiv);
            }
            population_ = new_population_;
            std::sort(population_.begin(), population_.end(), [](const Individual &a, const Individual &b)
                      { return a.score() < b.score(); });
            Individual new_best_indiv = population_[0];
            best_score = new_best_indiv.score();
            if (new_best_indiv.score() == best_indiv.score())
                iter++;
            else
                iter = 0;
        }

        return population_[0];
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
        return is_initialized_;
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

    void set_plan(const Eigen::VectorXd &q_start, const std::vector<Eigen::VectorXd> &q_waypoints, const double &traj_duration)
    {
        q_waypoints_.clear();
        waypoints_traj_duration_.clear();
        waypoints_start_time_.clear();
        waypoints_end_time_.clear();
        waypoints_traj_coeffs_.clear();
        // traj_duration_ = traj_duration;
        q_start_ = q_start;
        q_waypoints_ = q_waypoints;

        std::vector<double> dist_to_next_goals;
        Eigen::VectorXd previous_q = q_start;
        double total_dist = 0.;
        for (const Eigen::VectorXd &q_goal : q_waypoints_)
        {
            double dist_to_next_goal = (q_goal - previous_q).norm();
            dist_to_next_goals.push_back(dist_to_next_goal);
            total_dist += dist_to_next_goal;
            previous_q = q_goal;
        }
        traj_duration_ = total_dist * 1.3;
        if (q_waypoints.size() == 2)
        {
            traj_duration_ *= 1.2;
        }
        if (q_waypoints.size() == 3)
        {
            traj_duration_ *= 1.4;
        }

        double start_time = 0.;
        for (double &dist_to_next_goal : dist_to_next_goals)
        {
            double waypoint_traj_duration = traj_duration_ * dist_to_next_goal / total_dist;
            waypoints_traj_duration_.push_back(waypoint_traj_duration);
            waypoints_start_time_.push_back(start_time);
            waypoints_end_time_.push_back(start_time + waypoint_traj_duration);
            std::vector<double> waypoint_traj_coeffs;
            waypoint_traj_coeffs.push_back(1. / (waypoint_traj_duration * (1. - 2.5 * waypoint_traj_duration + 5. * std::pow(waypoint_traj_duration, 2) / 3.)));
            waypoint_traj_coeffs.push_back(waypoint_traj_coeffs[0] * -2.5 * waypoint_traj_duration);
            waypoint_traj_coeffs.push_back(waypoint_traj_coeffs[0] * 5. * std::pow(waypoint_traj_duration, 2) / 3.);
            waypoints_traj_coeffs_.push_back(waypoint_traj_coeffs);
            start_time += waypoint_traj_duration;
        }

        is_initialized_ = true;
    }

    Eigen::VectorXd get_configuration_at_t(const double &time)
    {
        if (time <= 0.)
            return q_start_;
        if (time >= traj_duration_)
            return q_waypoints_.back();
        Eigen::Matrix<double, 5, 1> q_init, q_goal;
        int current_waypoint_idx = -1;
        for (int waypoint_idx = 0; waypoint_idx < waypoints_traj_duration_.size(); waypoint_idx++)
        {
            if (time >= waypoints_start_time_[waypoint_idx] && time < waypoints_end_time_[waypoint_idx])
            {
                if (waypoint_idx == 0)
                    q_init = q_start_;
                else
                    q_init = q_waypoints_[waypoint_idx - 1];
                q_goal = q_waypoints_[waypoint_idx];
                current_waypoint_idx = waypoint_idx;
            }
        }
        if (current_waypoint_idx == -1)
        {
            return q_start_;
        }
        std::vector<double> traj_coeffs = waypoints_traj_coeffs_[current_waypoint_idx];
        Eigen::Matrix<double, 5, 1> q = q_init;
        for (int joint_idx = 0; joint_idx < 5; joint_idx++)
        {
            q[joint_idx] += (traj_coeffs[0] + traj_coeffs[1] + traj_coeffs[2]) * (time - waypoints_start_time_[current_waypoint_idx]) * (q_goal[joint_idx] - q_init[joint_idx]);
        }
        return q;
    }
    std::vector<Eigen::VectorXd> q_waypoints_;

private:
    std::vector<Individual> population_, new_population_;
    std::shared_ptr<pinocchio::Model> model_ptr_;
    std::shared_ptr<pinocchio::Data> data_ptr_;
    std::string ee_frame_name_;
    int ee_frame_id_;
    bool is_initialized_ = false;
    int nq_;

    // Quintic trajectory attributes
    std::vector<double> waypoints_traj_duration_;
    std::vector<double> waypoints_start_time_, waypoints_end_time_;
    std::vector<std::vector<double>> waypoints_traj_coeffs_;
    Eigen::VectorXd q_start_;
    double traj_duration_;

    // Inverse kinematics parameters
    const double eps = 1e-3;
    const int IT_MAX = 20000;
    const double DT = 1e-1;
    const double damp = 1e-6;

    // genetic algorithm parameters
    int population_size_ = 3000;
    int max_iter_ = 30;
    int nb_keep_ind = 400;
    double eps_ = 1e-3;
};