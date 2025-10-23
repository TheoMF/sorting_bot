#include <Eigen/Dense>
#include <vector>

class QuinticPolynom
{
public:
    QuinticPolynom() {}
    void set_plan(const Eigen::VectorXd &q_start, const std::vector<Eigen::VectorXd> &q_waypoints)
    {
        q_waypoints_.clear();
        waypoints_traj_duration_.clear();
        waypoints_start_time_.clear();
        waypoints_end_time_.clear();
        waypoints_traj_coeffs_.clear();
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

    double traj_duration(){
        return traj_duration_;
    }

    bool is_initialized()
    {
        return is_initialized_;
    }

    std::vector<Eigen::VectorXd> q_waypoints_;

private:
    bool is_initialized_ = false;
    int nq_;

    // Quintic trajectory attributes
    std::vector<double> waypoints_traj_duration_;
    std::vector<double> waypoints_start_time_, waypoints_end_time_;
    std::vector<std::vector<double>> waypoints_traj_coeffs_;
    Eigen::VectorXd q_start_;
    double traj_duration_;
};