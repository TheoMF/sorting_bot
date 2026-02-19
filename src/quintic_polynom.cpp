#include "sorting_bot/quintic_polynom.hpp"

namespace sorting_bot {

void QuinticPolynom::set_motion_planning_time_coeff(const double &motion_planning_time_coeff) {
  motion_planning_time_coeff_ = motion_planning_time_coeff;
}
void QuinticPolynom::set_motion_planning(const Eigen::VectorXd &q_start,
                                         const std::vector<Eigen::VectorXd> &q_waypoints) {
  // Reset motion planning attributes.
  q_waypoints_.clear();
  waypoints_traj_duration_.clear();
  waypoints_start_time_.clear();
  waypoints_end_time_.clear();
  q_start_ = q_start;
  q_waypoints_ = q_waypoints;

  // Compute distance between each waypoints.
  std::vector<double> dist_to_next_goals;
  Eigen::VectorXd previous_q = q_start;
  double total_dist = 0.;
  for (const Eigen::VectorXd &q_goal : q_waypoints_) {
    double dist_to_next_goal = (q_goal - previous_q).norm();
    dist_to_next_goals.push_back(dist_to_next_goal);
    total_dist += dist_to_next_goal;
    previous_q = q_goal;
  }

  // Compute total trajectory duration.
  traj_duration_ = total_dist * motion_planning_time_coeff_;

  // Compute for each sub-trajectory when it starts and when it ends.
  double start_time = 0.;
  for (double &dist_to_next_goal : dist_to_next_goals) {
    double waypoint_traj_duration = traj_duration_ * dist_to_next_goal / total_dist;
    waypoints_traj_duration_.push_back(waypoint_traj_duration);
    waypoints_start_time_.push_back(start_time);
    waypoints_end_time_.push_back(start_time + waypoint_traj_duration);
    start_time += waypoint_traj_duration;
  }

  is_initialized_ = true;
}

int QuinticPolynom::get_current_waypoint_idx(const double &time) const {
  for (int waypoint_idx = 0; waypoint_idx < waypoints_traj_duration_.size(); waypoint_idx++) {
    if (time >= waypoints_start_time_[waypoint_idx] && time < waypoints_end_time_[waypoint_idx])
      return waypoint_idx;
  }
  return -1;
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, bool> QuinticPolynom::get_traj_value_at_t(const double &time) const {
  // Handle cases where time is out of trajectory bounds.
  if (time <= 0.)
    return std::make_tuple(q_start_, Eigen::VectorXd::Zero(nq_), false);
  if (time >= traj_duration_)
    return std::make_tuple(q_waypoints_.back(), Eigen::VectorXd::Zero(nq_), true);
  int current_waypoint_idx = get_current_waypoint_idx(time);
  if (current_waypoint_idx == -1)
    return std::make_tuple(q_start_, Eigen::VectorXd::Zero(nq_), false);

  // Find current trajectory q_init and q_goal.
  Eigen::VectorXd q_init, q_goal;
  if (current_waypoint_idx == 0)
    q_init = q_start_;
  else
    q_init = q_waypoints_[current_waypoint_idx - 1];
  q_goal = q_waypoints_[current_waypoint_idx];

  // Compute quintic polynom coefficients.
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nq_), q_dot = Eigen::VectorXd::Zero(nq_);
  double time_scale = (time - waypoints_start_time_[current_waypoint_idx]) /
                      (waypoints_end_time_[current_waypoint_idx] - waypoints_start_time_[current_waypoint_idx]);
  double pose_polynom_val = polynom_coeffs_[0] * std::pow(time_scale, 3) +
                            polynom_coeffs_[1] * std::pow(time_scale, 4) + polynom_coeffs_[2] * std::pow(time_scale, 5);
  double vel_polynom_val = 3. * polynom_coeffs_[0] * std::pow(time_scale, 2) +
                           4. * polynom_coeffs_[1] * std::pow(time_scale, 3) +
                           5. * polynom_coeffs_[2] * std::pow(time_scale, 4);

  // Compute q and q_dot.
  for (int joint_idx = 0; joint_idx < 5; joint_idx++) {
    q[joint_idx] = q_init[joint_idx] + pose_polynom_val * (q_goal[joint_idx] - q_init[joint_idx]);
    q_dot[joint_idx] = vel_polynom_val * (q_goal[joint_idx] - q_init[joint_idx]);
  }

  return std::make_tuple(q, q_dot, false);
}

double QuinticPolynom::traj_duration() const { return traj_duration_; }

bool QuinticPolynom::is_initialized() { return is_initialized_; }

} // namespace sorting_bot
