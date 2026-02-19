#ifndef SORTING_BOT_QUINTIC_POLYNOM_HPP_
#define SORTING_BOT_QUINTIC_POLYNOM_HPP_

#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace sorting_bot {

class QuinticPolynom {
public:
  QuinticPolynom() {}

  void set_motion_planning_time_coeff(const double &motion_planning_time_coeff);

  void set_motion_planning(const Eigen::VectorXd &q_start, const std::vector<Eigen::VectorXd> &q_waypoints);

  int get_current_waypoint_idx(const double &time) const;

  std::tuple<Eigen::VectorXd, Eigen::VectorXd, bool> get_traj_value_at_t(const double &time) const;

  double traj_duration() const;

  bool is_initialized();

  std::vector<Eigen::VectorXd> q_waypoints_;

private:
  bool is_initialized_ = false;
  int nq_ = 5;

  // Quintic trajectory attributes
  std::vector<double> waypoints_traj_duration_;
  std::vector<double> waypoints_start_time_, waypoints_end_time_;
  Eigen::Vector3d polynom_coeffs_ = Eigen::Vector3d(10.0, -15.0, 6.0);
  Eigen::VectorXd q_start_;
  double traj_duration_, motion_planning_time_coeff_;
};

} // namespace sorting_bot

#endif