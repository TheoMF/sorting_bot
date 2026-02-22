#ifndef SORTING_BOT_GENETIC_ALGO_INVERSE_KIN_
#define SORTING_BOT_GENETIC_ALGO_INVERSE_KIN_

#include <random>

#include "sorting_bot/inverse_kin/inverse_kin_base.hpp"

namespace sorting_bot {

inline double get_rand_value_0_to_1() {
  static std::uniform_real_distribution<double> unif_distrib(0.0, 1.0);
  static std::default_random_engine rand_engine;
  return unif_distrib(rand_engine);
}

class Individual : public InverseKinBase {
public:
  Individual(const Eigen::VectorXd &q, const std::shared_ptr<pinocchio::Model> &model,
             const std::shared_ptr<pinocchio::Data> &data, const int &ee_frame_id,
             const pinocchio::SE3 &des_in_world_M_gripper, const double &mutation_amplitude,
             const Eigen::Matrix<double, 6, 6> &error_weights, const double &convergence_threshold);

  Eigen::VectorXd get_q() const;

  double score() const;

  void set_score(const Eigen::VectorXd &q);

  Individual cross(const Individual &other_indiv) const;

  void mutate();

private:
  pinocchio::SE3 des_in_world_M_gripper_;
  Eigen::VectorXd q_;
  double score_, mutation_amplitude_;
};

class GeneticAlgoInverseKin : public InverseKinBase {
public:
  using GeneticAlgoInverseKinParams =
      joint_trajectory_publisher::Params::PlannerManager::MotionPlanner::GeneticAlgoInverseKin;

  GeneticAlgoInverseKin();

  void initialize(const std::shared_ptr<pinocchio::Model> &model, const std::shared_ptr<pinocchio::Data> &data,
                  const int &ee_frame_id, InverseKinBaseParams &base_params, GeneticAlgoInverseKinParams &params);

  std::vector<Individual> initialize_population(const pinocchio::SE3 &des_in_world_M_gripper) const;

  std::pair<int, int> get_two_individuals_idx() const;

  Individual run_gen_algo(const pinocchio::SE3 &des_in_world_M_gripper) const;

private:
  // genetic algorithm parameters
  int population_size_, max_iter_, nb_keep_ind_;
  double mutation_amplitude_;
};

} // namespace sorting_bot

#endif