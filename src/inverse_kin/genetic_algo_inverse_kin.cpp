#include "sorting_bot/inverse_kin/genetic_algo_inverse_kin.hpp"

namespace sorting_bot {

Individual::Individual(const Eigen::VectorXd &q, const std::shared_ptr<pinocchio::Model> &model,
                       const std::shared_ptr<pinocchio::Data> &data, const int &ee_frame_id,
                       const pinocchio::SE3 &des_in_world_M_gripper, const double &mutation_max_amplitude,
                       const Eigen::Matrix<double, 6, 6> &error_weights, const double &convergence_threshold)
    : des_in_world_M_gripper_(des_in_world_M_gripper), q_(q) {
  initialize(model, data, ee_frame_id, error_weights, convergence_threshold);
  mutation_amplitude_ = mutation_max_amplitude;
  set_score(q);
}

Eigen::VectorXd Individual::get_q() const { return q_; }

double Individual::score() const { return score_; }

void Individual::set_score(const Eigen::VectorXd &q) {
  pinocchio::framesForwardKinematics(*model_, *data_, q);
  const pinocchio::SE3 in_current_gripper_M_des_gripper = data_->oMf[gripper_frame_id_].actInv(des_in_world_M_gripper_);
  Eigen::Matrix<double, 6, 1> error = error_weights_ * pinocchio::log6(in_current_gripper_M_des_gripper).toVector();
  score_ = error.norm();
}

Individual Individual::cross(const Individual &other_indiv) const {
  Eigen::VectorXd cross_q = (q_ + other_indiv.get_q()) / 2.0;
  return Individual(cross_q, model_, data_, gripper_frame_id_, des_in_world_M_gripper_, mutation_amplitude_,
                    error_weights_, convergence_threshold_);
}

void Individual::mutate() {
  // Add random value within mutation bounds and with respect to joint limits.
  for (int joint_idx = 0; joint_idx < model_->nq; joint_idx++) {
    double rand_mutation_value = get_rand_value_0_to_1() * mutation_amplitude_ - mutation_amplitude_ / 2.0;
    q_[joint_idx] += rand_mutation_value;
  }
  set_q_in_joint_limits(q_);

  set_score(q_);
}

GeneticAlgoInverseKin::GeneticAlgoInverseKin() {
  // Genetic algo parameters default values
  population_size_ = 3000;
  max_iter_ = 30;
  nb_keep_ind_ = 400;
  mutation_amplitude_ = 0.1;
}

void GeneticAlgoInverseKin::initialize(const std::shared_ptr<pinocchio::Model> &model,
                                       const std::shared_ptr<pinocchio::Data> &data, const int &ee_frame_id,
                                       InverseKinBaseParams &base_params, GeneticAlgoInverseKinParams &params) {
  InverseKinBase::initialize(model, data, ee_frame_id, base_params);
  population_size_ = params.population_size;
  max_iter_ = params.max_iter;
  nb_keep_ind_ = params.nb_keep_ind;
  mutation_amplitude_ = params.mutation_amplitude;
}

std::vector<Individual>
GeneticAlgoInverseKin::initialize_population(const pinocchio::SE3 &des_in_world_M_gripper) const {
  // Initialize randomization and population variables.
  std::vector<Individual> population;

  // Fill population;
  for (int indiv_idx = 0; indiv_idx < population_size_; indiv_idx++) {
    // Compute random configuration.
    Eigen::Matrix<double, 5, 1> random_q = Eigen::Matrix<double, 5, 1>::Zero();
    for (int joint_idx = 0; joint_idx < nq_; joint_idx++)
      random_q[joint_idx] =
          model_->lowerPositionLimit[joint_idx] +
          get_rand_value_0_to_1() * (model_->upperPositionLimit[joint_idx] - model_->lowerPositionLimit[joint_idx]);

    population.push_back(Individual(random_q, model_, data_, gripper_frame_id_, des_in_world_M_gripper,
                                    mutation_amplitude_, error_weights_, convergence_threshold_));
  }

  std::sort(population.begin(), population.end(),
            [](const Individual &indiv_1, const Individual &indiv_2) { return indiv_1.score() < indiv_2.score(); });
  return population;
}

std::pair<int, int> GeneticAlgoInverseKin::get_two_individuals_idx() const {
  int indiv_idx1 = int(get_rand_value_0_to_1() * (population_size_ - 1));
  int indiv_idx2 = int(get_rand_value_0_to_1() * (population_size_ - 1));
  while (indiv_idx2 == indiv_idx1)
    indiv_idx2 = int(get_rand_value_0_to_1() * (population_size_ - 1));
  return std::make_pair(indiv_idx1, indiv_idx2);
}

Individual GeneticAlgoInverseKin::run_gen_algo(const pinocchio::SE3 &des_in_world_M_gripper) const {
  // Initialization.
  double best_score = 100.0;
  std::vector<Individual> new_population, population = initialize_population(des_in_world_M_gripper);
  int iter = 0;

  while (iter < max_iter_ && best_score > convergence_threshold_) {
    Individual best_indiv = population[0];
    new_population.clear();

    // Keep individuals of previous population by rank.
    for (int i = 0; i < nb_keep_ind_; i++)
      new_population.push_back(population[i]);

    // Keep individuals of previous population. by tournament.
    for (int idx = nb_keep_ind_; idx < population_size_ / 2; idx++) {
      // Randomly select two different individual.
      std::pair<int, int> indiv_idxs = get_two_individuals_idx();
      if (population[indiv_idxs.first].score() < population[indiv_idxs.second].score())
        new_population.push_back(population[indiv_idxs.first]);
      else
        new_population.push_back(population[indiv_idxs.second]);
    }

    // Create new individuals using cross and mutation.
    for (int idx = 0; idx < population_size_ / 2; idx++) {
      std::pair<int, int> indiv_idxs = get_two_individuals_idx();
      Individual new_indiv = population[indiv_idxs.first].cross(population[indiv_idxs.second]);
      new_indiv.mutate();
      new_population.push_back(new_indiv);
    }

    // Update population.
    population = new_population;
    std::sort(population.begin(), population.end(),
              [](const Individual &a, const Individual &b) { return a.score() < b.score(); });

    // Update iteration number with same best individual.
    Individual new_best_indiv = population[0];
    best_score = new_best_indiv.score();
    if (new_best_indiv.score() == best_indiv.score())
      iter++;
    else
      iter = 0;
  }

  return population[0];
}

} // namespace sorting_bot
