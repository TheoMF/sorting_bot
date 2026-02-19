#include "sorting_bot/inverse_kin/genetic_algo_inverse_kin.hpp"

namespace sorting_bot {

typedef Eigen::Matrix<double, 5, 1> Vector5d;
Individual::Individual(Eigen::VectorXd q, std::shared_ptr<pinocchio::Model> model,
                       std::shared_ptr<pinocchio::Data> data, const int &ee_frame_id,
                       const pinocchio::SE3 &in_world_M_des_pose, const double &mutation_max_amplitude)
    : in_world_M_des_pose_(in_world_M_des_pose), q_(q) {
  mutation_lower_bound_ = -mutation_max_amplitude / 2.0;
  mutation_upper_bound_ = mutation_max_amplitude / 2.0;
  initialize_model(model, data, ee_frame_id);
  set_score(q);
}

Eigen::VectorXd Individual::q() const { return q_; }

double Individual::score() const { return score_; }

void Individual::set_score(const Eigen::VectorXd &q) {
  pinocchio::framesForwardKinematics(*model_, *data_, q);
  const pinocchio::SE3 iMd = data_->oMf[ee_frame_id_].actInv(in_world_M_des_pose_);
  Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();
  Eigen::Matrix<double, 5, 1> err_5d;
  err_5d.head<3>() = err.head<3>();
  err_5d.tail<2>() = err.tail<2>();
  score_ = err_5d.norm();
}

Individual Individual::cross(const Individual &other_indiv) {
  Eigen::VectorXd cross_q = (q_ + other_indiv.q()) / 2.0;
  return Individual(cross_q, model_, data_, ee_frame_id_, in_world_M_des_pose_, mutation_upper_bound_ * 2.0);
}

void Individual::mutate() {
  std::uniform_real_distribution<double> unif(mutation_lower_bound_, mutation_upper_bound_);

  static std::default_random_engine re;

  // Getting a random double value
  for (int joint_idx = 0; joint_idx < model_->nq; joint_idx++) {
    double rand_value = unif(re);
    q_[joint_idx] += rand_value;
  }
  set_q_in_joint_limits(q_);
  set_score(q_);
}

GeneticAlgoInverseKin::GeneticAlgoInverseKin() {
  // Genetic algo parameters default values
  population_size_ = 3000;
  max_iter_ = 30;
  nb_keep_ind_ = 400;
  eps_ = 1e-3;
  mutation_max_amplitude_ = 0.1;
}

void GeneticAlgoInverseKin::initialize(joint_trajectory_publisher::Params::GeneticAlgoInverseKin params) {
  population_size_ = params.population_size;
  max_iter_ = params.max_iter;
  nb_keep_ind_ = params.nb_keep_ind;
  eps_ = params.convergence_threshold;
  mutation_max_amplitude_ = params.mutation_max_amplitude;
}

std::vector<Individual> GeneticAlgoInverseKin::initialize_population(const pinocchio::SE3 &in_world_M_des_pose) const {
  // Initialize randomization attributes and population.
  std::uniform_real_distribution<double> unif(0.0, 1.0);
  std::default_random_engine re;
  std::vector<Individual> population;

  // Fill population;
  for (int indiv_idx = 0; indiv_idx < population_size_; indiv_idx++) {
    // Compute random configuration.
    Eigen::Matrix<double, 5, 1> random_q = Eigen::Matrix<double, 5, 1>::Zero();
    for (int joint_idx = 0; joint_idx < nq_; joint_idx++) {
      double rand_value = unif(re);
      random_q[joint_idx] =
          model_->lowerPositionLimit[joint_idx] +
          rand_value * (model_->upperPositionLimit[joint_idx] - model_->lowerPositionLimit[joint_idx]);
    }

    population.push_back(
        Individual(random_q, model_, data_, ee_frame_id_, in_world_M_des_pose, mutation_max_amplitude_));
  }

  std::sort(population.begin(), population.end(),
            [](const Individual &indiv_1, const Individual &indiv_2) { return indiv_1.score() < indiv_2.score(); });
  return population;
}

Individual GeneticAlgoInverseKin::run_gen_algo(const pinocchio::SE3 &in_world_M_des_pose) const {
  // Initialization.
  double best_score = 100.0;
  std::uniform_real_distribution<double> unif(0, population_size_ - 1);
  std::default_random_engine re;
  std::vector<Individual> new_population, population = initialize_population(in_world_M_des_pose);
  int iter = 0;

  while (iter < max_iter_ && best_score > eps_) {
    Individual best_indiv = population[0];
    new_population.clear();

    // Keep individuals of previous population by rank.
    for (int i = 0; i < nb_keep_ind_; i++)
      new_population.push_back(population[i]);

    // Keep individuals of previous population. by tournament.
    for (int idx = nb_keep_ind_; idx < population_size_ / 2; idx++) {
      // Randomly select two different individual.
      int indiv_idx1 = int(unif(re));
      int indiv_idx2 = int(unif(re));
      while (indiv_idx2 == indiv_idx1)
        indiv_idx2 = unif(re);

      if (population[indiv_idx1].score() < population[indiv_idx2].score())
        new_population.push_back(population[indiv_idx1]);
      else
        new_population.push_back(population[indiv_idx2]);
    }

    // Create new individuals using cross and mutation.
    for (int idx = 0; idx < population_size_ / 2; idx++) {
      int indiv_idx1 = int(unif(re));
      int indiv_idx2 = int(unif(re));
      while (indiv_idx2 == indiv_idx1)
        indiv_idx2 = unif(re);
      Individual new_indiv = population[indiv_idx1].cross(population[indiv_idx2]);
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
