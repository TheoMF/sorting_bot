#include <random>

#include "sorting_bot/inverse_kin/inverse_kin_base.hpp"

#include "sorting_bot/joint_trajectory_publisher_parameters.hpp"

class Individual : public InverseKinBase {
public:
  typedef Eigen::Matrix<double, 5, 1> Vector5d;
  Individual(Eigen::VectorXd q, std::shared_ptr<pinocchio::Model> model, std::shared_ptr<pinocchio::Data> data,
             const int &ee_frame_id, const pinocchio::SE3 &in_world_M_des_pose, const double &mutation_max_amplitude);

  Eigen::VectorXd q() const;

  double score() const;

  void set_score(const Eigen::VectorXd &q);

  Individual cross(const Individual &other_indiv);
  void mutate();

private:
  pinocchio::SE3 in_world_M_des_pose_;
  Eigen::VectorXd q_;
  double score_, mutation_lower_bound_, mutation_upper_bound_;
};

class GeneticAlgoInverseKin : public InverseKinBase {
public:
  GeneticAlgoInverseKin();

  void initialize(joint_trajectory_publisher::Params::GeneticAlgoInverseKin params);

  std::vector<Individual> initialize_population(const pinocchio::SE3 &in_world_M_des_pose) const;

  Individual run_gen_algo(const pinocchio::SE3 &in_world_M_des_pose) const;

private:
  // genetic algorithm parameters
  int population_size_, max_iter_, nb_keep_ind_;
  double eps_, mutation_max_amplitude_;
};