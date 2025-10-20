#include "pinocchio/algorithm/model.hpp"
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

class GeneticAlgoInvKin{
public:

    void initialize(std::shared_ptr<pinocchio::Model> model_ptr, std::shared_ptr<pinocchio::Data> data_ptr,int ee_frame_id)
    {
        model_ptr_ = model_ptr;
        data_ptr_ = data_ptr;
        ee_frame_id_ = ee_frame_id;
        nq_ = model_ptr_->nq;
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
private:
    std::vector<Individual> population_, new_population_;

    //Pinocchio model attributes
    std::shared_ptr<pinocchio::Model> model_ptr_;
    std::shared_ptr<pinocchio::Data> data_ptr_;
    int ee_frame_id_;
    int nq_;
    
    // genetic algorithm parameters
    int population_size_ = 3000;
    int max_iter_ = 30;
    int nb_keep_ind = 400;
    double eps_ = 1e-3;
};