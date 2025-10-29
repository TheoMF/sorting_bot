#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/frames.hpp"

#ifndef __sorting_bot_inverse_kin_base__
#define __sorting_bot_inverse_kin_base__
class InverseKinBase
{
public:
    void initialize_model(std::shared_ptr<pinocchio::Model> model_ptr, std::shared_ptr<pinocchio::Data> data_ptr, int ee_frame_id)
    {
        model_ptr_ = model_ptr;
        data_ptr_ = data_ptr;
        ee_frame_id_ = ee_frame_id;
        nq_ = model_ptr_->nq;
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

protected:
    // Pinocchio model attributes
    std::shared_ptr<pinocchio::Model> model_ptr_;
    std::shared_ptr<pinocchio::Data> data_ptr_;
    int ee_frame_id_;
    int nq_;
};
#endif