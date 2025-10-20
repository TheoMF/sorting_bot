#include "sorting_bot/inverse_kin_base.hpp"

class InverseKin : public InverseKinBase{

public:
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
private:
    // Inverse kinematics parameters
    const double eps = 1e-3;
    const int IT_MAX = 20000;
    const double DT = 1e-1;
    const double damp = 1e-6;
};