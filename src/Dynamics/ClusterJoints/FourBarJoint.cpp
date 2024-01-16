#include "grbda/Dynamics/ClusterJoints/FourBarJoint.h"

namespace grbda
{
    namespace LoopConstraint
    {
        template <typename Scalar>
        FourBar<Scalar>::FourBar(std::vector<Scalar> path1_lengths,
                                 std::vector<Scalar> path2_lengths,
                                 Vec2<Scalar> offset)
            : path1_size_(path1_lengths.size()), path2_size_(path2_lengths.size()),
              path1_lengths_(path1_lengths), path2_lengths_(path2_lengths)
        {
            if (path1_size_ + path2_size_ != 3)
            {
                throw std::runtime_error("FourBar: Must contain 3 links");
            }

            this->phi_ = [this, offset](const JointCoordinate<Scalar> &joint_pos)
            {
                DVec<Scalar> phi = DVec<Scalar>::Zero(2);

                DVec<Scalar> path1_joints = joint_pos.head(path1_size_);
                DVec<Scalar> path2_joints = joint_pos.tail(path2_size_);

                Scalar cumulative_angle = 0.;
                DVec<Scalar> path1 = DVec<Scalar>::Zero(2);
                for (size_t i = 0; i < path1_size_; i++)
                {
                    cumulative_angle += path1_joints(i);
                    path1(0) += path1_lengths_[i] * cos(cumulative_angle);
                    path1(1) += path1_lengths_[i] * sin(cumulative_angle);
                }

                cumulative_angle = 0.;
                DVec<Scalar> path2 = offset;
                for (size_t i = 0; i < path2_size_; i++)
                {
                    cumulative_angle += path2_joints(i);
                    path2(0) += path2_lengths_[i] * cos(cumulative_angle);
                    path2(1) += path2_lengths_[i] * sin(cumulative_angle);
                }

                phi = path1 - path2;
                return phi;
            };

            this->G_ = DMat<Scalar>::Zero(3, 1);
            this->g_ = DVec<Scalar>::Zero(3);

            this->K_ = DMat<Scalar>::Zero(2, 3);
            this->k_ = DVec<Scalar>::Zero(2);
        }

        template <typename Scalar>
        void FourBar<Scalar>::updateJacobians(const JointCoordinate<Scalar> &joint_pos)
        {
            updateImplicitJacobian(joint_pos);
            updateExplicitJacobian(this->K_);
        }

        template <typename Scalar>
        void FourBar<Scalar>::updateImplicitJacobian(const JointCoordinate<Scalar> &joint_pos)
        {
            DVec<Scalar> q1 = joint_pos.head(path1_size_);
            DVec<Scalar> q2 = joint_pos.tail(path2_size_);

            // Update K
            Scalar cumulative_angle = 0.;
            DMat<Scalar> K1 = DMat<Scalar>::Zero(2, path1_size_);
            for (size_t i = 0; i < path1_lengths_.size(); i++)
            {
                cumulative_angle += q1(i);
                for (size_t j = 0; j <= i; j++)
                {
                    K1(0, j) += -path1_lengths_[i] * sin(cumulative_angle);
                    K1(1, j) += path1_lengths_[i] * cos(cumulative_angle);
                }
            }

            cumulative_angle = 0.;
            DMat<Scalar> K2 = DMat<Scalar>::Zero(2, path2_size_);
            for (size_t i = 0; i < path2_size_; i++)
            {
                cumulative_angle += q2(i);
                for (size_t j = 0; j <= i; j++)
                {
                    K2(0, j) += path2_lengths_[i] * sin(cumulative_angle);
                    K2(1, j) += -path2_lengths_[i] * cos(cumulative_angle);
                }
            }

            this->K_ << K1, K2;
        }

        template <typename Scalar>
        void FourBar<Scalar>::updateExplicitJacobian(const DMat<Scalar> &K)
        {
            // Update G via the Algorithm in Ch 8.8 of Featherstone's book
            // TODO(@MatthewChignoli): Assumes K is rank 2, do we need to check that?
            // TODO(@MatthewChignoli): Featherstone notes that if the user specifies which coordinates are independent, it eliminates the need for full pivoting
            FullPivLuType lu_of_K(K);
            DMat<Scalar> matrixLU = lu_of_K.matrixLU();
            DMat<Scalar> L = DMat<Scalar>::Identity(3, 3);
            L.template block<2, 3>(0, 0).template triangularView<Eigen::StrictlyLower>() = matrixLU;
            DMat<Scalar> U = matrixLU.template triangularView<Eigen::Upper>();
            DMat<Scalar> LU = L * U;

            K11inv_ = InverseType(LU.template topLeftCorner<2, 2>());
            DMat<Scalar> K12 = LU.template topRightCorner<2, 1>();
            Q2T_ = InverseType(lu_of_K.permutationQ().transpose());
            Q1inv_ = lu_of_K.permutationP();

            DMat<Scalar> tmp(3, 1);
            tmp.template topRows<2>() = -K11inv_.solve(K12);
            tmp.template bottomRows<1>() = DVec<Scalar>::Ones(1);
            this->G_ = Q2T_.solve(tmp);
        }

        template <typename Scalar>
        void FourBar<Scalar>::updateBiases(const JointState<Scalar> &joint_state)
        {
            const JointCoordinate<Scalar> &joint_pos = joint_state.position;
            const JointCoordinate<Scalar> &joint_vel = joint_state.velocity;

            DVec<Scalar> q1 = joint_pos.head(path1_size_);
            DVec<Scalar> q2 = joint_pos.tail(path2_size_);

            DVec<Scalar> qd1 = joint_vel.head(path1_size_);
            DVec<Scalar> qd2 = joint_vel.tail(path2_size_);

            // Update k
            Scalar cumulative_angle = 0.;
            Scalar cumulative_velocity = 0.;
            DMat<Scalar> Kd1 = DMat<Scalar>::Zero(2, path1_size_);
            for (size_t i = 0; i < path1_lengths_.size(); i++)
            {
                cumulative_angle += q1(i);
                cumulative_velocity += qd1(i);
                for (size_t j = 0; j <= i; j++)
                {
                    Kd1(0, j) += -path1_lengths_[i] * cumulative_velocity * cos(cumulative_angle);
                    Kd1(1, j) += -path1_lengths_[i] * cumulative_velocity * sin(cumulative_angle);
                }
            }

            cumulative_angle = 0.;
            cumulative_velocity = 0.;
            DMat<Scalar> Kd2 = DMat<Scalar>::Zero(2, path2_size_);
            for (size_t i = 0; i < path2_size_; i++)
            {
                cumulative_angle += q2(i);
                cumulative_velocity += qd2(i);
                for (size_t j = 0; j <= i; j++)
                {
                    Kd2(0, j) += path2_lengths_[i] * cumulative_velocity * cos(cumulative_angle);
                    Kd2(1, j) += path2_lengths_[i] * cumulative_velocity * sin(cumulative_angle);
                }
            }

            DMat<Scalar> Kdot(2, 3);
            Kdot << Kd1, Kd2;
            this->k_ = -Kdot * joint_vel;

            // Update g
            DVec<Scalar> k_prime = Q1inv_ * this->k_;
            DVec<Scalar> tmp = DVec<Scalar>::Zero(3);
            tmp.template topRows<2>() = K11inv_.solve(k_prime.template topRows<2>());
            this->g_ = Q2T_.solve(tmp);
        }

        template struct FourBar<double>;
        template struct FourBar<casadi::SX>;
    }
}
