#include "grbda/Dynamics/ClusterJoints/FourBarJoint.h"

namespace grbda
{
    namespace LoopConstraint
    {
        template <typename Scalar>
        FourBar<Scalar>::FourBar(std::vector<Scalar> path1_link_lengths,
                                 std::vector<Scalar> path2_link_lengths,
                                 Vec2<Scalar> offset, int independent_coordinate)
            : links_in_path1_(path1_link_lengths.size()),
              links_in_path2_(path2_link_lengths.size()),
              path1_link_lengths_(path1_link_lengths),
              path2_link_lengths_(path2_link_lengths),
              independent_coordinate_(independent_coordinate)
        {
            if (links_in_path1_ + links_in_path2_ != 3)
            {
                throw std::runtime_error("FourBar: Must contain 3 links");
            }

            this->phi_ = [this, offset](const JointCoordinate<Scalar> &joint_pos)
            {
                DVec<Scalar> phi = DVec<Scalar>::Zero(2);

                DVec<Scalar> path1_joints = joint_pos.head(links_in_path1_);
                DVec<Scalar> path2_joints = joint_pos.tail(links_in_path2_);

                Scalar cumulative_angle = 0.;
                DVec<Scalar> path1 = DVec<Scalar>::Zero(2);
                for (size_t i = 0; i < links_in_path1_; i++)
                {
                    cumulative_angle += path1_joints(i);
                    path1(0) += path1_link_lengths_[i] * cos(cumulative_angle);
                    path1(1) += path1_link_lengths_[i] * sin(cumulative_angle);
                }

                cumulative_angle = 0.;
                DVec<Scalar> path2 = offset;
                for (size_t i = 0; i < links_in_path2_; i++)
                {
                    cumulative_angle += path2_joints(i);
                    path2(0) += path2_link_lengths_[i] * cos(cumulative_angle);
                    path2(1) += path2_link_lengths_[i] * sin(cumulative_angle);
                }

                phi = path1 - path2;
                return phi;
            };

            this->G_ = DMat<Scalar>::Zero(3, 1);
            this->g_ = DVec<Scalar>::Zero(3);

            this->K_ = DMat<Scalar>::Zero(2, 3);
            this->k_ = DVec<Scalar>::Zero(2);

            switch (independent_coordinate_)
            {
            case 0:
                indepenent_coordinate_map_ << 1., 0., 0.,
                    0., 1., 0.,
                    0., 0., 1.;
                break;
            case 1:
                indepenent_coordinate_map_ << 0., 1., 0.,
                    1., 0., 0.,
                    0., 0., 1.;
                break;
            case 2:
                indepenent_coordinate_map_ << 0., 1., 0.,
                    0., 0., 1.,
                    1., 0., 0.;
                break;
            default:
                throw std::runtime_error("FourBar: Invalid independent coordinate");
            }
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
            DVec<Scalar> q1 = joint_pos.head(links_in_path1_);
            DVec<Scalar> q2 = joint_pos.tail(links_in_path2_);

            Scalar cumulative_angle = 0.;
            DMat<Scalar> K1 = DMat<Scalar>::Zero(2, links_in_path1_);
            for (size_t i = 0; i < path1_link_lengths_.size(); i++)
            {
                cumulative_angle += q1(i);
                for (size_t j = 0; j <= i; j++)
                {
                    K1(0, j) += -path1_link_lengths_[i] * sin(cumulative_angle);
                    K1(1, j) += path1_link_lengths_[i] * cos(cumulative_angle);
                }
            }

            cumulative_angle = 0.;
            DMat<Scalar> K2 = DMat<Scalar>::Zero(2, links_in_path2_);
            for (size_t i = 0; i < links_in_path2_; i++)
            {
                cumulative_angle += q2(i);
                for (size_t j = 0; j <= i; j++)
                {
                    K2(0, j) += path2_link_lengths_[i] * sin(cumulative_angle);
                    K2(1, j) += -path2_link_lengths_[i] * cos(cumulative_angle);
                }
            }

            this->K_ << K1, K2;
        }

        template <typename Scalar>
        void FourBar<Scalar>::updateExplicitJacobian(const DMat<Scalar> &K)
        {
            DVec<Scalar> Ki(2, 1);
            DMat<Scalar> Kd(2, 2);
            int dep_counter = 0;
            for (int i = 0; i < 3; i++)
            {
                if (i == independent_coordinate_)
                {
                    Ki = K.col(i);
                }
                else
                {
                    Kd.col(dep_counter) = K.col(i);
                    dep_counter++;
                }
            }

            Kd_inv_ = InverseType(Kd);
            this->G_(0, 0) = 1.;
            this->G_.template bottomRows<2>() = -Kd_inv_.solve(Ki);
            this->G_ = indepenent_coordinate_map_ * this->G_;
        }

        template <typename Scalar>
        void FourBar<Scalar>::updateBiases(const JointState<Scalar> &joint_state)
        {
            const JointCoordinate<Scalar> &joint_pos = joint_state.position;
            const JointCoordinate<Scalar> &joint_vel = joint_state.velocity;

            DVec<Scalar> q1 = joint_pos.head(links_in_path1_);
            DVec<Scalar> q2 = joint_pos.tail(links_in_path2_);

            DVec<Scalar> qd1 = joint_vel.head(links_in_path1_);
            DVec<Scalar> qd2 = joint_vel.tail(links_in_path2_);

            // Update k
            Scalar cumulative_angle = 0.;
            Scalar cumulative_velocity = 0.;
            DMat<Scalar> Kd1 = DMat<Scalar>::Zero(2, links_in_path1_);
            for (size_t i = 0; i < path1_link_lengths_.size(); i++)
            {
                cumulative_angle += q1(i);
                cumulative_velocity += qd1(i);
                for (size_t j = 0; j <= i; j++)
                {
                    Kd1(0, j) += -path1_link_lengths_[i] *
                                 cumulative_velocity * cos(cumulative_angle);
                    Kd1(1, j) += -path1_link_lengths_[i] *
                                 cumulative_velocity * sin(cumulative_angle);
                }
            }

            cumulative_angle = 0.;
            cumulative_velocity = 0.;
            DMat<Scalar> Kd2 = DMat<Scalar>::Zero(2, links_in_path2_);
            for (size_t i = 0; i < links_in_path2_; i++)
            {
                cumulative_angle += q2(i);
                cumulative_velocity += qd2(i);
                for (size_t j = 0; j <= i; j++)
                {
                    Kd2(0, j) += path2_link_lengths_[i] *
                                 cumulative_velocity * cos(cumulative_angle);
                    Kd2(1, j) += path2_link_lengths_[i] *
                                 cumulative_velocity * sin(cumulative_angle);
                }
            }

            DMat<Scalar> Kdot(2, 3);
            Kdot << Kd1, Kd2;
            this->k_ = -Kdot * joint_vel;

            // Update g
            this->g_(0) = 0.;
            this->g_.template bottomRows<2>() = Kd_inv_.solve(this->k_);
            this->g_ = indepenent_coordinate_map_ * this->g_;
        }

        template struct FourBar<double>;
        template struct FourBar<casadi::SX>;
    }
}
