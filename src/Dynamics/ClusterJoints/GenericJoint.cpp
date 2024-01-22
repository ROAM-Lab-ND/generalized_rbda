#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"
#include "grbda/Utils/Utilities.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar>
        Generic<Scalar>::Generic(const std::vector<Body<Scalar>> &bodies,
                                 const std::vector<JointPtr<Scalar>> &joints,
                                 std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint)
            : Base<Scalar>((int)bodies.size(),
                           loop_constraint->numIndependentPos(),
                           loop_constraint->numIndependentVel()),
              bodies_(bodies)
        {
            this->loop_constraint_ = loop_constraint;

            for (auto &joint : joints)
                this->single_joints_.push_back(joint);

            extractConnectivity();

            S_spanning_ = DMat<Scalar>::Zero(0, 0);
            for (auto &joint : joints)
                S_spanning_ = appendEigenMatrix(S_spanning_, joint->S());

            X_intra_ = DMat<Scalar>::Identity(6 * this->num_bodies_, 6 * this->num_bodies_);
            X_intra_ring_ = DMat<Scalar>::Zero(6 * this->num_bodies_, 6 * this->num_bodies_);
        }

        template <typename Scalar>
        void Generic<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<Scalar> &q = spanning_joint_state.position;
            const DVec<Scalar> &qd = spanning_joint_state.velocity;

            int pos_idx = 0;
            int vel_idx = 0;
            for (int i = 0; i < this->num_bodies_; i++)
            {
                const auto &body = bodies_[i];
                auto joint = this->single_joints_[i];

                const int num_pos = joint->numPositions();
                const int num_vel = joint->numVelocities();

                joint->updateKinematics(q.segment(pos_idx, num_pos), qd.segment(vel_idx, num_vel));

                int k = i;
                for (int j = i - 1; j >= 0; j--)
                {
                    if (connectivity_(i, j))
                    {
                        const auto &body_k = bodies_[k];
                        const auto joint_k = this->single_joints_[k];

                        const Mat6<Scalar> Xup_prev = X_intra_.template block<6, 6>(6 * i, 6 * k);
                        const Mat6<Scalar> Xint = (joint_k->XJ() * body_k.Xtree_).toMatrix();
                        X_intra_.template block<6, 6>(6 * i, 6 * j) = Xup_prev * Xint;

                        k = j;
                    }
                }

                pos_idx += num_pos;
                vel_idx += num_vel;
            }

            const DMat<Scalar> S_implicit = X_intra_ * S_spanning_;
            this->S_ = S_implicit * this->loop_constraint_->G();
            this->vJ_ = S_implicit * qd;

            for (int i = 0; i < this->num_bodies_; i++)
            {
                SVec<Scalar> v_relative = SVec<Scalar>::Zero();
                for (int j = i - 1; j >= 0; j--)
                {
                    if (connectivity_(i, j))
                    {
                        const Mat6<Scalar> Xup = X_intra_.template block<6, 6>(6 * i, 6 * j);

                        const SVec<Scalar> v_parent = Xup * this->vJ_.template segment<6>(6 * j);
                        const SVec<Scalar> v_child = this->vJ_.template segment<6>(6 * i);
                        v_relative = v_child - v_parent;

                        X_intra_ring_.template block<6, 6>(6 * i, 6 * j) =
                            -spatial::motionCrossMatrix(v_relative) * Xup;
                    }
                }
            }

            this->cJ_ = X_intra_ring_ * this->S_spanning_ * qd +
                        S_implicit * this->loop_constraint_->g();
        }

        template <typename Scalar>
        void Generic<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
            for (int i = 0; i < this->num_bodies_; i++)
            {
                const auto &body = bodies_[i];
                const auto joint = this->single_joints_[i];
                Xup[i] = joint->XJ() * body.Xtree_;
                for (int j = i - 1; j >= 0; j--)
                    if (connectivity_(i, j))
                    {
                        Xup[i] = Xup[i] * Xup[j];
                        break;
                    }
            }
        }

        template <typename Scalar>
        void Generic<Scalar>::extractConnectivity()
        {
            connectivity_ = DMat<bool>::Zero(this->num_bodies_, this->num_bodies_);
            for (int i = 0; i < this->num_bodies_; i++)
            {
                int j = i;
                while (bodyInCurrentCluster(bodies_[j].parent_index_))
                {
                    const Body<Scalar> &parent_body = getBody(bodies_[j].parent_index_);
                    j = parent_body.sub_index_within_cluster_;
                    connectivity_(i, j) = true;
                }
            }
        }

        template <typename Scalar>
        bool Generic<Scalar>::bodyInCurrentCluster(const int body_index) const
        {
            for (const auto &body : bodies_)
                if (body.index_ == body_index)
                    return true;
            return false;
        }

        template <typename Scalar>
        const Body<Scalar> &Generic<Scalar>::getBody(const int body_index) const
        {
            for (const auto &body : bodies_)
                if (body.index_ == body_index)
                    return body;
            throw std::runtime_error("Body is not in the current cluster");
        }

        template class Generic<double>;
        template class Generic<float>;
        template class Generic<casadi::SX>;
    }

} // namespace grbda
