#include "GenericJoint.h"
#include "Utils/Utilities.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        Generic::Generic(const std::vector<Body> &bodies, const std::vector<JointPtr> &joints,
                         shared_ptr<LoopConstraint::Base> loop_constraint)
            : Base((int)bodies.size(),
                   loop_constraint->numIndependentPos(), loop_constraint->numIndependentVel(),
                   false, false),
              bodies_(bodies)
        {
            loop_constraint_ = loop_constraint;

            for (auto &joint : joints)
                single_joints_.push_back(joint);

            extractConnectivity();

            S_spanning_tree_ = DMat<double>::Zero(0, 0);
            for (auto &joint : joints)
                S_spanning_tree_ = appendEigenMatrix(S_spanning_tree_, joint->S());

            Xup_spanning_tree_ = DMat<double>::Identity(6 * num_bodies_, 6 * num_bodies_);
            Xup_ring_spanning_tree_ = DMat<double>::Zero(6 * num_bodies_, 6 * num_bodies_);
        }

        void Generic::updateKinematics(const JointState &joint_state)
        {
#ifdef DEBUG_MODE
            jointStateCheck(joint_state);
#endif

            const JointState spanning_joint_state = toSpanningTreeState(joint_state);
            const DVec<double> &q = spanning_joint_state.position;
            const DVec<double> &qd = spanning_joint_state.velocity;

            int pos_idx = 0;
            int vel_idx = 0;
            for (int i = 0; i < num_bodies_; i++)
            {
                const auto &body = bodies_[i];
                auto joint = single_joints_[i];

                const int num_pos = joint->numPositions();
                const int num_vel = joint->numVelocities();

                joint->updateKinematics(q.segment(pos_idx, num_pos), qd.segment(vel_idx, num_vel));

                int k = i;
                for (int j = i - 1; j >= 0; j--)
                {
                    if (connectivity_(i, j))
                    {
                        const auto &body_k = bodies_[k];
                        const auto joint_k = single_joints_[k];

                        const Mat6<double> Xup_prev = Xup_spanning_tree_.block<6, 6>(6 * i, 6 * k);
                        const Mat6<double> Xint = (joint_k->XJ() * body_k.Xtree_).toMatrix();
                        Xup_spanning_tree_.block<6, 6>(6 * i, 6 * j) = Xup_prev * Xint;

                        k = j;
                    }
                }

                pos_idx += num_pos;
                vel_idx += num_vel;
            }

            const DMat<double> S_implicit = Xup_spanning_tree_ * S_spanning_tree_;
            S_ = S_implicit * loop_constraint_->G();
            vJ_ = S_implicit * qd;

            for (int i = 0; i < num_bodies_; i++)
            {
                SVec<double> v_relative = SVec<double>::Zero();
                for (int j = i - 1; j >= 0; j--)
                {
                    if (connectivity_(i, j))
                    {
                        const Mat6<double> Xup = Xup_spanning_tree_.block<6, 6>(6 * i, 6 * j);

                        const SVec<double> v_parent = Xup * vJ_.segment<6>(6 * j);
                        const SVec<double> v_child = vJ_.segment<6>(6 * i);
                        v_relative = v_child - v_parent;

                        Xup_ring_spanning_tree_.block<6, 6>(6 * i, 6 * j) =
                            -motionCrossMatrix(v_relative) * Xup;
                    }
                }
            }

            cJ_ = Xup_ring_spanning_tree_ * S_spanning_tree_ * qd +
                  S_implicit * loop_constraint_->g();
        }

        void Generic::computeSpatialTransformFromParentToCurrentCluster(
            GeneralizedSpatialTransform &Xup) const
        {
            for (int i = 0; i < num_bodies_; i++)
            {
                const auto &body = bodies_[i];
                const auto joint = single_joints_[i];
                Xup[i] = joint->XJ() * body.Xtree_;
                for (int j = i - 1; j >= 0; j--)
                    if (connectivity_(i, j))
                    {
                        Xup[i] = Xup[i] * Xup[j];
                        break;
                    }
            }
        }

        void Generic::extractConnectivity()
        {
            connectivity_ = DMat<bool>::Zero(num_bodies_, num_bodies_);
            for (int i = 0; i < num_bodies_; i++)
            {
                int j = i;
                while (bodyInCurrentCluster(bodies_[j].parent_index_))
                {
                    const Body &parent_body = getBody(bodies_[j].parent_index_);
                    j = parent_body.sub_index_within_cluster_;
                    connectivity_(i, j) = true;
                }
            }
        }

        bool Generic::bodyInCurrentCluster(const int body_index) const
        {
            for (const auto &body : bodies_)
                if (body.index_ == body_index)
                    return true;
            return false;
        }

        const Body &Generic::getBody(const int body_index) const
        {
            for (const auto &body : bodies_)
                if (body.index_ == body_index)
                    return body;
            throw std::runtime_error("Body is not in the current cluster");
        }
    }

} // namespace grbda
