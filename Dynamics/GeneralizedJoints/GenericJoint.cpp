#include "GenericJoint.h"
#include "Utils/Utilities/utilities.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        Generic::Generic(const std::vector<Body> &bodies, const std::vector<JointPtr> &joints,
                         const ExplicitConstraint &explicit_constraint)
            : Base(explicit_constraint.numIndependentVelocities(),
                   explicit_constraint.numIndependentVelocities(),
                   (int)bodies.size()),
              bodies_(bodies)
        {
            for (auto &joint : joints)
                single_joints_.push_back(joint);

            gamma_ = explicit_constraint.gamma_;
            G_ = explicit_constraint.G_;
            g_ = explicit_constraint.g_;
            extractImplicitConstraintFromExplicit(explicit_constraint);

            extractConnectivity();

            S_spanning_tree_ = DMat<double>::Zero(0, 0);
            for (auto &joint : joints)
                S_spanning_tree_ = appendEigenMatrix(S_spanning_tree_, joint->S());

            Xup_spanning_tree_ = DMat<double>::Identity(6 * num_bodies_, 6 * num_bodies_);
            Xup_ring_spanning_tree_ = DMat<double>::Zero(6 * num_bodies_, 6 * num_bodies_);
        }

        void Generic::updateKinematics(const JointState &joint_state)
        {
            if (joint_state.position.size() != num_independent_positions_ ||
                joint_state.velocity.size() != num_independent_velocities_)
            {
                throw std::runtime_error("[Generic] Dimension of y or yd is wrong");
            }

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

                        Xup_spanning_tree_.block<6, 6>(6 * i, 6 * j) =
                            Xup_spanning_tree_.block<6, 6>(6 * i, 6 * k) *
                            (joint_k->XJ() * body_k.Xtree_).toMatrix();

                        k = j;
                    }
                }

                pos_idx += num_pos;
                vel_idx += num_vel;
            }

            // TODO(@MatthewChignoli): Bad because this assumes independent coords
            S_ = Xup_spanning_tree_ * S_spanning_tree_ * G_;
            vJ_ = S_ * joint_state.velocity;

            for (int i = 0; i < num_bodies_; i++)
            {
                SVec<double> v_relative = SVec<double>::Zero();
                for (int j = i - 1; j >= 0; j--)
                {
                    if (connectivity_(i, j))
                    {
                        v_relative = Xup_spanning_tree_.block<6, 6>(6 * i, 6 * j) * vJ_.segment<6>(6 * j) -
                                     vJ_.segment<6>(6 * i);

                        Xup_ring_spanning_tree_.block<6, 6>(6 * i, 6 * j) =
                            motionCrossMatrix(v_relative) * Xup_spanning_tree_.block<6, 6>(6 * i, 6 * j);
                    }
                }
            }

            S_ring_ = Xup_ring_spanning_tree_ * S_spanning_tree_ * G_;
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

        void
        Generic::extractImplicitConstraintFromExplicit(const ExplicitConstraint &explicit_constraint)
        {
            const DMat<double> &G = explicit_constraint.G_;
            const DVec<double> &g = explicit_constraint.g_;

            const int num_spanning_coords = G.rows();
            const int num_independent_coords = G.cols();
            const int num_constraints = num_spanning_coords - num_independent_coords;

            if (num_spanning_coords < num_independent_coords)
                throw std::runtime_error("Generic Joint cannot have more independent coordinates than spanning tree coordinates");
            if (!G.topRows(num_independent_coords).isIdentity())
                throw std::runtime_error("Generic joints require the independent coordinates be a subset of the spanning tree coordinates");

            K_ = DMat<double>::Zero(num_constraints, num_spanning_coords);
            K_.leftCols(num_independent_coords) = -G.bottomRows(num_constraints);
            K_.rightCols(num_constraints).setIdentity();
            k_ = -g.tail(num_constraints);

            spanning_tree_to_independent_coords_conversion_ =
                DMat<double>::Zero(num_independent_coords, num_spanning_coords);
            spanning_tree_to_independent_coords_conversion_.leftCols(num_independent_coords) = DMat<double>::Identity(num_independent_coords, num_independent_coords);
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

        // TODO(@MatthewChignoli): This assumes the generic joint can be described with independent coords
        JointState Generic::randomJointState() const
        {
            JointState joint_state(false, false);
            joint_state.position = DVec<double>::Random(numPositions());
            joint_state.velocity = DVec<double>::Random(numVelocities());
            return joint_state;
        }
    }

} // namespace grbda
