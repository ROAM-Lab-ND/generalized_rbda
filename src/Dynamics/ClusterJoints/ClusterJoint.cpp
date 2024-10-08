#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar>
        Base<Scalar>::Base(int num_bodies, int num_positions, int num_velocities)
            : num_bodies_(num_bodies),
              num_positions_(num_positions),
              num_velocities_(num_velocities)
        {
            const size_t motion_subspace_dimension = num_bodies * 6;
            S_ = DMat<Scalar>::Zero(motion_subspace_dimension, num_velocities_);
            Psi_ = DMat<Scalar>::Zero(motion_subspace_dimension, num_velocities_);
            vJ_ = DVec<Scalar>::Zero(motion_subspace_dimension);
            cJ_ = DVec<Scalar>::Zero(motion_subspace_dimension);
        }

        template <typename Scalar>
        JointState<Scalar> Base<Scalar>::toSpanningTreeState(const JointState<Scalar> &joint_state)
        {
            JointState<Scalar> spanning_joint_state(true, true);

            // Spanning positions
            if (!joint_state.position.isSpanning() && loop_constraint_->isExplicit())
            {
                spanning_joint_state.position = loop_constraint_->gamma(joint_state.position);
            }
            else if (!joint_state.position.isSpanning() && loop_constraint_->isImplicit())
            {
                throw std::runtime_error("Independent positions cannot be converted to spanning positions when the constraint is implicit.");
            }
            else if (joint_state.position.isSpanning() && loop_constraint_->isExplicit())
            {
                // TODO(@MatthewChignoli): We should check to make sure that the spanning position is valid. Will require turning gamma into phi
                spanning_joint_state.position = joint_state.position;
            }
            else if (joint_state.position.isSpanning() && loop_constraint_->isImplicit())
            {
                if (!loop_constraint_->isValidSpanningPosition(joint_state.position))
                {
                    throw std::runtime_error("Spanning position is not valid");
                }
                spanning_joint_state.position = joint_state.position;
            }
            else
            {
                throw std::runtime_error("Unhandled case");
            }

            // Spanning velocities
            loop_constraint_->updateJacobians(spanning_joint_state.position);
            if (!joint_state.velocity.isSpanning())
            {
                spanning_joint_state.velocity = G() * joint_state.velocity;
            }
            else
            {
                if (!loop_constraint_->isValidSpanningVelocity(joint_state.velocity))
                {
                    throw std::runtime_error("Spanning velocity is not valid");
                }
                spanning_joint_state.velocity = joint_state.velocity;
            }
            loop_constraint_->updateBiases(spanning_joint_state);

            return spanning_joint_state;
        }

        template <typename Scalar>
        JointState<double> Base<Scalar>::randomJointState() const
        {
            JointState<double> joint_state(false, false);
            joint_state.position = DVec<double>::Random(numPositions());
            joint_state.velocity = DVec<double>::Random(numVelocities());
            return joint_state;
        }

        template class Base<double>;
        template class Base<float>;
        template class Base<casadi::SX>;

    }

} // namespace grbda
