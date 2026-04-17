#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"
#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"

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
            S_ring_ = DMat<Scalar>::Zero(motion_subspace_dimension, num_velocities_);
        }

        template <typename Scalar>
        JointState<Scalar> Base<Scalar>::toSpanningTreeState(const JointState<Scalar> &joint_state)
        {
            JointState<Scalar> spanning_joint_state(true, true);
            std::shared_ptr<LoopConstraint::GenericImplicit<Scalar>> generic_implicit;
            if (loop_constraint_->isImplicit())
            {
                generic_implicit = std::dynamic_pointer_cast<LoopConstraint::GenericImplicit<Scalar>>(loop_constraint_);
            }

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
                spanning_joint_state.position = joint_state.position;
            }
            else
            {
                throw std::runtime_error("Unhandled case");
            }

            bool used_fused_implicit_generic = false;

            // Implicit Generic constraints can compute G and g directly from independent velocity.
            // This removes one CasADi boundary crossing in the common independent-velocity path.
            if (generic_implicit && !joint_state.velocity.isSpanning())
            {
                generic_implicit->updateGAndgFromIndependentVelocity(spanning_joint_state.position,
                                                                     joint_state.velocity);
                spanning_joint_state.velocity = generic_implicit->G() * joint_state.velocity;
                used_fused_implicit_generic = true;
            }
            else
            {
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

                // For implicit Generic constraints, only g is required here for kinematics updates.
                // Avoid evaluating k to reduce CasADi boundary cost on cold/random-state calls.
                if (loop_constraint_->isImplicit())
                {
                    if (generic_implicit)
                    {
                        generic_implicit->updateBiasGOnly(spanning_joint_state);
                    }
                    else
                    {
                        loop_constraint_->updateBiases(spanning_joint_state);
                    }
                }
                else
                {
                    loop_constraint_->updateBiases(spanning_joint_state);
                }
            }

            if (used_fused_implicit_generic)
            {
                // Biases were already updated by updateGAndgFromIndependentVelocity.
            }

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
        template class Base<std::complex<double>>;
        template class Base<float>;
        template class Base<casadi::SX>;

    }

} // namespace grbda
