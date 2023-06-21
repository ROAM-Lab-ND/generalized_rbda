#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        Base::Base(int num_independent_positions, int num_independent_velocities, int num_bodies)
            : num_bodies_(num_bodies), num_independent_positions_(num_independent_positions),
              num_independent_velocities_(num_independent_velocities)
        {
            const size_t motion_subspace_dimension = num_bodies * 6;
            S_ = DMat<double>::Zero(motion_subspace_dimension, num_independent_velocities);
            S_ring_ = DMat<double>::Zero(motion_subspace_dimension, num_independent_velocities);
            Psi_ = DMat<double>::Zero(motion_subspace_dimension, num_independent_velocities);
            vJ_ = DVec<double>::Zero(motion_subspace_dimension);
        }

        JointState Base::toSpanningTreeState(const JointState &joint_state)
        {
            JointState spanning_joint_state(true, true);

            if (!joint_state.position.isSpanning())
            {
                spanning_joint_state.position = gamma_(joint_state.position);
            }
            else
            {
                spanning_joint_state.position = joint_state.position;
            }
            updateConstraintJacobians(spanning_joint_state.position);

            if (!joint_state.velocity.isSpanning())
            {
                spanning_joint_state.velocity = G_ * joint_state.velocity;
            }
            else
            {
                spanning_joint_state.velocity = joint_state.velocity;
            }
            updateConstraintBias(spanning_joint_state);

            return spanning_joint_state;
        }

        JointState Base::randomJointState() const
        {
            JointState joint_state(false, false);
            joint_state.position = DVec<double>::Random(numIndependentPositions());
            joint_state.velocity = DVec<double>::Random(numIndependentVelocities());
            return joint_state;
        }

    }

} // namespace grbda
