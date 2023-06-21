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

        JointState Base::randomJointState(bool spanning_coordinates) const
        {
            if (spanning_coordinates)
            {
                JointState joint_state(true, true);
                joint_state.position = gamma_(DVec<double>::Random(numIndependentPositions()));
                joint_state.velocity = G_ * DVec<double>::Random(numIndependentVelocities());
                return joint_state;
            }
            else
            {
                JointState joint_state(false, false);
                joint_state.position = DVec<double>::Random(numIndependentPositions());
                joint_state.velocity = DVec<double>::Random(numIndependentVelocities());
                return joint_state;
            }
        }
    }

} // namespace grbda
