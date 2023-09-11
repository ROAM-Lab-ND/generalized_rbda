#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        Base::Base(int num_bodies, int num_positions, int num_velocities,
                   bool position_is_spanning, bool velocity_is_spanning)
            : num_bodies_(num_bodies),
              num_positions_(num_positions),
              num_velocities_(num_velocities),
              position_is_spanning_(position_is_spanning),
              velocity_is_spanning_(velocity_is_spanning)
        {
            const size_t motion_subspace_dimension = num_bodies * 6;
            S_ = DMat<double>::Zero(motion_subspace_dimension, num_velocities_);
            Psi_ = DMat<double>::Zero(motion_subspace_dimension, num_velocities_);
            vJ_ = DVec<double>::Zero(motion_subspace_dimension);
            cJ_ = DVec<double>::Zero(motion_subspace_dimension);
        }

        JointState Base::toSpanningTreeState(const JointState &joint_state)
        {
            JointState spanning_joint_state(true, true);

            if (!joint_state.position.isSpanning())
            {
                spanning_joint_state.position = loop_constraint_->gamma(joint_state.position);
            }
            else
            {
                spanning_joint_state.position = joint_state.position;
            }
            loop_constraint_->updateJacobians(spanning_joint_state.position);

            if (!joint_state.velocity.isSpanning())
            {
                spanning_joint_state.velocity = G() * joint_state.velocity;
            }
            else
            {
                spanning_joint_state.velocity = joint_state.velocity;
            }
            loop_constraint_->updateBiases(spanning_joint_state);

            return spanning_joint_state;
        }

        JointCoordinate Base::integratePosition(JointState joint_state, double dt) const
        {
            if (joint_state.position.isSpanning() && joint_state.velocity.isSpanning())
            {
                joint_state.position += joint_state.velocity * dt;
            }
            else if (joint_state.position.isSpanning())
            {
                joint_state.position += G() * joint_state.velocity * dt;
            }
            else if (joint_state.velocity.isSpanning())
            {
                throw std::runtime_error("Velocity is spanning but position is not. This is not supported.");
            }
            else
            {
                joint_state.position += joint_state.velocity * dt;
            }

            return joint_state.position;
        }

        JointState Base::randomJointState() const
        {
            JointState joint_state(position_is_spanning_, velocity_is_spanning_);
            joint_state.position = DVec<double>::Random(numPositions());
            joint_state.velocity = DVec<double>::Random(numVelocities());
            return joint_state;
        }

    }

} // namespace grbda
