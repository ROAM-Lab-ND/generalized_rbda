// #include "GeneralizedJoint.h"
#include "GeneralizedJointTypes.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        template <typename Derived>
        Base<Derived>::Base(int num_bodies, int num_positions, int num_velocities,
                            bool position_is_spanning, bool velocity_is_spanning)
            : num_bodies_(num_bodies),
              num_positions_(num_positions),
              num_velocities_(num_velocities),
              position_is_spanning_(position_is_spanning),
              velocity_is_spanning_(velocity_is_spanning)
        {
            const size_t motion_subspace_dimension = num_bodies * 6;
            S_ = DMat<double>::Zero(motion_subspace_dimension, num_velocities_);
            S_ring_ = DMat<double>::Zero(motion_subspace_dimension, num_velocities_);
            Psi_ = DMat<double>::Zero(motion_subspace_dimension, num_velocities_);
            vJ_ = DVec<double>::Zero(motion_subspace_dimension);
        }

        template <typename Derived>
        JointState Base<Derived>::toSpanningTreeState(const JointState &joint_state)
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

        template <typename Derived>
        JointState Base<Derived>::randomJointState() const
        {
            JointState joint_state(position_is_spanning_, velocity_is_spanning_);
            joint_state.position = DVec<double>::Random(numPositions());
            joint_state.velocity = DVec<double>::Random(numVelocities());
            return joint_state;
        }

        template class Base<Free>;
        template class Base<Revolute>;
        template class Base<RevolutePair>;
        template class Base<RevolutePairWithRotor>;
        template class Base<RevoluteTripleWithRotor>;
        template class Base<RevoluteWithMultipleRotorsJoint>;
        template class Base<RevoluteWithRotor>;
        template class Base<TelloDifferential>;
        // template class Base<TelloKneeAnkleDifferential>;
    }

} // namespace grbda
