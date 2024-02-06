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
            if (!joint_state.position.isSpanning())
            {
                spanning_joint_state.position = loop_constraint_->gamma(joint_state.position);
            }
            else
            {
                spanning_joint_state.position = joint_state.position;
            }
            loop_constraint_->updateJacobians(spanning_joint_state.position);

            // Spanning velocities
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

        template class Base<double>;
        template class Base<float>;
        template class Base<casadi::SX>;

        template <typename Scalar>
        Explicit<Scalar>::Explicit(int num_bodies, int num_positions, int num_velocities)
            : Base<Scalar>(num_bodies, num_positions, num_velocities) {}

        template <typename Scalar>
        bool Explicit<Scalar>::isValidSpanningState(const JointState<Scalar> &joint_state) const
        {
            return true;
        }

        template <typename Scalar>
        JointState<Scalar> Explicit<Scalar>::randomJointState()
        {
            JointState<Scalar> joint_state(false, false);
            joint_state.position = DVec<Scalar>::Random(this->numPositions());
            joint_state.velocity = DVec<Scalar>::Random(this->numVelocities());
            return joint_state;
        }

        template class Explicit<double>;
        template class Explicit<float>;
        template class Explicit<casadi::SX>;

        template <typename Scalar>
        Implicit<Scalar>::Implicit(int num_bodies, int num_positions, int num_velocities)
            : Base<Scalar>(num_bodies, num_positions, num_velocities) {}

        template <typename Scalar>
        bool Implicit<Scalar>::isValidSpanningState(const JointState<Scalar> &joint_state) const
        {
            if (!this->loop_constraint_->isValidSpanningPosition(joint_state.position) ||
                !this->loop_constraint_->isValidSpanningVelocity(joint_state.velocity))
            {
                return false;
            }
            return true;
        }

        template <typename Scalar>
        JointState<Scalar> Implicit<Scalar>::randomJointState()
        {
            int iterations = 0;
            bool is_valid_spanning_state = false;
            JointState<Scalar> spanning_joint_state(true, true);
            while (!is_valid_spanning_state)
            {
                if (iterations == 1000)
                {
                    throw std::runtime_error("Could not generate a valid spanning state");
                }

                JointState<Scalar> joint_state(false, false);
                joint_state.position = DVec<Scalar>::Random(this->numPositions());
                joint_state.velocity = DVec<Scalar>::Random(this->numVelocities());
                spanning_joint_state = this->toSpanningTreeState(joint_state);
                is_valid_spanning_state = isValidSpanningState(spanning_joint_state);
                iterations++;
            }
            return spanning_joint_state;
        }

        template class Implicit<double>;
        template class Implicit<float>;
        template class Implicit<casadi::SX>;
    }

} // namespace grbda
