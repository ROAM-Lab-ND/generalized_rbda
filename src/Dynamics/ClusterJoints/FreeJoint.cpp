#include "grbda/Dynamics/ClusterJoints/FreeJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar, typename OrientationRepresentation>
        Free<Scalar, OrientationRepresentation>::Free(const Body<Scalar> &body, std::string name)
            : Base<Scalar>(1, OrientationRepresentation::num_ori_parameter + 3, 6),
              body_(body)
        {
            q_cache_ = DVec<Scalar>::Zero(OrientationRepresentation::num_ori_parameter + 3);
            qd_cache_ = DVec<Scalar>::Zero(6);
            if (body.parent_index_ >= 0)
                throw std::runtime_error("Free joint is only valid as the first joint in a tree and thus cannot have a parent body");

            this->S_.setIdentity();
            this->Psi_.setIdentity();

            this->single_joints_.emplace_back(
                new Joints::Free<Scalar, OrientationRepresentation>(name));

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Identity(6, 6);

            this->loop_constraint_ =
                std::make_shared<LoopConstraint::Free<Scalar, OrientationRepresentation>>();
        }

        template <typename Scalar, typename OrientationRepresentation>
        void Free<Scalar, OrientationRepresentation>::updateKinematics(
            const JointState<Scalar> &joint_state)
        {
            // Cache state for derivative methods
            q_cache_ = joint_state.position;
            qd_cache_ = joint_state.velocity;

            this->single_joints_[0]->updateKinematics(joint_state.position, joint_state.velocity);
            this->vJ_ = this->S_ * joint_state.velocity;
        }

        template <typename Scalar, typename OrientationRepresentation>
        void
        Free<Scalar, OrientationRepresentation>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 1 || Xup.getNumParentBodies() != 1)
                throw std::runtime_error("[Free Joint] Xup must be 6x6");
#endif
            Xup[0] = this->single_joints_[0]->XJ();
        }

        template <typename Scalar, typename OrientationRepresentation>
        JointState<double> Free<Scalar, OrientationRepresentation>::randomJointState(bool enforce_position_constraint) const
        {
            const int num_ori_param = OrientationRepresentation::num_ori_parameter;

            JointState<double> joint_state(false, false);
            joint_state.position = DVec<double>::Zero(num_ori_param + 3);
            joint_state.position.template segment<3>(0) = Vec3<double>::Random(3);
            joint_state.position.template segment<num_ori_param>(3) =
                OrientationRepresentation::template randomOrientation<double>();
            joint_state.velocity = DVec<double>::Random(6);
            return joint_state;
        }

        template <typename Scalar, typename OrientationRepresentation>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        Free<Scalar, OrientationRepresentation>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> bodies_joints_and_ref_inertias;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(body_, this->single_joints_[0],
                                                                     Mat6<Scalar>::Zero()));
            return bodies_joints_and_ref_inertias;
        }

        // Derivative methods for Free joint

        template <typename Scalar, typename OrientationRepresentation>
        DMat<Scalar> Free<Scalar, OrientationRepresentation>::getSdotqd_q() const
        {
            return DMat<Scalar>::Zero(6, 6);
        }

        // Template specializations for complex<double> (used by complex-step differentiation)
        template <>
        DMat<std::complex<double>>
        Free<std::complex<double>, ori_representation::Quaternion>::getSdotqd_q() const
        {
            // CRITICAL FIX: Must return (6, nv) not (6, nq)
            // nv = 6 for free joint, nq = 7 for quaternion
            return DMat<std::complex<double>>::Zero(6, 6);
        }

        template class Free<double, ori_representation::RollPitchYaw>;
        template class Free<double, ori_representation::Quaternion>;
        template class Free<std::complex<double>, ori_representation::RollPitchYaw>;
        template class Free<std::complex<double>, ori_representation::Quaternion>;
        template class Free<float, ori_representation::RollPitchYaw>;
        template class Free<float, ori_representation::Quaternion>;
        template class Free<casadi::SX, ori_representation::RollPitchYaw>;
        template class Free<casadi::SX, ori_representation::Quaternion>;
    }

} // namespace grbda
