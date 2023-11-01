#include "FreeJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar, typename OrientationRepresentation>
        Free<Scalar, OrientationRepresentation>::Free(const Body<Scalar> &body)
            : Base<Scalar>(1, OrientationRepresentation::num_ori_parameter + 3, 6), body_(body)
        {
            if (body.parent_index_ >= 0)
                throw std::runtime_error("Free joint is only valid as the first joint in a tree and thus cannot have a parent body");

            this->S_.setIdentity();
            this->Psi_.setIdentity();

            this->single_joints_.emplace_back(new Joints::Free<Scalar, OrientationRepresentation>());

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Identity(6, 6);

            this->loop_constraint_ =
                std::make_shared<LoopConstraint::Free<Scalar, OrientationRepresentation>>();
        }

        template <typename Scalar, typename OrientationRepresentation>
        void Free<Scalar, OrientationRepresentation>::updateKinematics(
            const JointState<Scalar> &joint_state)
        {
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
        JointState<double> Free<Scalar, OrientationRepresentation>::randomJointState() const
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

        template class Free<double, ori_representation::RollPitchYaw>;
        template class Free<double, ori_representation::Quaternion>;
        template class Free<float, ori_representation::RollPitchYaw>;
        template class Free<float, ori_representation::Quaternion>;
        template class Free<casadi::SX, ori_representation::RollPitchYaw>;
        template class Free<casadi::SX, ori_representation::Quaternion>;
    }

} // namespace grbda
