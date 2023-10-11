#include "FreeJoint.h"

namespace grbda
{

    namespace LoopConstraint
    {
        template <typename Scalar, typename OrientationRepresentation>
        Free<Scalar, OrientationRepresentation>::Free()
        {
            this->G_ = DMat<Scalar>::Identity(6, 6);
            this->g_ = DVec<Scalar>::Zero(6);

            this->K_ = DMat<Scalar>::Zero(0, 6);
            this->k_ = DVec<Scalar>::Zero(0);
        }

        template <typename Scalar, typename OrientationRepresentation>
        std::shared_ptr<Base<Scalar>> Free<Scalar, OrientationRepresentation>::clone() const
        {
            return std::make_shared<Free<Scalar, OrientationRepresentation>>(*this);
        }

        template <typename Scalar, typename OrientationRepresentation>
        DVec<Scalar> Free<Scalar, OrientationRepresentation>::gamma(const JointCoordinate<Scalar> &joint_pos) const
        {
            return joint_pos;
        }

        template class Free<double, ori_representation::RollPitchYawRepresentation< Vec3<double> >>;
        template class Free<double, ori_representation::QuaternionRepresentation< Quat<casadi::SX> >>;
        template class Free<casadi::SX, ori_representation::RollPitchYawRepresentation< Vec3<casadi::SX> >>;
        template class Free<casadi::SX, ori_representation::QuaternionRepresentation< Quat<casadi::SX> >>;

    }

    namespace ClusterJoints
    {

        template <typename Scalar, typename OrientationRepresentation>
        Free<Scalar, OrientationRepresentation>::Free(const Body<Scalar> &body) : Base<Scalar>(1, OrientationRepresentation::num_ori_parameter + 3, 6), body_(body)
        {
            if (body.parent_index_ >= 0)
                throw std::runtime_error("Free joint is only valid as the first joint in a tree and thus cannot have a parent body");

            this->S_.setIdentity();
            this->Psi_.setIdentity();

            this->single_joints_.emplace_back(new Joints::Free<Scalar, OrientationRepresentation>());

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Identity(6, 6);

            this->loop_constraint_ = std::make_shared<LoopConstraint::Free<Scalar, OrientationRepresentation>>();
        }

        template <typename Scalar, typename OrientationRepresentation>
        void Free<Scalar, OrientationRepresentation>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            this->single_joints_[0]->updateKinematics(joint_state.position, joint_state.velocity);
            this->vJ_ = this->S_ * joint_state.velocity;
        }

        template <typename Scalar, typename OrientationRepresentation>
        void Free<Scalar, OrientationRepresentation>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 1 || Xup.getNumParentBodies() != 1)
                throw std::runtime_error("[Free Joint] Xup must be 6x6");
#endif
            Xup[0] = this->single_joints_[0]->XJ();
        }

        template <typename Scalar, typename OrientationRepresentation>
        JointState<Scalar> Free<Scalar, OrientationRepresentation>::randomJointState() const
        {
            JointState<Scalar> joint_state(false, false);
            joint_state.position = DVec<Scalar>::Zero(OrientationRepresentation::num_ori_parameter + 3);
            joint_state.position.template segment<3>(0) = Vec3<Scalar>::Random(3);
            joint_state.position.template segment<OrientationRepresentation::num_ori_parameter>(3) = OrientationRepresentation::randomOrientation();
            joint_state.velocity = DVec<Scalar>::Random(6);
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

        template class Free<double, ori_representation::RollPitchYawRepresentation< Vec3<double> >>;
        template class Free<double, ori_representation::QuaternionRepresentation< Quat<double> >>;
        template class Free<casadi::SX, ori_representation::RollPitchYawRepresentation< Vec3<casadi::SX> >>;
        template class Free<casadi::SX, ori_representation::QuaternionRepresentation< Quat<casadi::SX> >>;

    }

} // namespace grbda
