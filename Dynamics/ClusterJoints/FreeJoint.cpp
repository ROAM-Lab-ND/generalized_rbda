#include "FreeJoint.h"

namespace grbda
{

    namespace LoopConstraint
    {
        template <typename Scalar>
        Free<Scalar>::Free()
        {
            this->G_ = DMat<Scalar>::Identity(6, 6);
            this->g_ = DVec<Scalar>::Zero(6);

            this->K_ = DMat<Scalar>::Zero(0, 6);
            this->k_ = DVec<Scalar>::Zero(0);
        }

        template <typename Scalar>
        std::shared_ptr<Base<Scalar>> Free<Scalar>::clone() const
        {
            return std::make_shared<Free<Scalar>>(*this);
        }

        template <typename Scalar>
        DVec<Scalar> Free<Scalar>::gamma(const JointCoordinate<Scalar> &joint_pos) const
        {
            return joint_pos;
        }

        template class Free<double>;
        template class Free<casadi::SX>;

    }

    namespace ClusterJoints
    {

        template <typename Scalar>
        Free<Scalar>::Free(const Body<Scalar> &body) : Base<Scalar>(1, 7, 6), body_(body)
        {
            if (body.parent_index_ >= 0)
                throw std::runtime_error("Free joint is only valid as the first joint in a tree and thus cannot have a parent body");

            this->S_.setIdentity();
            this->Psi_.setIdentity();

            this->single_joints_.emplace_back(new Joints::Free<Scalar>());

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Identity(6, 6);

            this->loop_constraint_ = std::make_shared<LoopConstraint::Free<Scalar>>();
        }

        template <typename Scalar>
        void Free<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            this->single_joints_[0]->updateKinematics(joint_state.position, joint_state.velocity);
            this->vJ_ = this->S_ * joint_state.velocity;
        }

        template <typename Scalar>
        void Free<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 1 || Xup.getNumParentBodies() != 1)
                throw std::runtime_error("[Free Joint] Xup must be 6x6");
#endif
            Xup[0] = this->single_joints_[0]->XJ();
        }

        template <typename Scalar>
        JointCoordinate<Scalar> Free<Scalar>::integratePosition(JointState<Scalar> joint_state,
                                                                Scalar dt) const
        {
            const Quat<Scalar> quat = joint_state.position.template tail<4>();
            const Vec3<Scalar> lin_vel = joint_state.velocity.template tail<3>();
            const Vec3<Scalar> ang_vel = joint_state.velocity.template head<3>();

            joint_state.position.template head<3>() += lin_vel * dt;
            joint_state.position.template tail<4>() = ori::integrateQuat(quat, ang_vel, dt);

            return joint_state.position;
        }

        template <typename Scalar>
        JointState<Scalar> Free<Scalar>::randomJointState() const
        {
            JointState<Scalar> joint_state(false, false);
            joint_state.position = DVec<Scalar>::Zero(7);
            joint_state.position.template segment<3>(0) = Vec3<Scalar>::Random(3);
            joint_state.position.template segment<4>(3) = ori::rpyToQuat(Vec3<Scalar>::Random(3));
            joint_state.velocity = DVec<Scalar>::Random(6);
            return joint_state;
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        Free<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> bodies_joints_and_ref_inertias;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(body_, this->single_joints_[0],
                                                                     Mat6<Scalar>::Zero()));
            return bodies_joints_and_ref_inertias;
        }

        template class Free<double>;
        template class Free<casadi::SX>;

    }

} // namespace grbda
