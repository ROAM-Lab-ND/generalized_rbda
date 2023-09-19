#include "FreeJoint.h"

namespace grbda
{

    namespace LoopConstraint
    {
        Free::Free()
        {
            G_ = DMat<double>::Identity(6, 6);
            g_ = DVec<double>::Zero(6);

            K_ = DMat<double>::Zero(0, 6);
            k_ = DVec<double>::Zero(0);
        }

        std::shared_ptr<Base> Free::clone() const
        {
            return std::make_shared<Free>(*this);
        }

        DVec<double> Free::gamma(const JointCoordinate &joint_pos) const
        {
            return joint_pos;
        }

    }

    namespace GeneralizedJoints
    {

        Free::Free(const Body &body) : Base(1, 7, 6), body_(body)
        {
            if (body.parent_index_ >= 0)
                throw std::runtime_error("Free joint is only valid as the first joint in a tree and thus cannot have a parent body");

            S_.setIdentity();
            Psi_.setIdentity();

            single_joints_.emplace_back(new Joints::Free());

            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Identity(6, 6);

            loop_constraint_ = std::make_shared<LoopConstraint::Free>();
        }

        void Free::updateKinematics(const JointState &joint_state)
        {
            single_joints_[0]->updateKinematics(joint_state.position, joint_state.velocity);
            vJ_ = S_ * joint_state.velocity;
        }

        void Free::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 1 || Xup.getNumParentBodies() != 1)
                throw std::runtime_error("[Free Joint] Xup must be 6x6");
#endif
            Xup[0] = single_joints_[0]->XJ();
        }

        JointCoordinate Free::integratePosition(JointState joint_state, double dt) const
        {
            const Quat<double> quat = joint_state.position.tail<4>();
            const DVec<double> &vel = joint_state.velocity;

            joint_state.position.head<3>() += vel.tail<3>() * dt;
            joint_state.position.tail<4>() = ori::integrateQuat(quat, vel.head<3>(), dt);

            return joint_state.position;
        }

        JointState Free::randomJointState() const
        {
            JointState joint_state(false, false);
            joint_state.position = DVec<double>::Zero(7);
            joint_state.position.segment<3>(0) = Vec3<double>::Random(3);
            joint_state.position.segment<4>(3) = ori::rpyToQuat(Vec3<double>::Random(3));
            joint_state.velocity = DVec<double>::Random(6);
            return joint_state;
        }

        std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        Free::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_ref_inertias_;
            bodies_joints_and_ref_inertias_.push_back(std::make_tuple(body_, single_joints_[0],
                                                                      Mat6<double>::Zero()));
            return bodies_joints_and_ref_inertias_;
        }

    }

} // namespace grbda
