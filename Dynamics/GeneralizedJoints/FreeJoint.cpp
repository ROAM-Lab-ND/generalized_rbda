#include "FreeJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        Free::Free(const Body &body) : Base(1, 7, 6, false, false), body_(body)
        {
            if (body.parent_index_ >= 0)
                throw std::runtime_error("Free joint is only valid as the first joint in a tree and thus cannot have a parent body");

            S_.setIdentity();
            Psi_.setIdentity();
            vJ_ = SVec<double>::Zero();

            single_joints_.emplace_back(new Joints::Free());

            gamma_ = [](DVec<double> y)
            { return y; };

            phi_ = [](DVec<double> q)
            { return DVec<double>::Zero(0); };

            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Identity(6, 6);

            loop_constraint_ = std::make_shared<LoopConstraint::Free>();
        }

        void Free::updateKinematics(const JointState &joint_state)
        {
#ifdef DEBUG_MODE
            jointStateCheck(joint_state);
#endif
            single_joints_[0]->updateKinematics(joint_state.position, joint_state.velocity);
            vJ_ = S_ * joint_state.velocity;
        }

        void Free::computeSpatialTransformFromParentToCurrentCluster(
            GeneralizedSpatialTransform &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 1 || Xup.getNumParentBodies() != 1)
                throw std::runtime_error("[Free Joint] Xup must be 6x6");
#endif
            Xup[0] = single_joints_[0]->XJ();
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
