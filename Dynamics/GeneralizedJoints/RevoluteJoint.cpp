#include "RevoluteJoint.h"

namespace grbda
{
    namespace GeneralizedJoints
    {

        Revolute::Revolute(const Body &body, CoordinateAxis joint_axis)
            : Base(1, 1, 1), body_(body)
        {
            single_joints_.emplace_back(new Joints::Revolute(joint_axis));

            S_ = single_joints_[0]->S();
            Psi_ = single_joints_[0]->S();
            vJ_ = SVec<double>::Zero();

            gamma_ = [](DVec<double> y)
            { return y; };
            G_ = DMat<double>::Identity(1, 1);
            g_ = DVec<double>::Zero(1);

            phi_ = [](DVec<double> q)
            { return DVec<double>::Zero(0); };
            K_ = DMat<double>::Identity(0, 1);
            k_ = DVec<double>::Zero(0);

            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Identity(1, 1);
        }

        void Revolute::updateKinematics(const JointState &joint_state)
        {
            // TODO(@MatthewChignoli): Only do these types of checks in debug mode
            if (joint_state.position.size() != 1 || joint_state.velocity.size() != 1)
                throw std::runtime_error("[Revolute Joint] Dimension of joint position must be 1");

            single_joints_[0]->updateKinematics(joint_state.position, joint_state.velocity);
            vJ_ = S_ * joint_state.velocity;
        }

        void Revolute::computeSpatialTransformFromParentToCurrentCluster(
            GeneralizedSpatialTransform &Xup) const
        {
            if (Xup.getNumOutputBodies() != 1)
                throw std::runtime_error("[Revolute Joint] Xup must have 6 rows");
            Xup[0] = single_joints_[0]->XJ() * body_.Xtree_;
        }

        std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        Revolute::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_reflected_inertias_;
            bodies_joints_and_reflected_inertias_.push_back(
                std::make_tuple(body_, single_joints_[0], DMat<double>::Zero(1, 1)));
            return bodies_joints_and_reflected_inertias_;
        }

        JointState Revolute::randomJointState() const
        {
            JointState joint_state(false, false);
            joint_state.position = DVec<double>::Random(1);
            joint_state.velocity = DVec<double>::Random(1);
            return joint_state;
        }
    }

} // namespace grbda
