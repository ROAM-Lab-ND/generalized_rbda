#include "RevoluteJoint.h"

namespace grbda
{
    namespace GeneralizedJoints
    {

        Revolute::Revolute(const Body &body, CoordinateAxis joint_axis)
            : Base(1, 1, 1, false, false), body_(body)
        {
            single_joints_.emplace_back(new Joints::Revolute(joint_axis));

            S_ = single_joints_[0]->S();
            Psi_ = single_joints_[0]->S();
            vJ_ = SVec<double>::Zero();

            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Identity(1, 1);

            const DMat<double> G = DMat<double>::Identity(1, 1);
            const DMat<double> K = DMat<double>::Identity(0, 1);
            loop_constraint_ = std::make_shared<LoopConstraint::Static>(G, K);

        }

        void Revolute::updateKinematics(const JointState &joint_state)
        {
#ifdef DEBUG_MODE
            jointStateCheck(joint_state);
#endif

            single_joints_[0]->updateKinematics(joint_state.position, joint_state.velocity);
            vJ_ = S_ * joint_state.velocity;
        }

        void Revolute::computeSpatialTransformFromParentToCurrentCluster(
            GeneralizedSpatialTransform &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 1)
                throw std::runtime_error("[Revolute Joint] Xup must have 6 rows");
#endif
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

    }

} // namespace grbda
