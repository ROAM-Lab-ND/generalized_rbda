#include "RevolutePairJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        RevolutePair::RevolutePair(Body &link_1, Body &link_2, CoordinateAxis joint_axis_1,
                                   CoordinateAxis joint_axis_2)
            : Base(2, 2, 2, false, false), link_1_(link_1), link_2_(link_2)
        {
            link_1_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_1));
            link_2_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_2));

            // ISSUE: #72 (old repo)
            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Zero(0, 0);

            DMat<double> G = DMat<double>::Zero(2, 2);
            G << 1., 0.,
                0., 1.;
            const DMat<double> K = DMat<double>::Identity(0, 2);
            loop_constraint_ = std::make_shared<LoopConstraint::Static>(G, K);

            S_ = DMat<double>::Zero(12, 2);
            S_.block<6, 1>(0, 0) = link_1_joint_->S();
            S_.block<6, 1>(6, 1) = link_2_joint_->S();

            vJ_ = DVec<double>::Zero(12);
        }

        void RevolutePair::updateKinematics(const JointState &joint_state)
        {
#ifdef DEBUG_MODE
            jointStateCheck(joint_state);
#endif

            const JointState spanning_joint_state = toSpanningTreeState(joint_state);
            const DVec<double> &q = spanning_joint_state.position;
            const DVec<double> &qd = spanning_joint_state.velocity;

            link_1_joint_->updateKinematics(q.segment<1>(0), qd.segment<1>(0));
            link_2_joint_->updateKinematics(q.segment<1>(1), qd.segment<1>(1));

            X21_ = link_2_joint_->XJ() * link_2_.Xtree_;
            const DVec<double> v2_relative = link_2_joint_->S() * qd[1];

            S_.block<6, 1>(6, 0) = X21_.transformMotionSubspace(link_1_joint_->S());

            vJ_ = S_ * joint_state.velocity;

            cJ_.segment<6>(6) = -generalMotionCrossMatrix(v2_relative) *
                                S_.block<6, 1>(6, 0) * joint_state.velocity;
        }

        void RevolutePair::computeSpatialTransformFromParentToCurrentCluster(
            GeneralizedSpatialTransform &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 2)
                throw std::runtime_error("[RevolutePair] Xup must have 12 rows");
#endif

            Xup[0] = link_1_joint_->XJ() * link_1_.Xtree_;
            Xup[1] = link_2_joint_->XJ() * link_2_.Xtree_ * Xup[0];
        }

        std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        RevolutePair::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_reflected_inertias_;

            const DMat<double> reflected_inertia_1 =
                DMat<double>::Zero(link_1_joint_->numVelocities(), link_1_joint_->numVelocities());
            bodies_joints_and_reflected_inertias_.push_back(
                std::make_tuple(link_1_, link_1_joint_, reflected_inertia_1));

            const DMat<double> reflected_inertia_2 =
                DMat<double>::Zero(link_2_joint_->numVelocities(), link_2_joint_->numVelocities());
            bodies_joints_and_reflected_inertias_.push_back(
                std::make_tuple(link_2_, link_2_joint_, reflected_inertia_2));

            return bodies_joints_and_reflected_inertias_;
        }
    }

} // namespace grbda
