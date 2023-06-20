#include "RevolutePairJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        RevolutePair::RevolutePair(Body &link_1, Body &link_2, CoordinateAxis joint_axis_1,
                                   CoordinateAxis joint_axis_2)
            : Base(2, 2, 2), link_1_(link_1), link_2_(link_2)
        {
            link_1_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_1));
            link_2_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_2));

            gamma_ = [&](DVec<double> y)
            {
                Vec2<double> q;
                q[0] = y[0];
                q[1] = y[1] - y[0];
                return q;
            };

            G_ = DMat<double>::Zero(2, 2);
            G_ << 1., 0.,
                -1., 1.;

            g_ = DVec<double>::Zero(2);

            phi_ = [&](DVec<double> q)
            {
                return DVec<double>::Zero(0);
            };
            K_ = DMat<double>::Zero(0, 2);
            k_ = DVec<double>::Zero(0);

            // ISSUE: #72
            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Zero(0, 0);

            S_ = DMat<double>::Zero(12, 2);
            S_.block<6, 1>(0, 0) = link_1_joint_->S();
            S_.block<6, 1>(6, 1) = link_2_joint_->S();

            vJ_ = DVec<double>::Zero(12);
        }

        void RevolutePair::updateKinematics(const JointState &joint_state)
        {
            // TODO(@MatthewChignoli): Commented out because this depends on the State type
            // if (y.size() != 2)
                // throw std::runtime_error("[RevolutePair] Dimension of joint position must be 2");

            const JointState spanning_joint_state = toSpanningTreeState(joint_state);
            const DVec<double> &q = spanning_joint_state.position;
            const DVec<double> &qd = spanning_joint_state.velocity;

            link_1_joint_->updateKinematics(q.segment<1>(0), qd.segment<1>(0));
            link_2_joint_->updateKinematics(q.segment<1>(1), qd.segment<1>(1));

            X21_ = link_2_joint_->XJ() * link_2_.Xtree_;

            S_.block<6, 1>(6, 0) = X21_.transformMotionSubspace(link_1_joint_->S()) - link_2_joint_->S();

            const DVec<double> v2_relative = link_2_joint_->S() * qd[1];
            S_ring_.block<6, 1>(6, 0) = -generalMotionCrossMatrix(v2_relative) *
                                        X21_.transformMotionSubspace(link_1_joint_->S());

            // TODO(@MatthewChignoli): Issue if joint_vel is spanning
            vJ_ = S_ * joint_state.velocity;
        }

        void RevolutePair::computeSpatialTransformFromParentToCurrentCluster(
            GeneralizedSpatialTransform &Xup) const
        {
            if (Xup.getNumOutputBodies() != 2)
                throw std::runtime_error("[RevolutePair] Xup must have 12 rows");

            Xup[0] = link_1_joint_->XJ() * link_1_.Xtree_;
            Xup[1] = link_2_joint_->XJ() * link_2_.Xtree_ * Xup[0];
        }

        std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        RevolutePair::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_reflected_inertias_;

            DMat<double> reflected_inertia_1 = DMat<double>::Zero(link_1_joint_->numVelocities(),
                                                                  link_1_joint_->numVelocities());
            bodies_joints_and_reflected_inertias_.push_back(
                std::make_tuple(link_1_, link_1_joint_, reflected_inertia_1));

            DMat<double> reflected_inertia_2 = DMat<double>::Zero(link_2_joint_->numVelocities(),
                                                                  link_2_joint_->numVelocities());
            bodies_joints_and_reflected_inertias_.push_back(
                std::make_tuple(link_2_, link_2_joint_, reflected_inertia_2));

            return bodies_joints_and_reflected_inertias_;
        }

        JointState RevolutePair::randomJointState() const
        {
            JointState joint_state(false, false);
            joint_state.position = DVec<double>::Random(numPositions());
            joint_state.velocity = DVec<double>::Random(numVelocities());
            return joint_state;
        }
    }

} // namespace grbda
