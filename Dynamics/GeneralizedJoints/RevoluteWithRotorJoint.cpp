#include "RevoluteWithRotorJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        RevoluteWithRotor::RevoluteWithRotor(Body &link, Body &rotor, CoordinateAxis joint_axis,
                                             CoordinateAxis rotor_axis, double gear_ratio)
            : Base(2, 1, 1, false, false), link_(link), rotor_(rotor)
        {
            link_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis));
            rotor_joint_ = single_joints_.emplace_back(new Joints::Revolute(rotor_axis));

            gamma_ = [gear_ratio](DVec<double> y)
            {
                Vec2<double> q;
                q[0] = y[0];
                q[1] = gear_ratio * y[0];
                return q;
            };

            phi_ = [gear_ratio](DVec<double> q)
            {
                DVec<double> out = DVec<double>::Zero(1);
                out[0] = gear_ratio * q[0] - q[1];
                return out;
            };

            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Zero(1, 2);
            spanning_tree_to_independent_coords_conversion_ << 1., 0.;

            DMat<double> G = DMat<double>::Zero(2, 1);
            G << 1., gear_ratio;
            DMat<double> K = DMat<double>::Zero(1, 2);
            K << gear_ratio, -1.;
            loop_constraint_ = std::make_shared<LoopConstraint::Static>(G, K);

            S_.block<6, 1>(0, 0) = link_joint_->S();
            S_.block<6, 1>(6, 0) = gear_ratio * rotor_joint_->S();

            Psi_.block<6, 1>(6, 0) = 1. / gear_ratio * rotor_joint_->S();

            vJ_ = DVec<double>::Zero(12);
        }

        void RevoluteWithRotor::updateKinematics(const JointState &joint_state)
        {
#ifdef DEBUG_MODE
            jointStateCheck(joint_state);
#endif

            const JointState spanning_joint_state = toSpanningTreeState(joint_state);
            const DVec<double> &q = spanning_joint_state.position;
            const DVec<double> &qd = spanning_joint_state.velocity;

            link_joint_->updateKinematics(q.segment<1>(0), qd.segment<1>(0));
            rotor_joint_->updateKinematics(q.segment<1>(1), qd.segment<1>(1));

            vJ_.head<6>() = link_joint_->S() * qd[0];
            vJ_.tail<6>() = rotor_joint_->S() * qd[1];
        }

        void RevoluteWithRotor::computeSpatialTransformFromParentToCurrentCluster(
            GeneralizedSpatialTransform &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 2)
                throw std::runtime_error("[Revolute+Rotor Joint] Xup must have 12 rows");
#endif

            Xup[0] = link_joint_->XJ() * link_.Xtree_;
            Xup[1] = rotor_joint_->XJ() * rotor_.Xtree_;
        }

        std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        RevoluteWithRotor::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_reflected_inertias_;

            DMat<double> S_dependent = S_.bottomRows<6>();
            DMat<double> reflected_inertia =
                S_dependent.transpose() * rotor_.inertia_.getMatrix() * S_dependent;

            bodies_joints_and_reflected_inertias_.push_back(
                std::make_tuple(link_, link_joint_, reflected_inertia));

            return bodies_joints_and_reflected_inertias_;
        }
    }

} // namespace grbda
