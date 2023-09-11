#include "RevolutePairWithRotorJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        RevolutePairWithRotor::RevolutePairWithRotor(
            Body &link_1, Body &link_2, Body &rotor_1, Body &rotor_2,
            CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2,
            CoordinateAxis rotor_axis_1, CoordinateAxis rotor_axis_2,
            double gear_ratio_1, double gear_ratio_2,
            double belt_ratio_1, double belt_ratio_2)
            : Base(4, 2, 2, false, false), link_1_(link_1), link_2_(link_2),
              rotor_1_(rotor_1), rotor_2_(rotor_2)
        {
            double net_ratio_1 = gear_ratio_1 * belt_ratio_1;
            double net_ratio_2 = gear_ratio_2 * belt_ratio_2;

            link_1_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_1));
            rotor_1_joint_ = single_joints_.emplace_back(new Joints::Revolute(rotor_axis_1));
            rotor_2_joint_ = single_joints_.emplace_back(new Joints::Revolute(rotor_axis_2));
            link_2_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_2));

            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Identity(2, 4);
            spanning_tree_to_independent_coords_conversion_ << 1., 0., 0., 0., 0., 0., 0., 1.;

            DMat<double> G = DMat<double>::Zero(4, 2);
            G << 1., 0.,
                net_ratio_1, 0.,
                gear_ratio_2 * belt_ratio_1, net_ratio_2,
                0., 1.;
            DMat<double> K = DMat<double>::Identity(2, 4);
            K << net_ratio_1, -1., 0., 0.,
                gear_ratio_2 * belt_ratio_1, 0, -1., net_ratio_2;
            loop_constraint_ = std::make_shared<LoopConstraint::Static>(G, K);

            S_implict_ = DMat<double>::Zero(24, 4);
            S_implict_ring_ = DMat<double>::Zero(24, 4);

            S_implict_.block<6, 1>(0, 0) = link_1_joint_->S();
            S_implict_.block<6, 1>(6, 1) = rotor_1_joint_->S();
            S_implict_.block<6, 1>(12, 2) = rotor_2_joint_->S();
            S_implict_.block<6, 1>(18, 3) = link_2_joint_->S();

            S_ = S_implict_ * loop_constraint_->G();
        }

        void RevolutePairWithRotor::updateKinematics(const JointState &joint_state)
        {
#ifdef DEBUG_MODE
            jointStateCheck(joint_state);
#endif

            const JointState spanning_joint_state = toSpanningTreeState(joint_state);
            const DVec<double> &q = spanning_joint_state.position;
            const DVec<double> &qd = spanning_joint_state.velocity;

            link_1_joint_->updateKinematics(q.segment<1>(0), qd.segment<1>(0));
            rotor_1_joint_->updateKinematics(q.segment<1>(1), qd.segment<1>(1));
            rotor_2_joint_->updateKinematics(q.segment<1>(2), qd.segment<1>(2));
            link_2_joint_->updateKinematics(q.segment<1>(3), qd.segment<1>(3));

            X21_ = link_2_joint_->XJ() * link_2_.Xtree_;
            const DVec<double> v2_relative = link_2_joint_->S() * qd[3];

            S_implict_.block<6, 1>(18, 0) = X21_.transformMotionSubspace(link_1_joint_->S());
            S_.block<6, 1>(18, 0) = S_implict_.block<6, 1>(18, 0);

            S_implict_ring_.block<6, 1>(18, 0) = -generalMotionCrossMatrix(v2_relative) *
                                                 S_implict_.block<6, 1>(18, 0);

            vJ_ = S_implict_ * qd;
            cJ_ = S_implict_ring_ * qd;
        }

        void RevolutePairWithRotor::computeSpatialTransformFromParentToCurrentCluster(
            GeneralizedSpatialTransform &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 4)
                throw std::runtime_error("[RevolutePairWithRotor] Xup must have 24 rows");
#endif

            Xup[0] = link_1_joint_->XJ() * link_1_.Xtree_;
            Xup[1] = rotor_1_joint_->XJ() * rotor_1_.Xtree_;
            Xup[2] = rotor_2_joint_->XJ() * rotor_2_.Xtree_;
            Xup[3] = link_2_joint_->XJ() * link_2_.Xtree_ * Xup[0];
        }

        std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        RevolutePairWithRotor::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_ref_inertias_;

            DMat<double> S_dependent_1 = S_.middleRows<6>(6);
            Mat6<double> Ir1 = rotor_1_.inertia_.getMatrix();
            DMat<double> ref_inertia_1 = S_dependent_1.transpose() * Ir1 * S_dependent_1;
            bodies_joints_and_ref_inertias_.push_back(std::make_tuple(link_1_, link_1_joint_,
                                                                      ref_inertia_1));

            DMat<double> S_dependent_2 = S_.middleRows<6>(12);
            Mat6<double> Ir2 = rotor_2_.inertia_.getMatrix();
            DMat<double> ref_inertia_2 = S_dependent_2.transpose() * Ir2 * S_dependent_2;
            bodies_joints_and_ref_inertias_.push_back(std::make_tuple(link_2_, link_2_joint_,
                                                                      ref_inertia_2));

            return bodies_joints_and_ref_inertias_;
        }

    }

} // namespace grbda
