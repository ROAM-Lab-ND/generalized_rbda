#include "RevolutePairWithRotorJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        RevolutePairWithRotor::RevolutePairWithRotor(ParallelBeltTransmissionModule &module_1,
                                                     ParallelBeltTransmissionModule &module_2)
            : Base(4, 2, 2), link1_(module_1.body_), link2_(module_2.body_),
              rotor1_(module_1.rotor_), rotor2_(module_2.rotor_)
        {
            const double &gear_ratio_2 = module_2.gear_ratio_;
            const double &belt_ratio_1 = module_1.belt_ratio_;
            const double net_ratio_1 = module_1.gear_ratio_ * belt_ratio_1;
            const double net_ratio_2 = gear_ratio_2 * module_2.belt_ratio_;

            link1_joint_ = single_joints_.emplace_back(new Joints::Revolute(module_1.joint_axis_));
            rotor1_joint_ = single_joints_.emplace_back(new Joints::Revolute(module_1.rotor_axis_));
            rotor2_joint_ = single_joints_.emplace_back(new Joints::Revolute(module_2.rotor_axis_));
            link2_joint_ = single_joints_.emplace_back(new Joints::Revolute(module_2.joint_axis_));

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

            X_inter_S_span_ = DMat<double>::Zero(24, 4);
            X_inter_S_span_ring_ = DMat<double>::Zero(24, 4);

            X_inter_S_span_.block<6, 1>(0, 0) = link1_joint_->S();
            X_inter_S_span_.block<6, 1>(6, 1) = rotor1_joint_->S();
            X_inter_S_span_.block<6, 1>(12, 2) = rotor2_joint_->S();
            X_inter_S_span_.block<6, 1>(18, 3) = link2_joint_->S();

            S_ = X_inter_S_span_ * loop_constraint_->G();
        }

        void RevolutePairWithRotor::updateKinematics(const JointState &joint_state)
        {
            const JointState spanning_joint_state = toSpanningTreeState(joint_state);
            const DVec<double> &q = spanning_joint_state.position;
            const DVec<double> &qd = spanning_joint_state.velocity;

            link1_joint_->updateKinematics(q.segment<1>(0), qd.segment<1>(0));
            rotor1_joint_->updateKinematics(q.segment<1>(1), qd.segment<1>(1));
            rotor2_joint_->updateKinematics(q.segment<1>(2), qd.segment<1>(2));
            link2_joint_->updateKinematics(q.segment<1>(3), qd.segment<1>(3));

            X21_ = link2_joint_->XJ() * link2_.Xtree_;
            const DVec<double> v2_relative = link2_joint_->S() * qd[3];

            X_inter_S_span_.block<6, 1>(18, 0) = X21_.transformMotionSubspace(link1_joint_->S());
            S_.block<6, 1>(18, 0) = X_inter_S_span_.block<6, 1>(18, 0);

            X_inter_S_span_ring_.block<6, 1>(18, 0) =
                -spatial::generalMotionCrossMatrix(v2_relative) *
                X_inter_S_span_.block<6, 1>(18, 0);

            vJ_ = X_inter_S_span_ * qd;
            cJ_ = X_inter_S_span_ring_ * qd;
        }

        void RevolutePairWithRotor::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 4)
                throw std::runtime_error("[RevolutePairWithRotor] Xup must have 24 rows");
#endif

            Xup[0] = link1_joint_->XJ() * link1_.Xtree_;
            Xup[1] = rotor1_joint_->XJ() * rotor1_.Xtree_;
            Xup[2] = rotor2_joint_->XJ() * rotor2_.Xtree_;
            Xup[3] = link2_joint_->XJ() * link2_.Xtree_ * Xup[0];
        }

        std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        RevolutePairWithRotor::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_ref_inertias_;

            DMat<double> S_dependent_1 = S_.middleRows<6>(6);
            Mat6<double> Ir1 = rotor1_.inertia_.getMatrix();
            DMat<double> ref_inertia_1 = S_dependent_1.transpose() * Ir1 * S_dependent_1;
            bodies_joints_and_ref_inertias_.push_back(std::make_tuple(link1_, link1_joint_,
                                                                      ref_inertia_1));

            DMat<double> S_dependent_2 = S_.middleRows<6>(12);
            Mat6<double> Ir2 = rotor2_.inertia_.getMatrix();
            DMat<double> ref_inertia_2 = S_dependent_2.transpose() * Ir2 * S_dependent_2;
            bodies_joints_and_ref_inertias_.push_back(std::make_tuple(link2_, link2_joint_,
                                                                      ref_inertia_2));

            return bodies_joints_and_ref_inertias_;
        }

    }

} // namespace grbda
