#include "RevoluteTripleWithRotorJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        RevoluteTripleWithRotor::RevoluteTripleWithRotor(
            const ParallelBeltTransmissionModule &module_1,
            const ParallelBeltTransmissionModule &module_2,
            const ParallelBeltTransmissionModule &module_3)
            : Base(6, 3, 3, false, false), link_1_(module_1.body_), link_2_(module_2.body_),
              link_3_(module_3.body_), rotor_1_(module_1.rotor_), rotor_2_(module_2.rotor_),
              rotor_3_(module_3.rotor_)
        {
            link_1_joint_ = single_joints_.emplace_back(new Joints::Revolute(module_1.joint_axis_));
            link_2_joint_ = single_joints_.emplace_back(new Joints::Revolute(module_2.joint_axis_));
            link_3_joint_ = single_joints_.emplace_back(new Joints::Revolute(module_3.joint_axis_));

            rotor_1_joint_ = 
                single_joints_.emplace_back(new Joints::Revolute(module_1.rotor_axis_));
            rotor_2_joint_ =
                single_joints_.emplace_back(new Joints::Revolute(module_2.rotor_axis_));
            rotor_3_joint_ =
                single_joints_.emplace_back(new Joints::Revolute(module_3.rotor_axis_));

            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Zero(3, 6);
            spanning_tree_to_independent_coords_conversion_.topLeftCorner<3, 3>().setIdentity();

            const double &gr1 = module_1.gear_ratio_;
            const double &gr2 = module_2.gear_ratio_;
            const double &gr3 = module_3.gear_ratio_;
            const double &br1 = module_1.belt_ratio_;
            const double &br2 = module_2.belt_ratio_;
            const double &br3 = module_3.belt_ratio_;

            DMat<double> G = DMat<double>::Zero(6, 3);
            G.topRows<3>().setIdentity();
            G.row(3) << gr1 * br1, 0., 0.;
            G.row(4) << gr2 * br1, gr2 * br2, 0.;
            G.row(5) << -gr3 * br1, -gr3 * br2, gr3 * br3;

            DMat<double> K = DMat<double>::Zero(3, 6);
            K.leftCols(3) = -G.bottomRows(3);
            K.rightCols(3).setIdentity();
            loop_constraint_ = std::make_shared<LoopConstraint::Static>(G, K);

            X_inter_S_span_ = DMat<double>::Zero(36, 6);
            X_inter_S_span_ring_ = DMat<double>::Zero(36, 6);

            X_inter_S_span_.block<6, 1>(0, 0) = link_1_joint_->S();
            X_inter_S_span_.block<6, 1>(6, 1) = link_2_joint_->S();
            X_inter_S_span_.block<6, 1>(12, 2) = link_3_joint_->S();
            X_inter_S_span_.block<6, 1>(18, 3) = rotor_1_joint_->S();
            X_inter_S_span_.block<6, 1>(24, 4) = rotor_2_joint_->S();
            X_inter_S_span_.block<6, 1>(30, 5) = rotor_3_joint_->S();

            S_ = X_inter_S_span_ * loop_constraint_->G();
        }

        void RevoluteTripleWithRotor::updateKinematics(const JointState &joint_state)
        {
#ifdef DEBUG_MODE
            jointStateCheck(joint_state);
#endif

            const JointState spanning_joint_state = toSpanningTreeState(joint_state);
            const DVec<double> &q = spanning_joint_state.position;
            const DVec<double> &qd = spanning_joint_state.velocity;

            link_1_joint_->updateKinematics(q.segment<1>(0), qd.segment<1>(0));
            link_2_joint_->updateKinematics(q.segment<1>(1), qd.segment<1>(1));
            link_3_joint_->updateKinematics(q.segment<1>(2), qd.segment<1>(2));
            rotor_1_joint_->updateKinematics(q.segment<1>(3), qd.segment<1>(3));
            rotor_2_joint_->updateKinematics(q.segment<1>(4), qd.segment<1>(4));
            rotor_3_joint_->updateKinematics(q.segment<1>(5), qd.segment<1>(5));

            X21_ = link_2_joint_->XJ() * link_2_.Xtree_;
            X32_ = link_3_joint_->XJ() * link_3_.Xtree_;
            X31_ = X32_ * X21_;

            const DVec<double> v2_relative1 = link_2_joint_->S() * qd[1];
            const DMat<double> X21_S1 = X21_.transformMotionSubspace(link_1_joint_->S());
            const DVec<double> v3_relative1 = X32_.transformMotionVector(v2_relative1) +
                                              link_3_joint_->S() * qd[2];
            const DMat<double> X31_S1 = X31_.transformMotionSubspace(link_1_joint_->S());
            const DVec<double> v3_relative2 = link_3_joint_->S() * qd[2];
            const DMat<double> X32_S2 = X32_.transformMotionSubspace(link_2_joint_->S());

            X_inter_S_span_.block<6, 1>(6, 0) = X21_S1;
            X_inter_S_span_.block<6, 1>(12, 0) = X31_S1;
            X_inter_S_span_.block<6, 1>(12, 1) = X32_S2;

            S_.topLeftCorner<18, 3>() = X_inter_S_span_.topLeftCorner<18, 3>();

            X_inter_S_span_ring_.block<6, 1>(6, 0) =
                -spatial::generalMotionCrossMatrix(v2_relative1) * X21_S1;
            X_inter_S_span_ring_.block<6, 1>(12, 0) =
                -spatial::generalMotionCrossMatrix(v3_relative1) * X31_S1;
            X_inter_S_span_ring_.block<6, 1>(12, 1) =
                -spatial::generalMotionCrossMatrix(v3_relative2) * X32_S2;

            vJ_ = X_inter_S_span_ * qd;
            cJ_ = X_inter_S_span_ring_ * qd;
        }

        void RevoluteTripleWithRotor::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 4)
                throw std::runtime_error("[RevoluteTripleWithRotor] Xup must have 24 rows");
#endif

            Xup[0] = link_1_joint_->XJ() * link_1_.Xtree_;
            Xup[1] = X21_ * Xup[0];
            Xup[2] = X31_ * Xup[0];
            Xup[3] = rotor_1_joint_->XJ() * rotor_1_.Xtree_;
            Xup[4] = rotor_2_joint_->XJ() * rotor_2_.Xtree_;
            Xup[5] = rotor_3_joint_->XJ() * rotor_3_.Xtree_;
        }

        std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        RevoluteTripleWithRotor::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_ref_inertias_;

            const DMat<double> S_dependent_1 = S_.middleRows<6>(18);
            const Mat6<double> Ir1 = rotor_1_.inertia_.getMatrix();
            const DMat<double> ref_inertia_1 = S_dependent_1.transpose() * Ir1 * S_dependent_1;
            bodies_joints_and_ref_inertias_.push_back(std::make_tuple(link_1_, link_1_joint_,
                                                                      ref_inertia_1));

            const DMat<double> S_dependent_2 = S_.middleRows<6>(24);
            const Mat6<double> Ir2 = rotor_2_.inertia_.getMatrix();
            const DMat<double> ref_inertia_2 = S_dependent_2.transpose() * Ir2 * S_dependent_2;
            bodies_joints_and_ref_inertias_.push_back(std::make_tuple(link_2_, link_2_joint_,
                                                                      ref_inertia_2));

            const DMat<double> S_dependent_3 = S_.middleRows<6>(30);
            const Mat6<double> Ir3 = rotor_3_.inertia_.getMatrix();
            const DMat<double> ref_inertia_3 = S_dependent_3.transpose() * Ir3 * S_dependent_3;
            bodies_joints_and_ref_inertias_.push_back(std::make_tuple(link_3_, link_3_joint_,
                                                                      ref_inertia_3));

            return bodies_joints_and_ref_inertias_;
        }

    }

} // namespace grbda
