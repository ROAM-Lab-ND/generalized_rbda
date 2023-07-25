#include "RevoluteTripleWithRotorJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        RevoluteTripleWithRotor::RevoluteTripleWithRotor(
            std::vector<ParallelBeltTransmissionModule> modules)
            : Base(6, 3, 3, false, false), link_1_(modules[0].body_), link_2_(modules[1].body_),
              link_3_(modules[2].body_), rotor_1_(modules[0].rotor_), rotor_2_(modules[1].rotor_),
              rotor_3_(modules[2].rotor_)
        {
            link_1_joint_ =
                single_joints_.emplace_back(new Joints::Revolute(modules[0].joint_axis_));
            link_2_joint_ =
                single_joints_.emplace_back(new Joints::Revolute(modules[1].joint_axis_));
            link_3_joint_ =
                single_joints_.emplace_back(new Joints::Revolute(modules[2].joint_axis_));

            rotor_1_joint_ =
                single_joints_.emplace_back(new Joints::Revolute(modules[0].rotor_axis_));
            rotor_2_joint_ =
                single_joints_.emplace_back(new Joints::Revolute(modules[1].rotor_axis_));
            rotor_3_joint_ =
                single_joints_.emplace_back(new Joints::Revolute(modules[2].rotor_axis_));

            spanning_tree_to_independent_coords_conversion_ = DMat<double>::Zero(3, 6);
            spanning_tree_to_independent_coords_conversion_.topLeftCorner<3, 3>().setIdentity();

            const double &gr1 = modules[0].gear_ratio_;
            const double &gr2 = modules[1].gear_ratio_;
            const double &gr3 = modules[2].gear_ratio_;
            const double &br1 = modules[0].belt_ratio_;
            const double &br2 = modules[1].belt_ratio_;
            const double &br3 = modules[2].belt_ratio_;

            DMat<double> G = DMat<double>::Zero(6, 3);
            G.topRows<3>().setIdentity();
            G.row(3) << gr1 * br1, 0., 0.;
            G.row(4) << gr2 * br1, gr2 * br2, 0.;
            G.row(5) << -gr3 * br1, -gr3 * br2, gr3 * br3;

            DMat<double> K = DMat<double>::Zero(3, 6);
            K.leftCols(3) = -G.bottomRows(3);
            K.rightCols(3).setIdentity();
            loop_constraint_ = std::make_shared<LoopConstraint::Static>(G, K);

            S_.block<6, 1>(0, 0) = link_1_joint_->S();
            S_.block<6, 1>(18, 0) = gr1 * br1 * rotor_1_joint_->S();
            S_.block<6, 1>(24, 0) = gr2 * br1 * rotor_2_joint_->S();
            S_.block<6, 1>(30, 0) = -gr3 * br1 * rotor_3_joint_->S();

            S_.block<6, 1>(6, 1) = link_2_joint_->S();
            S_.block<6, 1>(24, 1) = gr2 * br2 * rotor_2_joint_->S();
            S_.block<6, 1>(30, 1) = -gr3 * br2 * rotor_3_joint_->S();

            S_.block<6, 1>(12, 2) = link_3_joint_->S();
            S_.block<6, 1>(30, 2) = gr3 * br3 * rotor_3_joint_->S();

            vJ_ = DVec<double>::Zero(36);
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

            S_.block<6, 1>(6, 0) = X21_.transformMotionSubspace(link_1_joint_->S());
            S_.block<6, 1>(12, 0) = X31_.transformMotionSubspace(link_1_joint_->S());
            S_.block<6, 1>(12, 1) = X32_.transformMotionSubspace(link_2_joint_->S());

            const DVec<double> v2_relative1 = link_2_joint_->S() * qd[1];
            S_ring_.block<6, 1>(6, 0) = -generalMotionCrossMatrix(v2_relative1) *
                                        X21_.transformMotionSubspace(link_1_joint_->S());

            const DVec<double> v3_relative1 = X32_.transformMotionVector(v2_relative1) +
                                              link_3_joint_->S() * qd[2];
            S_ring_.block<6, 1>(12, 0) = -generalMotionCrossMatrix(v3_relative1) *
                                         X31_.transformMotionSubspace(link_1_joint_->S());

            const DVec<double> v3_relative2 = link_3_joint_->S() * qd[2];
            S_ring_.block<6, 1>(12, 1) = -generalMotionCrossMatrix(v3_relative2) *
                                         X32_.transformMotionSubspace(link_2_joint_->S());

            vJ_ = S_ * joint_state.velocity;
        }

        void RevoluteTripleWithRotor::computeSpatialTransformFromParentToCurrentCluster(
            GeneralizedSpatialTransform &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 6)
                throw std::runtime_error("[RevoluteTripleWithRotor] Xup must have 36 rows");
#endif

            // TODO(@MatthewChignoli): Could be some cost savings here by not recomputing the internal spatial transforms
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
