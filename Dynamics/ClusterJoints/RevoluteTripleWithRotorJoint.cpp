#include "RevoluteTripleWithRotorJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar>
        RevoluteTripleWithRotor<Scalar>::RevoluteTripleWithRotor(
            const ParallelBeltTransmissionModule<1, Scalar> &module_1,
            const ParallelBeltTransmissionModule<2, Scalar> &module_2,
            const ParallelBeltTransmissionModule<3, Scalar> &module_3)
            : Base<Scalar>(6, 3, 3), link_1_(module_1.body_), link_2_(module_2.body_),
              link_3_(module_3.body_), rotor_1_(module_1.rotor_), rotor_2_(module_2.rotor_),
              rotor_3_(module_3.rotor_)
        {
            using Rev = Joints::Revolute<Scalar>;

            link_1_joint_ = this->single_joints_.emplace_back(new Rev(module_1.joint_axis_));
            link_2_joint_ = this->single_joints_.emplace_back(new Rev(module_2.joint_axis_));
            link_3_joint_ = this->single_joints_.emplace_back(new Rev(module_3.joint_axis_));

            rotor_1_joint_ = this->single_joints_.emplace_back(new Rev(module_1.rotor_axis_));
            rotor_2_joint_ = this->single_joints_.emplace_back(new Rev(module_2.rotor_axis_));
            rotor_3_joint_ = this->single_joints_.emplace_back(new Rev(module_3.rotor_axis_));

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Zero(3, 6);
            this->spanning_tree_to_independent_coords_conversion_.template topLeftCorner<3, 3>().setIdentity();

            Vec3<Scalar> gear_ratios{module_1.gear_ratio_, module_2.gear_ratio_, module_3.gear_ratio_};
            Eigen::DiagonalMatrix<Scalar, 3> rotor_matrix(gear_ratios);

            DMat<Scalar> belt_matrix = DMat<Scalar>::Zero(3, 3);
            belt_matrix << beltMatrixRowFromBeltRatios(module_1.belt_ratios_), 0., 0.,
                beltMatrixRowFromBeltRatios(module_2.belt_ratios_), 0.,
                beltMatrixRowFromBeltRatios(module_3.belt_ratios_);

            DMat<Scalar> G = DMat<Scalar>::Zero(6, 3);
            G.template topRows<3>().setIdentity();
            G.template bottomRows<3>() = rotor_matrix * belt_matrix;

            DMat<Scalar> K = DMat<Scalar>::Zero(3, 6);
            K.template leftCols(3) = -G.bottomRows(3);
            K.template rightCols(3).setIdentity();
            this->loop_constraint_ = std::make_shared<LoopConstraint::Static<Scalar>>(G, K);

            X_intra_S_span_ = DMat<Scalar>::Zero(36, 6);
            X_intra_S_span_ring_ = DMat<Scalar>::Zero(36, 6);

            X_intra_S_span_.template block<6, 1>(0, 0) = link_1_joint_->S();
            X_intra_S_span_.template block<6, 1>(6, 1) = link_2_joint_->S();
            X_intra_S_span_.template block<6, 1>(12, 2) = link_3_joint_->S();
            X_intra_S_span_.template block<6, 1>(18, 3) = rotor_1_joint_->S();
            X_intra_S_span_.template block<6, 1>(24, 4) = rotor_2_joint_->S();
            X_intra_S_span_.template block<6, 1>(30, 5) = rotor_3_joint_->S();

            this->S_ = X_intra_S_span_ * this->loop_constraint_->G();
        }

        template <typename Scalar>
        void RevoluteTripleWithRotor<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<Scalar> &q = spanning_joint_state.position;
            const DVec<Scalar> &qd = spanning_joint_state.velocity;

            link_1_joint_->updateKinematics(q.template segment<1>(0), qd.template segment<1>(0));
            link_2_joint_->updateKinematics(q.template segment<1>(1), qd.template segment<1>(1));
            link_3_joint_->updateKinematics(q.template segment<1>(2), qd.template segment<1>(2));
            rotor_1_joint_->updateKinematics(q.template segment<1>(3), qd.template segment<1>(3));
            rotor_2_joint_->updateKinematics(q.template segment<1>(4), qd.template segment<1>(4));
            rotor_3_joint_->updateKinematics(q.template segment<1>(5), qd.template segment<1>(5));

            X21_ = link_2_joint_->XJ() * link_2_.Xtree_;
            X32_ = link_3_joint_->XJ() * link_3_.Xtree_;
            X31_ = X32_ * X21_;

            const DVec<Scalar> v2_relative1 = link_2_joint_->S() * qd[1];
            const DMat<Scalar> X21_S1 = X21_.transformMotionSubspace(link_1_joint_->S());
            const DVec<Scalar> v3_relative1 = X32_.transformMotionVector(v2_relative1) +
                                              link_3_joint_->S() * qd[2];
            const DMat<Scalar> X31_S1 = X31_.transformMotionSubspace(link_1_joint_->S());
            const DVec<Scalar> v3_relative2 = link_3_joint_->S() * qd[2];
            const DMat<Scalar> X32_S2 = X32_.transformMotionSubspace(link_2_joint_->S());

            X_intra_S_span_.template block<6, 1>(6, 0) = X21_S1;
            X_intra_S_span_.template block<6, 1>(12, 0) = X31_S1;
            X_intra_S_span_.template block<6, 1>(12, 1) = X32_S2;

            this->S_.template topLeftCorner<18, 3>() =
                X_intra_S_span_.template topLeftCorner<18, 3>();

            X_intra_S_span_ring_.template block<6, 1>(6, 0) =
                -spatial::generalMotionCrossMatrix(v2_relative1) * X21_S1;
            X_intra_S_span_ring_.template block<6, 1>(12, 0) =
                -spatial::generalMotionCrossMatrix(v3_relative1) * X31_S1;
            X_intra_S_span_ring_.template block<6, 1>(12, 1) =
                -spatial::generalMotionCrossMatrix(v3_relative2) * X32_S2;

            this->vJ_ = X_intra_S_span_ * qd;
            this->cJ_ = X_intra_S_span_ring_ * qd;
        }

        template <typename Scalar>
        void RevoluteTripleWithRotor<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
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

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        RevoluteTripleWithRotor<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> bodies_joints_and_ref_inertias;

            const DMat<Scalar> S_dependent_1 = this->S_.template middleRows<6>(18);
            const Mat6<Scalar> Ir1 = rotor_1_.inertia_.getMatrix();
            const DMat<Scalar> ref_inertia_1 = S_dependent_1.transpose() * Ir1 * S_dependent_1;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link_1_, link_1_joint_,
                                                                     ref_inertia_1));

            const DMat<Scalar> S_dependent_2 = this->S_.template middleRows<6>(24);
            const Mat6<Scalar> Ir2 = rotor_2_.inertia_.getMatrix();
            const DMat<Scalar> ref_inertia_2 = S_dependent_2.transpose() * Ir2 * S_dependent_2;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link_2_, link_2_joint_,
                                                                     ref_inertia_2));

            const DMat<Scalar> S_dependent_3 = this->S_.template middleRows<6>(30);
            const Mat6<Scalar> Ir3 = rotor_3_.inertia_.getMatrix();
            const DMat<Scalar> ref_inertia_3 = S_dependent_3.transpose() * Ir3 * S_dependent_3;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link_3_, link_3_joint_,
                                                                     ref_inertia_3));

            return bodies_joints_and_ref_inertias;
        }

        template class RevoluteTripleWithRotor<double>;
        template class RevoluteTripleWithRotor<casadi::SX>;

    }

} // namespace grbda
