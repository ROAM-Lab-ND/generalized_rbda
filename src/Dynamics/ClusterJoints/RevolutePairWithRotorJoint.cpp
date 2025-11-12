#include "grbda/Dynamics/ClusterJoints/RevolutePairWithRotorJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar>
        RevolutePairWithRotor<Scalar>::RevolutePairWithRotor(
            ProximalTransmission &module_1, DistalTransmission &module_2)
            : Base<Scalar>(4, 2, 2),
              link1_(module_1.body_), link2_(module_2.body_),
              rotor1_(module_1.rotor_), rotor2_(module_2.rotor_),
              link1_index_(link1_.sub_index_within_cluster_),
              link2_index_(link2_.sub_index_within_cluster_),
              rotor1_index_(rotor1_.sub_index_within_cluster_),
              rotor2_index_(rotor2_.sub_index_within_cluster_)
        {
            using Rev = Joints::Revolute<Scalar>;

            link1_joint_ = this->single_joints_.emplace_back(new Rev(module_1.joint_axis_));
            rotor1_joint_ = this->single_joints_.emplace_back(new Rev(module_1.rotor_axis_));
            rotor2_joint_ = this->single_joints_.emplace_back(new Rev(module_2.rotor_axis_));
            link2_joint_ = this->single_joints_.emplace_back(new Rev(module_2.joint_axis_));

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Zero(2, 4);
            this->spanning_tree_to_independent_coords_conversion_(0, link1_index_) = 1;
            this->spanning_tree_to_independent_coords_conversion_(1, link2_index_) = 1;

            Vec2<Scalar> gear_ratios{module_1.gear_ratio_, module_2.gear_ratio_};
            Eigen::DiagonalMatrix<Scalar, 2> rotor_matrix(gear_ratios);
            Mat2<Scalar> belt_matrix;
            belt_matrix << beltMatrixRowFromBeltRatios(module_1.belt_ratios_), 0,
                beltMatrixRowFromBeltRatios(module_2.belt_ratios_);
            Mat2<Scalar> ratio_product = rotor_matrix * belt_matrix;

            DMat<Scalar> G = DMat<Scalar>::Zero(4, 2);
            G(link1_index_, 0) = 1.;
            G(rotor1_index_, 0) = ratio_product(0, 0);
            G(rotor2_index_, 0) = ratio_product(1, 0);
            G(rotor2_index_, 1) = ratio_product(1, 1);
            G(link2_index_, 1) = 1.;

            DMat<Scalar> K = DMat<Scalar>::Zero(2, 4);
            int cnstr1_index = rotor1_index_ > rotor2_index_;
            int cnstr2_index = rotor2_index_ > rotor1_index_;
            K(cnstr1_index, rotor1_index_) = -1.;
            K(cnstr1_index, link1_index_) = G(rotor1_index_, 0);
            K(cnstr2_index, rotor2_index_) = -1.;
            K(cnstr2_index, link1_index_) = G(rotor2_index_, 0);
            K(cnstr2_index, link2_index_) = G(rotor2_index_, 1);

            this->loop_constraint_ = std::make_shared<LoopConstraint::Static<Scalar>>(G, K);

            X_intra_S_span_ = DMat<Scalar>::Zero(24, 4);
            X_intra_S_span_ring_ = DMat<Scalar>::Zero(24, 4);

            const DMat<Scalar> &link1_S = link1_joint_->S();
            X_intra_S_span_.template block<6, 1>(6 * link1_index_, link1_index_) = link1_S;
            const DMat<Scalar> &rotor1_S = rotor1_joint_->S();
            X_intra_S_span_.template block<6, 1>(6 * rotor1_index_, rotor1_index_) = rotor1_S;
            const DMat<Scalar> &rotor2_S = rotor2_joint_->S();
            X_intra_S_span_.template block<6, 1>(6 * rotor2_index_, rotor2_index_) = rotor2_S;
            const DMat<Scalar> &link2_S = link2_joint_->S();
            X_intra_S_span_.template block<6, 1>(6 * link2_index_, link2_index_) = link2_S;

            this->S_ = X_intra_S_span_ * this->loop_constraint_->G();
        }

        template <typename Scalar>
        void RevolutePairWithRotor<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<Scalar> &q = spanning_joint_state.position;
            const DVec<Scalar> &qd = spanning_joint_state.velocity;

            link1_joint_->updateKinematics(q.template segment<1>(link1_index_),
                                           qd.template segment<1>(link1_index_));
            rotor1_joint_->updateKinematics(q.template segment<1>(rotor1_index_),
                                            qd.template segment<1>(rotor1_index_));
            rotor2_joint_->updateKinematics(q.template segment<1>(rotor2_index_),
                                            qd.template segment<1>(rotor2_index_));
            link2_joint_->updateKinematics(q.template segment<1>(link2_index_),
                                           qd.template segment<1>(link2_index_));

            X21_ = link2_joint_->XJ() * link2_.Xtree_;
            const DVec<Scalar> v2_relative = link2_joint_->S() * qd[link2_index_];

            X_intra_S_span_.template block<6, 1>(6 * link2_index_, link1_index_) =
                X21_.transformMotionSubspace(link1_joint_->S());
            this->S_.template block<6, 1>(6 * link2_index_, 0) =
                X_intra_S_span_.template block<6, 1>(6 * link2_index_, link1_index_);

            X_intra_S_span_ring_.template block<6, 1>(6 * link2_index_, link1_index_) =
                -spatial::generalMotionCrossMatrix(v2_relative) *
                X_intra_S_span_.template block<6, 1>(6 * link2_index_, link1_index_);

            this->vJ_ = X_intra_S_span_ * qd;
            this->cJ_ = X_intra_S_span_ring_ * qd;
        }

        template <typename Scalar>
        void RevolutePairWithRotor<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 4)
                throw std::runtime_error("[RevolutePairWithRotor] Xup must have 24 rows");
#endif

            Xup[link1_index_] = link1_joint_->XJ() * link1_.Xtree_;
            Xup[rotor1_index_] = rotor1_joint_->XJ() * rotor1_.Xtree_;
            Xup[rotor2_index_] = rotor2_joint_->XJ() * rotor2_.Xtree_;
            Xup[link2_index_] = link2_joint_->XJ() * link2_.Xtree_ * Xup[link1_index_];
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        RevolutePairWithRotor<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> bodies_joints_and_ref_inertias;

            DMat<Scalar> S_dependent_1 = this->S_.template middleRows<6>(6 * rotor1_index_);
            Mat6<Scalar> Ir1 = rotor1_.inertia_.getMatrix();
            DMat<Scalar> ref_inertia_1 = S_dependent_1.transpose() * Ir1 * S_dependent_1;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link1_, link1_joint_,
                                                                      ref_inertia_1));

            DMat<Scalar> S_dependent_2 = this->S_.template middleRows<6>(6 * rotor2_index_);
            Mat6<Scalar> Ir2 = rotor2_.inertia_.getMatrix();
            DMat<Scalar> ref_inertia_2 = S_dependent_2.transpose() * Ir2 * S_dependent_2;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link2_, link2_joint_,
                                                                      ref_inertia_2));

            return bodies_joints_and_ref_inertias;
        }

        template class RevolutePairWithRotor<double>;
template class RevolutePairWithRotor<std::complex<double>>;
        template class RevolutePairWithRotor<float>;
        template class RevolutePairWithRotor<casadi::SX>;

    }

} // namespace grbda
