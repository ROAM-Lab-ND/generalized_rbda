#include "RevolutePairWithRotorJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar>
        RevolutePairWithRotor<Scalar>::RevolutePairWithRotor(
            ParallelBeltTransmissionModule<Scalar> &module_1,
            ParallelBeltTransmissionModule<Scalar> &module_2)
            : Base<Scalar>(4, 2, 2), link1_(module_1.body_), link2_(module_2.body_),
              rotor1_(module_1.rotor_), rotor2_(module_2.rotor_)
        {
            const double gear_ratio_2 = module_2.gear_ratio_;
            const double belt_ratio_1 = module_1.belt_ratio_;
            const double net_ratio_1 = module_1.gear_ratio_ * belt_ratio_1;
            const double net_ratio_2 = gear_ratio_2 * module_2.belt_ratio_;

            using Rev = Joints::Revolute<Scalar>;

            link1_joint_ = this->single_joints_.emplace_back(new Rev(module_1.joint_axis_));
            rotor1_joint_ = this->single_joints_.emplace_back(new Rev(module_1.rotor_axis_));
            rotor2_joint_ = this->single_joints_.emplace_back(new Rev(module_2.rotor_axis_));
            link2_joint_ = this->single_joints_.emplace_back(new Rev(module_2.joint_axis_));

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Identity(2, 4);
            this->spanning_tree_to_independent_coords_conversion_ << 1, 0, 0, 0, 0, 0, 0, 1;

            DMat<Scalar> G = DMat<Scalar>::Zero(4, 2);
            G << 1., 0.,
                net_ratio_1, 0.,
                gear_ratio_2 * belt_ratio_1, net_ratio_2,
                0., 1.;
            DMat<Scalar> K = DMat<Scalar>::Identity(2, 4);
            K << net_ratio_1, -1., 0., 0.,
                gear_ratio_2 * belt_ratio_1, 0, -1., net_ratio_2;
            this->loop_constraint_ = std::make_shared<LoopConstraint::Static<Scalar>>(G, K);

            X_intra_S_span_ = DMat<Scalar>::Zero(24, 4);
            X_intra_S_span_ring_ = DMat<Scalar>::Zero(24, 4);

            X_intra_S_span_.template block<6, 1>(0, 0) = link1_joint_->S();
            X_intra_S_span_.template block<6, 1>(6, 1) = rotor1_joint_->S();
            X_intra_S_span_.template block<6, 1>(12, 2) = rotor2_joint_->S();
            X_intra_S_span_.template block<6, 1>(18, 3) = link2_joint_->S();

            this->S_ = X_intra_S_span_ * this->loop_constraint_->G();
        }

        template <typename Scalar>
        void RevolutePairWithRotor<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<Scalar> &q = spanning_joint_state.position;
            const DVec<Scalar> &qd = spanning_joint_state.velocity;

            link1_joint_->updateKinematics(q.template segment<1>(0), qd.template segment<1>(0));
            rotor1_joint_->updateKinematics(q.template segment<1>(1), qd.template segment<1>(1));
            rotor2_joint_->updateKinematics(q.template segment<1>(2), qd.template segment<1>(2));
            link2_joint_->updateKinematics(q.template segment<1>(3), qd.template segment<1>(3));

            X21_ = link2_joint_->XJ() * link2_.Xtree_;
            const DVec<Scalar> v2_relative = link2_joint_->S() * qd[3];

            X_intra_S_span_.template block<6, 1>(18, 0) =
                X21_.transformMotionSubspace(link1_joint_->S());
            this->S_.template block<6, 1>(18, 0) = X_intra_S_span_.template block<6, 1>(18, 0);

            X_intra_S_span_ring_.template block<6, 1>(18, 0) =
                -spatial::generalMotionCrossMatrix(v2_relative) *
                X_intra_S_span_.template block<6, 1>(18, 0);

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

            Xup[0] = link1_joint_->XJ() * link1_.Xtree_;
            Xup[1] = rotor1_joint_->XJ() * rotor1_.Xtree_;
            Xup[2] = rotor2_joint_->XJ() * rotor2_.Xtree_;
            Xup[3] = link2_joint_->XJ() * link2_.Xtree_ * Xup[0];
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        RevolutePairWithRotor<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> bodies_joints_and_ref_inertias;

            DMat<Scalar> S_dependent_1 = this->S_.template middleRows<6>(6);
            Mat6<Scalar> Ir1 = rotor1_.inertia_.getMatrix();
            DMat<Scalar> ref_inertia_1 = S_dependent_1.transpose() * Ir1 * S_dependent_1;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link1_, link1_joint_,
                                                                      ref_inertia_1));

            DMat<Scalar> S_dependent_2 = this->S_.template middleRows<6>(12);
            Mat6<Scalar> Ir2 = rotor2_.inertia_.getMatrix();
            DMat<Scalar> ref_inertia_2 = S_dependent_2.transpose() * Ir2 * S_dependent_2;
            bodies_joints_and_ref_inertias.push_back(std::make_tuple(link2_, link2_joint_,
                                                                      ref_inertia_2));

            return bodies_joints_and_ref_inertias;
        }

        template class RevolutePairWithRotor<double>;
        template class RevolutePairWithRotor<casadi::SX>;

    }

} // namespace grbda
