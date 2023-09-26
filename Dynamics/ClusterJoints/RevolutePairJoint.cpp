#include "RevolutePairJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar>
        RevolutePair<Scalar>::RevolutePair(Body<Scalar> &link_1, Body<Scalar> &link_2,
                                           ori::CoordinateAxis joint_axis_1,
                                           ori::CoordinateAxis joint_axis_2)
            : Base<Scalar>(2, 2, 2), link_1_(link_1), link_2_(link_2)
        {
            using Rev = Joints::Revolute<Scalar>;
            link_1_joint_ =  this->single_joints_.emplace_back(new Rev(joint_axis_1));
            link_2_joint_ =  this->single_joints_.emplace_back(new Rev(joint_axis_2));

            this->spanning_tree_to_independent_coords_conversion_ = DMat<Scalar>::Zero(0, 0);

            DMat<Scalar> G = DMat<Scalar>::Zero(2, 2);
            G << 1., 0.,
                0., 1.;
            const DMat<Scalar> K = DMat<Scalar>::Identity(0, 2);
            this->loop_constraint_ = std::make_shared<LoopConstraint::Static<Scalar>>(G, K);

            X_intra_S_span_ = DMat<Scalar>::Zero(12, 2);
            X_intra_S_span_ring_ = DMat<Scalar>::Zero(12, 2);

            X_intra_S_span_.template block<6, 1>(0, 0) = link_1_joint_->S();
            X_intra_S_span_.template block<6, 1>(6, 1) = link_2_joint_->S();

            this->S_ = X_intra_S_span_ * this->loop_constraint_->G();
        }

        template <typename Scalar>
        void RevolutePair<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<Scalar> &q = spanning_joint_state.position;
            const DVec<Scalar> &qd = spanning_joint_state.velocity;

            link_1_joint_->updateKinematics(q.template segment<1>(0), qd.template segment<1>(0));
            link_2_joint_->updateKinematics(q.template segment<1>(1), qd.template segment<1>(1));

            X21_ = link_2_joint_->XJ() * link_2_.Xtree_;
            const DVec<Scalar> v2_relative = link_2_joint_->S() * qd[1];
            X_intra_S_span_.template block<6, 1>(6, 0) =
                X21_.transformMotionSubspace(link_1_joint_->S());
            this->S_.template block<6, 1>(6, 0) = X21_.transformMotionSubspace(link_1_joint_->S());

            X_intra_S_span_ring_.template block<6, 1>(6, 0) =
                -spatial::generalMotionCrossMatrix(v2_relative) *
                X_intra_S_span_.template block<6, 1>(6, 0);

            this->vJ_ = X_intra_S_span_ * qd;
            this->cJ_ = X_intra_S_span_ring_ * qd;
        }

        template <typename Scalar>
        void RevolutePair<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 2)
                throw std::runtime_error("[RevolutePair] Xup must have 12 rows");
#endif

            Xup[0] = link_1_joint_->XJ() * link_1_.Xtree_;
            Xup[1] = link_2_joint_->XJ() * link_2_.Xtree_ * Xup[0];
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        RevolutePair<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> bodies_joints_and_reflected_inertias;

            const DMat<Scalar> reflected_inertia_1 =
                DMat<Scalar>::Zero(link_1_joint_->numVelocities(), link_1_joint_->numVelocities());
            bodies_joints_and_reflected_inertias.push_back(
                std::make_tuple(link_1_, link_1_joint_, reflected_inertia_1));

            const DMat<Scalar> reflected_inertia_2 =
                DMat<Scalar>::Zero(link_2_joint_->numVelocities(), link_2_joint_->numVelocities());
            bodies_joints_and_reflected_inertias.push_back(
                std::make_tuple(link_2_, link_2_joint_, reflected_inertia_2));

            return bodies_joints_and_reflected_inertias;
        }

        template class RevolutePair<double>;
        template class RevolutePair<casadi::SX>;
    }

} // namespace grbda
