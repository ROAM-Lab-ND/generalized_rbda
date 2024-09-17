#include "grbda/Dynamics/ClusterJoints/RevoluteWithRotorJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {
        template <typename Scalar>
        RevoluteWithRotor<Scalar>::RevoluteWithRotor(GearedTransmissionModule<Scalar> &module)
            : Base<Scalar>(2, 1, 1), link_(module.body_), rotor_(module.rotor_)
        {
            using Rev = Joints::Revolute<Scalar>;
            link_joint_ = this->single_joints_.emplace_back(new Rev(module.joint_axis_,
                                                                    module.body_joint_name_));
            rotor_joint_ = this->single_joints_.emplace_back(new Rev(module.rotor_axis_,
                                                                     module.rotor_joint_name_));

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Zero(1, 2);
            this->spanning_tree_to_independent_coords_conversion_ << 1, 0;

            DMat<Scalar> G = DMat<Scalar>::Zero(2, 1);
            G << 1., module.gear_ratio_;
            DMat<Scalar> K = DMat<Scalar>::Zero(1, 2);
            K << module.gear_ratio_, -1.;
            this->loop_constraint_ = std::make_shared<LoopConstraint::Static<Scalar>>(G, K);

            this->S_.template block<6, 1>(0, 0) = link_joint_->S();
            this->S_.template block<6, 1>(6, 0) = module.gear_ratio_ * rotor_joint_->S();

            this->Psi_.template block<6, 1>(6, 0) = 1. / module.gear_ratio_ * rotor_joint_->S();
        }

        template <typename Scalar>
        void RevoluteWithRotor<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            const JointState<Scalar> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<Scalar> &q = spanning_joint_state.position;
            const DVec<Scalar> &qd = spanning_joint_state.velocity;

            link_joint_->updateKinematics(q.template segment<1>(0), qd.template segment<1>(0));
            rotor_joint_->updateKinematics(q.template segment<1>(1), qd.template segment<1>(1));

            this->vJ_.template head<6>() = link_joint_->S() * qd[0];
            this->vJ_.template tail<6>() = rotor_joint_->S() * qd[1];
        }

        template <typename Scalar>
        void RevoluteWithRotor<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 2)
                throw std::runtime_error("[Revolute+Rotor Joint] Xup must have 12 rows");
#endif

            Xup[0] = link_joint_->XJ() * link_.Xtree_;
            Xup[1] = rotor_joint_->XJ() * rotor_.Xtree_;
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        RevoluteWithRotor<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> bodies_joints_and_reflected_inertias;

            DMat<Scalar> S_dependent = this->S_.template bottomRows<6>();
            DMat<Scalar> reflected_inertia =
                S_dependent.transpose() * rotor_.inertia_.getMatrix() * S_dependent;

            bodies_joints_and_reflected_inertias.push_back(
                std::make_tuple(link_, link_joint_, reflected_inertia));

            return bodies_joints_and_reflected_inertias;
        }

        template class RevoluteWithRotor<double>;
        template class RevoluteWithRotor<float>;
        template class RevoluteWithRotor<casadi::SX>;
    }

} // namespace grbda
