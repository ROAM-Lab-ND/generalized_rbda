#include "RevoluteJoint.h"

namespace grbda
{
    namespace ClusterJoints
    {

        template <typename Scalar>
        Revolute<Scalar>::Revolute(const Body<Scalar> &body, ori::CoordinateAxis joint_axis)
            : Base<Scalar>(1, 1, 1), body_(body)
        {
            this->single_joints_.emplace_back(new Joints::Revolute<Scalar>(joint_axis));

            this->S_ = this->single_joints_[0]->S();
            this->Psi_ = this->single_joints_[0]->S();

            this->spanning_tree_to_independent_coords_conversion_ = DMat<int>::Identity(1, 1);

            const DMat<Scalar> G = DMat<Scalar>::Identity(1, 1);
            const DMat<Scalar> K = DMat<Scalar>::Identity(0, 1);
            this->loop_constraint_ = std::make_shared<LoopConstraint::Static<Scalar>>(G, K);
        }

        template <typename Scalar>
        void Revolute<Scalar>::updateKinematics(const JointState<Scalar> &joint_state)
        {
            this->single_joints_[0]->updateKinematics(joint_state.position, joint_state.velocity);
            this->vJ_ = this->S_ * joint_state.velocity;
        }

        template <typename Scalar>
        void Revolute<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<Scalar> &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 1)
                throw std::runtime_error("[Revolute Joint] Xup must have 6 rows");
#endif
            Xup[0] = this->single_joints_[0]->XJ() * body_.Xtree_;
        }

        template <typename Scalar>
        std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
        Revolute<Scalar>::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>> bodies_joints_and_reflected_inertias_;
            bodies_joints_and_reflected_inertias_.push_back(
                std::make_tuple(body_, this->single_joints_[0], DMat<Scalar>::Zero(1, 1)));
            return bodies_joints_and_reflected_inertias_;
        }

        template class Revolute<double>;
        template class Revolute<float>;
        template class Revolute<casadi::SX>;

    }

} // namespace grbda
