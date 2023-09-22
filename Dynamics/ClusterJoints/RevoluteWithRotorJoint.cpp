#include "RevoluteWithRotorJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {
        RevoluteWithRotor::RevoluteWithRotor(GearedTransmissionModule &module)
        : Base(2, 1, 1), link_(module.body_), rotor_(module.rotor_)
        {
            link_joint_ = 
            this->single_joints_.emplace_back(new Joints::Revolute(module.joint_axis_));
            rotor_joint_ = 
            this->single_joints_.emplace_back(new Joints::Revolute(module.rotor_axis_));

            this->spanning_tree_to_independent_coords_conversion_ = DMat<double>::Zero(1, 2);
            this->spanning_tree_to_independent_coords_conversion_ << 1., 0.;

            DMat<double> G = DMat<double>::Zero(2, 1);
            G << 1., module.gear_ratio_;
            DMat<double> K = DMat<double>::Zero(1, 2);
            K << module.gear_ratio_, -1.;
            this->loop_constraint_ = std::make_shared<LoopConstraint::Static>(G, K);

            this->S_.block<6, 1>(0, 0) = link_joint_->S();
            this->S_.block<6, 1>(6, 0) = module.gear_ratio_ * rotor_joint_->S();

            this->Psi_.block<6, 1>(6, 0) = 1. / module.gear_ratio_ * rotor_joint_->S();
        }

        void RevoluteWithRotor::updateKinematics(const JointState<> &joint_state)
        {
            const JointState<> spanning_joint_state = this->toSpanningTreeState(joint_state);
            const DVec<double> &q = spanning_joint_state.position;
            const DVec<double> &qd = spanning_joint_state.velocity;

            link_joint_->updateKinematics(q.segment<1>(0), qd.segment<1>(0));
            rotor_joint_->updateKinematics(q.segment<1>(1), qd.segment<1>(1));

            this->vJ_.head<6>() = link_joint_->S() * qd[0];
            this->vJ_.tail<6>() = rotor_joint_->S() * qd[1];
        }

        void RevoluteWithRotor::computeSpatialTransformFromParentToCurrentCluster(
            spatial::GeneralizedTransform<> &Xup) const
        {
#ifdef DEBUG_MODE
            if (Xup.getNumOutputBodies() != 2)
                throw std::runtime_error("[Revolute+Rotor Joint] Xup must have 12 rows");
#endif

            Xup[0] = link_joint_->XJ() * link_.Xtree_;
            Xup[1] = rotor_joint_->XJ() * rotor_.Xtree_;
        }

        std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        RevoluteWithRotor::bodiesJointsAndReflectedInertias() const
        {
            std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_reflected_inertias;

            DMat<double> S_dependent = this->S_.bottomRows<6>();
            DMat<double> reflected_inertia =
                S_dependent.transpose() * rotor_.inertia_.getMatrix() * S_dependent;

            bodies_joints_and_reflected_inertias.push_back(
                std::make_tuple(link_, link_joint_, reflected_inertia));

            return bodies_joints_and_reflected_inertias;
        }
    }

} // namespace grbda
