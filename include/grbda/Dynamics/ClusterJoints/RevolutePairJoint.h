#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_JOINT_H

#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class RevolutePair : public Base<Scalar>
        {
        public:
            RevolutePair(Body<Scalar> &link_1, Body<Scalar> &link_2,
                         ori::CoordinateAxis joint_axis_1, ori::CoordinateAxis joint_axis_2);
            virtual ~RevolutePair() {}

            ClusterJointTypes type() const override { return ClusterJointTypes::RevolutePair; }

            void updateKinematics(const JointState<Scalar> &joint_state) override;
                                  
            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const override;

            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
            bodiesJointsAndReflectedInertias() const override;

            // Derivative methods
            std::vector<DMat<Scalar>> getSq() const override;
            DMat<Scalar> getSdotqd_q() const override;
            DMat<Scalar> getSdotqd_qd() const override;

        private:
            char axisToChar(ori::CoordinateAxis axis) const;
            void initializeCasadiFunctions() const;

            JointPtr<Scalar> link_1_joint_;
            JointPtr<Scalar> link_2_joint_;

            spatial::Transform<Scalar> X21_;

            const Body<Scalar> link_1_;
            const Body<Scalar> link_2_;

            const ori::CoordinateAxis axis1_;
            const ori::CoordinateAxis axis2_;

            DMat<Scalar> X_intra_S_span_;
            DMat<Scalar> X_intra_S_span_ring_;

            // CasADi functions for derivatives
            mutable bool casadi_functions_initialized_ = false;
            mutable casadi::Function f_dS_dq1_;
            mutable casadi::Function f_dS_dq2_;
            mutable casadi::Function f_Sdotqd_q_;
            mutable casadi::Function f_Sdotqd_qd_;

            // Cache for current state
            mutable DVec<Scalar> q_cache_;
            mutable DVec<Scalar> qd_cache_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_JOINT_H
