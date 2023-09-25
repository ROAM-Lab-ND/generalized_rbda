/**
 * @file TelloDifferential.h
 *
 * @brief Implementation of generalized joint for differential
 * mechanism (hip differential and knee-ankle differential)
 * found in Tello.
 *
 * Maximal coordinate consists of the spanning tree coordinates:
 * - 2x pre-gearbox rotor coordinates (independent coordinates)
 * - 2x joint link coordinate (dependent coordinates)
 *
 * Minimal coordinate consists of:
 * - 2x post-gearbox rotor coordinates
 */

#ifndef GRBDA_GENERALIZED_JOINTS_TELLO_DIFFERENTIAL_H
#define GRBDA_GENERALIZED_JOINTS_TELLO_DIFFERENTIAL_H

#include "ClusterJoint.h"
#include "Utils/CasadiGen/header/CasadiGen.h"

namespace grbda
{

    namespace LoopConstraint
    {
        struct TelloDifferential : Base
        {
            TelloDifferential(const CasadiHelperFunctions &jacobian_helpers,
                              const CasadiHelperFunctions &bias_helpers,
                              const CasadiHelperFunctions &IK_pos_helpers,
                              const CasadiHelperFunctions &IK_vel_helpers);

            std::shared_ptr<Base> clone() const override;

            DVec<double> gamma(const JointCoordinate<> &joint_pos) const override;

            void updateJacobians(const JointCoordinate<> &joint_pos) override;
            void updateBiases(const JointState<> &joint_state) override;

            const CasadiHelperFunctions jacobian_helpers_;
            const CasadiHelperFunctions bias_helpers_;
            const CasadiHelperFunctions IK_pos_helpers_;
            const CasadiHelperFunctions IK_vel_helpers_;
        };
    }

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class TelloDifferential : public Base<Scalar>
        {
        public:
            TelloDifferential(TelloDifferentialModule &module);
            virtual ~TelloDifferential() {}

            void updateKinematics(const JointState<> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<> &Xup) const override;

            std::vector<std::tuple<Body, JointPtr<double>, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

            JointState<> randomJointState() const override;

        protected:
            std::shared_ptr<LoopConstraint::TelloDifferential> tello_constraint_;

        private:
            JointPtr<Scalar> rotor1_joint_;
            JointPtr<Scalar> rotor2_joint_;
            JointPtr<Scalar> link1_joint_;
            JointPtr<Scalar> link2_joint_;

            spatial::Transform<> X21_;

            const Body rotor1_;
            const Body rotor2_;
            const Body link1_;
            const Body link2_;

            DMat<double> X_intra_S_span_;
            DMat<double> X_intra_S_span_ring_;

            const double gear_ratio_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_TELLO_DIFFERENTIAL_H
