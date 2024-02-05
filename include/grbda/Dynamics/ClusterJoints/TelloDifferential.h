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

#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"
#include "grbda/Codegen/CasadiGen.h"

namespace grbda
{

    namespace LoopConstraint
    {
        template <typename Scalar = double>
        struct TelloDifferential : Base<Scalar>
        {
            TelloDifferential(const CasadiHelperFunctions<Scalar> &jacobian_helpers,
                              const CasadiHelperFunctions<Scalar> &bias_helpers,
                              const CasadiHelperFunctions<Scalar> &IK_pos_helpers,
                              const CasadiHelperFunctions<Scalar> &IK_vel_helpers);

            std::shared_ptr<Base<Scalar>> clone() const override;

            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override;
            void updateBiases(const JointState<Scalar> &joint_state) override;

            const CasadiHelperFunctions<Scalar> jacobian_helpers_;
            const CasadiHelperFunctions<Scalar> bias_helpers_;
            const CasadiHelperFunctions<Scalar> IK_pos_helpers_;
            const CasadiHelperFunctions<Scalar> IK_vel_helpers_;
        };
    }

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class TelloDifferential : public Base<Scalar>
        {
        public:
            TelloDifferential(TelloDifferentialModule<Scalar> &module);
            virtual ~TelloDifferential() {}

            void updateKinematics(const JointState<Scalar> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const override;

            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
            bodiesJointsAndReflectedInertias() const override;

            JointState<double> randomJointState() const override;

        protected:
            std::shared_ptr<LoopConstraint::TelloDifferential<Scalar>> tello_constraint_;

        private:
            JointPtr<Scalar> rotor1_joint_;
            JointPtr<Scalar> rotor2_joint_;
            JointPtr<Scalar> link1_joint_;
            JointPtr<Scalar> link2_joint_;

            spatial::Transform<Scalar> X21_;

            const Body<Scalar> rotor1_;
            const Body<Scalar> rotor2_;
            const Body<Scalar> link1_;
            const Body<Scalar> link2_;

            DMat<Scalar> X_intra_S_span_;
            DMat<Scalar> X_intra_S_span_ring_;

            const Scalar gear_ratio_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_TELLO_DIFFERENTIAL_H
