#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_TRIPLE_WITH_ROTOR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_TRIPLE_WITH_ROTOR_JOINT_H

#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class RevoluteTripleWithRotor : public Base<Scalar>
        {
        public:
            typedef ParallelBeltTransmissionModule<1, Scalar> ProximalTransmission;
            typedef ParallelBeltTransmissionModule<2, Scalar> IntermediateTransmission;
            typedef ParallelBeltTransmissionModule<3, Scalar> DistalTransmission;


            RevoluteTripleWithRotor(const ProximalTransmission& module_1,
                                    const IntermediateTransmission& module_2,
                                    const DistalTransmission& module_3);
            virtual ~RevoluteTripleWithRotor() {}

            ClusterJointTypes type() const override
            {
                return ClusterJointTypes::RevoluteTripleWithRotor;
            }

            void updateKinematics(const JointState<Scalar> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const override;

            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr<Scalar> link_1_joint_;
            JointPtr<Scalar> link_2_joint_;
            JointPtr<Scalar> link_3_joint_;
            JointPtr<Scalar> rotor_1_joint_;
            JointPtr<Scalar> rotor_2_joint_;
            JointPtr<Scalar> rotor_3_joint_;

            spatial::Transform<Scalar> X21_;
            spatial::Transform<Scalar> X32_;
            spatial::Transform<Scalar> X31_;

            const Body<Scalar> link_1_;
            const Body<Scalar> link_2_;
            const Body<Scalar> link_3_;
            const Body<Scalar> rotor_1_;
            const Body<Scalar> rotor_2_;
            const Body<Scalar> rotor_3_;

            DMat<Scalar> X_intra_S_span_;
            DMat<Scalar> X_intra_S_span_ring_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_TRIPLE_WITH_ROTOR_JOINT_H
