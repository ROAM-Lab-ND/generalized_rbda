#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_TRIPLE_WITH_ROTOR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_TRIPLE_WITH_ROTOR_JOINT_H

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class RevoluteTripleWithRotor : public Base
        {
        public:
            RevoluteTripleWithRotor(const ParallelBeltTransmissionModule& module_1,
                                    const ParallelBeltTransmissionModule& module_2,
                                    const ParallelBeltTransmissionModule& module_3);
            virtual ~RevoluteTripleWithRotor() {}

            GeneralizedJointTypes type() const override
            {
                return GeneralizedJointTypes::RevoluteTripleWithRotor;
            }

            void updateKinematics(const JointState &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform &Xup) const override;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr link_1_joint_;
            JointPtr link_2_joint_;
            JointPtr link_3_joint_;
            JointPtr rotor_1_joint_;
            JointPtr rotor_2_joint_;
            JointPtr rotor_3_joint_;

            spatial::Transform X21_;
            spatial::Transform X32_;
            spatial::Transform X31_;

            const Body link_1_;
            const Body link_2_;
            const Body link_3_;
            const Body rotor_1_;
            const Body rotor_2_;
            const Body rotor_3_;

            DMat<double> X_inter_S_span_;
            DMat<double> X_inter_S_span_ring_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_TRIPLE_WITH_ROTOR_JOINT_H
