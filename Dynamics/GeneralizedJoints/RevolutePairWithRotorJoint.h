#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_WITH_ROTOR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_WITH_ROTOR_JOINT_H

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class RevolutePairWithRotor : public Base
        {
        public:
            RevolutePairWithRotor(ParallelBeltTransmissionModule &module_1,
                                  ParallelBeltTransmissionModule &module_2);
            virtual ~RevolutePairWithRotor() {}

            GeneralizedJointTypes type() const override
            {
                return GeneralizedJointTypes::RevolutePairWithRotor;
            }

            void updateKinematics(const JointState &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const override;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr link1_joint_;
            JointPtr rotor1_joint_;
            JointPtr rotor2_joint_;
            JointPtr link2_joint_;

            SpatialTransform X21_;

            const Body link1_;
            const Body link2_;
            const Body rotor1_;
            const Body rotor2_;

            DMat<double> X_inter_S_span_;
            DMat<double> X_inter_S_span_ring_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_WITH_ROTOR_JOINT_H
