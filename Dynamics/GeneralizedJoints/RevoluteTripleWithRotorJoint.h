#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_TRIPLE_WITH_ROTOR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_TRIPLE_WITH_ROTOR_JOINT_H

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        // TODO(@MatthewChignoli): Use for the RevPairWithRotor? Abstract this joint somehow?
        struct ParallelBeltTransmissionModule
        {
            Body body_;
            Body rotor_;
            CoordinateAxis joint_axis_;
            CoordinateAxis rotor_axis_;
            double gear_ratio_;
            double belt_ratio_;
        };

        class RevoluteTripleWithRotor : public Base
        {
        public:
            RevoluteTripleWithRotor(std::vector<ParallelBeltTransmissionModule> modules);
            virtual ~RevoluteTripleWithRotor() {}

            GeneralizedJointTypes type() const override
            {
                return GeneralizedJointTypes::RevoluteTripleWithRotor;
            }

            void updateKinematics(const JointState &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const override;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr link_1_joint_;
            JointPtr link_2_joint_;
            JointPtr link_3_joint_;
            JointPtr rotor_1_joint_;
            JointPtr rotor_2_joint_;
            JointPtr rotor_3_joint_;

            SpatialTransform X21_;
            SpatialTransform X32_;
            SpatialTransform X31_;

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
