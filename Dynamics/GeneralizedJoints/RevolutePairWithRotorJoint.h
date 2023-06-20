#pragma once

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class RevolutePairWithRotor : public Base
        {
        public:
            RevolutePairWithRotor(Body &link_1, Body &link_2, Body &rotor_1, Body &rotor_2,
                                  CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2,
                                  CoordinateAxis rotor_axis_1, CoordinateAxis rotor_axis_2,
                                  double gear_ratio_1, double gear_ratio_2,
                                  double belt_ratio_1, double belt_ratio_2);
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

            JointState randomJointState() const override;

        private:
            JointPtr link_1_joint_;
            JointPtr rotor_1_joint_;
            JointPtr rotor_2_joint_;
            JointPtr link_2_joint_;

            SpatialTransform X21_;

            const Body link_1_;
            const Body link_2_;
            const Body rotor_1_;
            const Body rotor_2_;
        };

    }

} // namespace grbda
