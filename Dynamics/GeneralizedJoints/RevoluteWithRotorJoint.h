#pragma once

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class RevoluteWithRotor : public Base<RevoluteWithRotor>
        {
        public:
            RevoluteWithRotor(Body &link, Body &rotor, CoordinateAxis joint_axis,
                              CoordinateAxis rotor_axis, double gear_ratio);
            virtual ~RevoluteWithRotor() {}

            GeneralizedJointTypes type() const { return GeneralizedJointTypes::RevoluteWithRotor; }

            void updateKinematics(const JointState &joint_state);

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const;

        private:
            JointPtr link_joint_;
            JointPtr rotor_joint_;

            const Body link_;
            const Body rotor_;
        };

    }

} // namespace grbda
