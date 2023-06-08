#pragma once

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class RevoluteWithRotor : public Base
        {
        public:
            RevoluteWithRotor(Body &link, Body &rotor, CoordinateAxis joint_axis,
                              CoordinateAxis rotor_axis, double gear_ratio);
            virtual ~RevoluteWithRotor() {}

            GeneralizedJointTypes type() const override { return GeneralizedJointTypes::RevoluteWithRotor; }

            void updateKinematics(const DVec<double> &y, const DVec<double> &yd) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const override;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr link_joint_;
            JointPtr rotor_joint_;

            const Body link_;
            const Body rotor_;
        };

    }

} // namespace grbda
