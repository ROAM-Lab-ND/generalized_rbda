#pragma once

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class RevolutePair : public Base
        {
        public:
            RevolutePair(Body &link_1, Body &link_2,
                         CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2);
            virtual ~RevolutePair() {}

            GeneralizedJointTypes type() const override { return GeneralizedJointTypes::RevolutePair; }

            void updateKinematics(const DVec<double> &y, const DVec<double> &yd) override;
            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const override;

            // ISSUE: #72
            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr link_1_joint_;
            JointPtr link_2_joint_;

            SpatialTransform X21_;

            const Body link_1_;
            const Body link_2_;
        };

    }

} // namespace grbda
