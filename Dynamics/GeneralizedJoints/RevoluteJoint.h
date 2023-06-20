#pragma once

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class Revolute : public Base
        {
        public:
            Revolute(const Body &body, CoordinateAxis joint_axis);
            virtual ~Revolute() {}

            GeneralizedJointTypes type() const override { return GeneralizedJointTypes::Revolute; }

            void updateKinematics(const JointState &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const override;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

            JointState randomJointState() const override;

        private:
            const Body body_;
        };

    }
    
} // namespace grbda
