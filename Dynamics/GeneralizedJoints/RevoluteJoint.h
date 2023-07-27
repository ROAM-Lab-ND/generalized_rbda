#pragma once

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class Revolute : public Base<Revolute>
        {
        public:
            Revolute(const Body &body, CoordinateAxis joint_axis);
            virtual ~Revolute() {}

            GeneralizedJointTypes type() const { return GeneralizedJointTypes::Revolute; }

            void updateKinematics(const JointState &joint_state);

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const;

        private:
            const Body body_;
        };

    }
    
} // namespace grbda
