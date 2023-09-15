#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_JOINT_H

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class Revolute : public Base
        {
        public:
            Revolute(const Body &body, ori::CoordinateAxis joint_axis);
            virtual ~Revolute() {}

            GeneralizedJointTypes type() const override { return GeneralizedJointTypes::Revolute; }

            void updateKinematics(const JointState &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform &Xup) const override;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            const Body body_;
        };

    }
    
} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_JOINT_H
