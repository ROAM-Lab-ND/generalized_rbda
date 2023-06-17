#pragma once

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class Free : public Base
        {
        public:
            Free(const Body &body);
            virtual ~Free() {}

            GeneralizedJointTypes type() const override { return GeneralizedJointTypes::Free; }

            int numUnactuatedVelocities() const override { return 6; }

            void updateKinematics(const State<double> &joint_pos,
                                  const State<double> &joint_vel) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const override;
        };

    }

} // namespace grbda
