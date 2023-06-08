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

            void updateKinematics(const DVec<double> &y, const DVec<double> &yd) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const override;
        };

    }

} // namespace grbda
