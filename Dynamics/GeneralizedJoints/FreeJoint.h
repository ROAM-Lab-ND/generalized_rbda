#pragma once

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace LoopConstraint
    {
        struct Free : Base
        {
            Free();

            int numSpanningPos() const override { return 7; }
            int numIndependentPos() const override { return 7; }

            std::shared_ptr<Base> clone() const override;

            void updateJacobians(const JointCoordinate &joint_pos) override {}
            void updateBiases(const JointState &joint_state) override {}

            DVec<double> gamma(const JointCoordinate &joint_pos) const override;

        };

    }

    namespace GeneralizedJoints
    {

        class Free : public Base<Free>
        {
        public:
            Free(const Body &body);
            virtual ~Free() {}

            GeneralizedJointTypes type() const { return GeneralizedJointTypes::Free; }

            int numUnactuatedVelocities() const { return 6; }

            void updateKinematics(const JointState &joint_state);

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const;

            JointState randomJointState() const;

        private:
            const Body body_;
        };

    }

} // namespace grbda
