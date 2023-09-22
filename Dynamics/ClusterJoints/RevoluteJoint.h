#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_JOINT_H

#include "ClusterJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class Revolute : public Base<Scalar>
        {
        public:
            Revolute(const Body &body, ori::CoordinateAxis joint_axis);
            virtual ~Revolute() {}

            ClusterJointTypes type() const override { return ClusterJointTypes::Revolute; }

            void updateKinematics(const JointState<> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<> &Xup) const override;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            const Body body_;
        };

    }
    
} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_JOINT_H
