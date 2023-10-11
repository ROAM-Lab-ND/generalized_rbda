#ifndef GRBDA_GENERALIZED_JOINTS_FREE_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_FREE_JOINT_H

#include "ClusterJoint.h"
#include "Joints/OrientationRepresentation.h"

namespace grbda
{

    namespace LoopConstraint
    {
        template <typename Scalar = double, typename OrientationRepresentation = ori_representation::QuaternionRepresentation< Quat<Scalar> >>
        struct Free : Base<Scalar>
        {
            Free();

            int numSpanningPos() const override { return OrientationRepresentation::numSpanningPos; }
            int numIndependentPos() const override { return OrientationRepresentation::numIndependentPos; }

            std::shared_ptr<Base<Scalar>> clone() const override;

            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override {}
            void updateBiases(const JointState<Scalar> &joint_state) override {}

            DVec<Scalar> gamma(const JointCoordinate<Scalar> &joint_pos) const override;
        };

    }

    namespace ClusterJoints
    {

        template <typename Scalar = double, typename OrientationRepresentation = ori_representation::QuaternionRepresentation< Quat<Scalar> >>
        class Free : public Base<Scalar>
        {
        public:
            Free(const Body<Scalar> &body);
            virtual ~Free() {}

            ClusterJointTypes type() const override { return ClusterJointTypes::Free; }

            int numUnactuatedVelocities() const override { return 6; }

            void updateKinematics(const JointState<Scalar> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const override;

            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
            bodiesJointsAndReflectedInertias() const override;

            JointState<Scalar> randomJointState() const override;

        private:
            const Body<Scalar> body_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_FREE_JOINT_H
