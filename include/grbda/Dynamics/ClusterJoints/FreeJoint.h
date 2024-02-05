#ifndef GRBDA_GENERALIZED_JOINTS_FREE_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_FREE_JOINT_H

#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"
#include "grbda/Dynamics/Joints/OrientationRepresentation.h"

namespace grbda
{

    namespace LoopConstraint
    {
        template <typename Scalar = double,
                  typename OrientationRepresentation = ori_representation::Quaternion>
        struct Free : Base<Scalar>
        {
            Free()
            {
                this->gamma_ = [](const JointCoordinate<Scalar> &joint_pos)
                {
                    return joint_pos;
                };

                this->G_ = DMat<Scalar>::Identity(6, 6);
                this->g_ = DVec<Scalar>::Zero(6);

                this->K_ = DMat<Scalar>::Zero(0, 6);
                this->k_ = DVec<Scalar>::Zero(0);
            }

            int numSpanningPos() const override
            {
                return OrientationRepresentation::numSpanningPos;
            }

            int numIndependentPos() const override
            {
                return OrientationRepresentation::numIndependentPos;
            }

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<Free<Scalar, OrientationRepresentation>>(*this);
            }

            void updateJacobians(const JointCoordinate<Scalar> &joint_pos) override {}
            void updateBiases(const JointState<Scalar> &joint_state) override {}
        };

    }

    namespace ClusterJoints
    {

        template <typename Scalar = double,
                  typename OrientationRepresentation = ori_representation::Quaternion>
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

            JointState<double> randomJointState() const override;

        private:
            const Body<Scalar> body_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_FREE_JOINT_H
