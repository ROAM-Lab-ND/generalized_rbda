#ifndef GRBDA_GENERALIZED_JOINT_GENERIC_H
#define GRBDA_GENERALIZED_JOINT_GENERIC_H

#include "ClusterJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class Generic : public Base<Scalar>
        {
        public:
            Generic(const std::vector<Body<Scalar>> &bodies,
                    const std::vector<JointPtr<Scalar>> &joints,
                    std::shared_ptr<LoopConstraint::Base<Scalar>> loop_constraint);

            ClusterJointTypes type() const override { return ClusterJointTypes::Generic; }

            void updateKinematics(const JointState<Scalar> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const override;

        private:
            void extractConnectivity();

            bool bodyInCurrentCluster(const int body_index) const;
            const Body<Scalar> &getBody(const int body_index) const;

            const std::vector<Body<Scalar>> bodies_;

            DMat<Scalar> S_spanning_;
            DMat<Scalar> X_intra_;
            DMat<Scalar> X_intra_ring_;
            DMat<bool> connectivity_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_GENERIC_H
