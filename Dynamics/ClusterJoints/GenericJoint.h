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
            Generic(const std::vector<Body> &bodies, const std::vector<JointPtr> &joints,
                    std::shared_ptr<LoopConstraint::Base> loop_constraint);

            ClusterJointTypes type() const override { return ClusterJointTypes::Generic; }

            void updateKinematics(const JointState<> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<> &Xup) const override;

        private:
            void extractConnectivity();

            bool bodyInCurrentCluster(const int body_index) const;
            const Body &getBody(const int body_index) const;

            const std::vector<Body> bodies_;

            DMat<double> S_spanning_;
            DMat<double> X_intra_;
            DMat<double> X_intra_ring_;
            DMat<bool> connectivity_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_GENERIC_H
