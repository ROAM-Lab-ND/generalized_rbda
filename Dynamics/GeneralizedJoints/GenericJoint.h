#ifndef GRBDA_GENERALIZED_JOINT_GENERIC_H
#define GRBDA_GENERALIZED_JOINT_GENERIC_H

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class Generic : public Base
        {
        public:
            Generic(const std::vector<Body> &bodies, const std::vector<JointPtr> &joints,
                    shared_ptr<LoopConstraint::Base> loop_constraint);

            GeneralizedJointTypes type() const override { return GeneralizedJointTypes::Generic; }

            void updateKinematics(const JointState &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const override;

        private:
            void extractConnectivity();

            bool bodyInCurrentCluster(const int body_index) const;
            const Body &getBody(const int body_index) const;

            const std::vector<Body> bodies_;

            DMat<double> S_spanning_tree_;
            DMat<double> Xup_spanning_tree_;
            DMat<double> Xup_ring_spanning_tree_;
            DMat<bool> connectivity_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINT_GENERIC_H
