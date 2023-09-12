#ifndef GRBDA_GENERALIZED_JOINT_GENERIC_H
#define GRBDA_GENERALIZED_JOINT_GENERIC_H

#include "GeneralizedJoint.h"

namespace grbda
{
    // TODO(@MatthewChignoli): This class is not unit tested at the moment

    // TODO(@MatthewChignoli): Add a unit test that compares generic joint versions to specialized versions. For example, compare revolute pair with rotor as a generic joint to the specialized joint class.

    namespace GeneralizedJoints
    {

        class Generic : public Base
        {
        public:
            Generic(const std::vector<Body> &bodies, const std::vector<JointPtr> &joints,
                    shared_ptr<const LoopConstraint::Base> loop_constraint);

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
