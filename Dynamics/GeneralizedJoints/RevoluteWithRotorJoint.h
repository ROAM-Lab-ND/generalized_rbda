#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_ROTOR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_ROTOR_JOINT_H

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class RevoluteWithRotor : public Base
        {
        public:
            RevoluteWithRotor(Body &link, Body &rotor, CoordinateAxis joint_axis,
                              CoordinateAxis rotor_axis, double gear_ratio);
            virtual ~RevoluteWithRotor() {}

            GeneralizedJointTypes type() const override { return GeneralizedJointTypes::RevoluteWithRotor; }

            void updateKinematics(const JointState &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const override;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr link_joint_;
            JointPtr rotor_joint_;

            const Body link_;
            const Body rotor_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_ROTOR_JOINT_H
