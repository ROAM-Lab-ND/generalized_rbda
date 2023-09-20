#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_MULTIPLE_ROTORS_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_MULTIPLE_ROTORS_JOINT_H

#include "ClusterJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        class RevoluteWithMultipleRotorsJoint : public Base
        {
        public:
            RevoluteWithMultipleRotorsJoint(Body &link, std::vector<Body> &rotors,
                                            ori::CoordinateAxis joint_axis,
                                            std::vector<ori::CoordinateAxis> &rotor_axes,
                                            std::vector<double> &gear_ratios);
            virtual ~RevoluteWithMultipleRotorsJoint() {}

            ClusterJointTypes type() const override
            {
                return ClusterJointTypes::RevoluteWithMultipleRotorsJoint;
            }

            void updateKinematics(const JointState &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform &Xup) const override;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr link_joint_;
            std::vector<JointPtr> rotor_joints_;

            const Body link_;
            std::vector<Body> rotors_;

            DMat<double> X_inter_;
            DMat<double> S_spanning_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_MULTIPLE_ROTORS_JOINT_H
