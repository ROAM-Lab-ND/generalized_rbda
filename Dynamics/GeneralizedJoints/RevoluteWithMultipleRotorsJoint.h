#pragma once

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class RevoluteWithMultipleRotorsJoint : public Base<RevoluteWithMultipleRotorsJoint>
        {
        public:
            RevoluteWithMultipleRotorsJoint(Body &link, std::vector<Body> &rotors,
                                            CoordinateAxis joint_axis,
                                            std::vector<CoordinateAxis> &rotor_axes,
                                            std::vector<double> &gear_ratios);
            virtual ~RevoluteWithMultipleRotorsJoint() {}

            GeneralizedJointTypes type() const
            {
                return GeneralizedJointTypes::RevoluteWithMultipleRotorsJoint;
            }

            void updateKinematics(const JointState &joint_state);

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const;

        private:
            JointPtr link_joint_;
            std::vector<JointPtr> rotor_joints_;

            const Body link_;
            std::vector<Body> rotors_;

            DMat<double> S_spanning_tree_;
            DMat<double> Xup_spanning_tree_;
        };

    }

} // namespace grbda
