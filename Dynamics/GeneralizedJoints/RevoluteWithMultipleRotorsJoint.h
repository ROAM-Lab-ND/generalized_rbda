#pragma once

#include "GeneralizedJoint.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class RevoluteWithMultipleRotorsJoint : public Base
        {
        public:
            RevoluteWithMultipleRotorsJoint(Body &link, std::vector<Body> &rotors,
                                            CoordinateAxis joint_axis,
                                            std::vector<CoordinateAxis> &rotor_axes,
                                            std::vector<double> &gear_ratios);
            virtual ~RevoluteWithMultipleRotorsJoint() {}

            GeneralizedJointTypes type() const override
            {
                return GeneralizedJointTypes::RevoluteWithMultipleRotorsJoint;
            }

            void updateKinematics(const State<double> &joint_pos,
                                  const State<double> &joint_vel) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                GeneralizedSpatialTransform &Xup) const override;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr link_joint_;
            std::vector<JointPtr> rotor_joints_;

            const Body link_;
            std::vector<Body> rotors_;
        };

    }

} // namespace grbda
