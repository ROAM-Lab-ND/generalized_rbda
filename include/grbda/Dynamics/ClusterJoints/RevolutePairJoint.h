#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_JOINT_H

#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class RevolutePair : public Generic<Scalar>
        {
        public:
            RevolutePair(Body<Scalar> &link_1, Body<Scalar> &link_2,
                         ori::CoordinateAxis joint_axis_1, ori::CoordinateAxis joint_axis_2);
            virtual ~RevolutePair() {}

            ClusterJointTypes type() const override { return ClusterJointTypes::RevolutePair; }

            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            const Body<Scalar> link_1_;
            const Body<Scalar> link_2_;

            JointPtr<Scalar> link_1_joint_;
            JointPtr<Scalar> link_2_joint_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_JOINT_H
