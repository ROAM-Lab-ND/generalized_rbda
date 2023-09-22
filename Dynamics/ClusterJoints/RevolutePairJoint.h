#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_JOINT_H

#include "ClusterJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        class RevolutePair : public Base<>
        {
        public:
            RevolutePair(Body &link_1, Body &link_2,
                         ori::CoordinateAxis joint_axis_1, ori::CoordinateAxis joint_axis_2);
            virtual ~RevolutePair() {}

            ClusterJointTypes type() const override { return ClusterJointTypes::RevolutePair; }

            void updateKinematics(const JointState<> &joint_state) override;
                                  
            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<> &Xup) const override;

            std::vector<std::tuple<Body, JointPtr, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr link_1_joint_;
            JointPtr link_2_joint_;

            spatial::Transform<> X21_;

            const Body link_1_;
            const Body link_2_;

            DMat<double> X_intra_S_span_;
            DMat<double> X_intra_S_span_ring_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_JOINT_H
