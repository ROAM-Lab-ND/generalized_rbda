#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_ROTOR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_ROTOR_JOINT_H

#include "ClusterJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class RevoluteWithRotor : public Base<Scalar>
        {
        public:
            RevoluteWithRotor(GearedTransmissionModule &module);

            virtual ~RevoluteWithRotor() {}

            ClusterJointTypes type() const override { return ClusterJointTypes::RevoluteWithRotor; }

            void updateKinematics(const JointState<> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<> &Xup) const override;

            std::vector<std::tuple<Body<>, JointPtr<double>, DMat<double>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr<Scalar> link_joint_;
            JointPtr<Scalar> rotor_joint_;

            const Body<> link_;
            const Body<> rotor_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_ROTOR_JOINT_H
