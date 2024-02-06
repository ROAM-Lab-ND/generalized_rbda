#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_ROTOR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_ROTOR_JOINT_H

#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class RevoluteWithRotor : public Explicit<Scalar>
        {
        public:
            RevoluteWithRotor(GearedTransmissionModule<Scalar> &module);

            virtual ~RevoluteWithRotor() {}

            ClusterJointTypes type() const override { return ClusterJointTypes::RevoluteWithRotor; }

            void updateKinematics(const JointState<Scalar> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const override;

            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr<Scalar> link_joint_;
            JointPtr<Scalar> rotor_joint_;

            const Body<Scalar> link_;
            const Body<Scalar> rotor_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_WITH_ROTOR_JOINT_H
