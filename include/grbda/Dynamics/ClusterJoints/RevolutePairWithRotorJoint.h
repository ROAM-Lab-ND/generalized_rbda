#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_WITH_ROTOR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_WITH_ROTOR_JOINT_H

#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class RevolutePairWithRotor : public Base<Scalar>
        {
        public:
            typedef ParallelBeltTransmissionModule<1, Scalar> ProximalTransmission;
            typedef ParallelBeltTransmissionModule<2, Scalar> DistalTransmission;

            RevolutePairWithRotor(ProximalTransmission &module_1, DistalTransmission &module_2);
            virtual ~RevolutePairWithRotor() {}

            ClusterJointTypes type() const override
            {
                return ClusterJointTypes::RevolutePairWithRotor;
            }

            void updateKinematics(const JointState<Scalar> &joint_state) override;

            void computeSpatialTransformFromParentToCurrentCluster(
                spatial::GeneralizedTransform<Scalar> &Xup) const override;

            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            JointPtr<Scalar> link1_joint_;
            JointPtr<Scalar> rotor1_joint_;
            JointPtr<Scalar> rotor2_joint_;
            JointPtr<Scalar> link2_joint_;

            spatial::Transform<Scalar> X21_;

            const Body<Scalar> link1_;
            const Body<Scalar> link2_;
            const Body<Scalar> rotor1_;
            const Body<Scalar> rotor2_;

            const int link1_index_;
            const int link2_index_;
            const int rotor1_index_;
            const int rotor2_index_;

            DMat<Scalar> X_intra_S_span_;
            DMat<Scalar> X_intra_S_span_ring_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_WITH_ROTOR_JOINT_H
