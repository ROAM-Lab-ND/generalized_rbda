#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_WITH_ROTOR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_WITH_ROTOR_JOINT_H

#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class RevolutePairWithRotor : public Generic<Scalar>
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

            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
            bodiesJointsAndReflectedInertias() const override;

        private:
            const Body<Scalar> link1_;
            const Body<Scalar> link2_;
            const Body<Scalar> rotor1_;
            const Body<Scalar> rotor2_;

            const int link1_index_;
            const int link2_index_;
            const int rotor1_index_;
            const int rotor2_index_;

            // Gear/belt ratio matrix: ratio_product_(i, j) is the effective ratio from link j
            // to rotor i. Stored at construction so bodiesJointsAndReflectedInertias() can
            // compute reflected inertia without needing a prior updateKinematics() call.
            Mat2<Scalar> ratio_product_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_PAIR_WITH_ROTOR_JOINT_H
