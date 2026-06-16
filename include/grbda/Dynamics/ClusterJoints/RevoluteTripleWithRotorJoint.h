#ifndef GRBDA_GENERALIZED_JOINTS_REVOLUTE_TRIPLE_WITH_ROTOR_JOINT_H
#define GRBDA_GENERALIZED_JOINTS_REVOLUTE_TRIPLE_WITH_ROTOR_JOINT_H

#include "grbda/Dynamics/ClusterJoints/GenericJoint.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar = double>
        class RevoluteTripleWithRotor : public Generic<Scalar>
        {
        public:
            typedef ParallelBeltTransmissionModule<1, Scalar> ProximalTransmission;
            typedef ParallelBeltTransmissionModule<2, Scalar> IntermediateTransmission;
            typedef ParallelBeltTransmissionModule<3, Scalar> DistalTransmission;

            RevoluteTripleWithRotor(const ProximalTransmission &module_1,
                                    const IntermediateTransmission &module_2,
                                    const DistalTransmission &module_3);
            virtual ~RevoluteTripleWithRotor() {}

            ClusterJointTypes type() const override
            {
                return ClusterJointTypes::RevoluteTripleWithRotor;
            }

            std::vector<std::tuple<Body<Scalar>, JointPtr<Scalar>, DMat<Scalar>>>
            bodiesJointsAndReflectedInertias() const override;

            const Mat3<Scalar>& getRatioProduct() const { return ratio_product_; }

            int link1Index() const { return link1_index_; }
            int link2Index() const { return link2_index_; }
            int link3Index() const { return link3_index_; }
            int rotor1Index() const { return rotor1_index_; }
            int rotor2Index() const { return rotor2_index_; }
            int rotor3Index() const { return rotor3_index_; }

        private:
            const Body<Scalar> link1_;
            const Body<Scalar> link2_;
            const Body<Scalar> link3_;
            const Body<Scalar> rotor1_;
            const Body<Scalar> rotor2_;
            const Body<Scalar> rotor3_;

            const int link1_index_;
            const int link2_index_;
            const int link3_index_;
            const int rotor1_index_;
            const int rotor2_index_;
            const int rotor3_index_;

            Mat3<Scalar> ratio_product_;
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_REVOLUTE_TRIPLE_WITH_ROTOR_JOINT_H
