#ifndef GRBDA_GENERALIZED_JOINTS_TELLO_KNEE_ANKLE_DIFFERENTIAL_H
#define GRBDA_GENERALIZED_JOINTS_TELLO_KNEE_ANKLE_DIFFERENTIAL_H

#include "grbda/Dynamics/ClusterJoints/TelloDifferential.h"
#include "CasadiGen.h"

namespace grbda
{

    namespace ClusterJoints
    {

        template <typename Scalar>
        class TelloKneeAnkleDifferential : public TelloDifferential<Scalar>
        {
        public:
            TelloKneeAnkleDifferential(TelloDifferentialModule<Scalar> &module)
            : TelloDifferential<Scalar>(module)
            {
                CasadiHelperFunctions<Scalar> jacobian_helpers(tkad_jacobian,
                                                               tkad_jacobian_sparsity_out,
                                                               tkad_jacobian_work);
                CasadiHelperFunctions<Scalar> bias_helpers(tkad_bias, tkad_bias_sparsity_out,
                                                           tkad_bias_work);
                CasadiHelperFunctions<Scalar> IK_pos_helpers(tkad_IK_pos, tkad_IK_pos_sparsity_out,
                                                             tkad_IK_pos_work);
                CasadiHelperFunctions<Scalar> IK_vel_helpers(tkad_IK_vel, tkad_IK_vel_sparsity_out,
                                                             tkad_IK_vel_work);

                this->tello_constraint_ =
                    std::make_shared<LoopConstraint::TelloDifferential<Scalar>>(
                        jacobian_helpers, bias_helpers, IK_pos_helpers, IK_vel_helpers);
                this->loop_constraint_ = this->tello_constraint_;
            }
            virtual ~TelloKneeAnkleDifferential() {}
    
            ClusterJointTypes type() const override { return ClusterJointTypes::TelloKneeAnkleDifferential; }
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_TELLO_KNEE_ANKLE_DIFFERENTIAL_H
