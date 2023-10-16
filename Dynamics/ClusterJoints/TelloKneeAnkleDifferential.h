#ifndef GRBDA_GENERALIZED_JOINTS_TELLO_KNEE_ANKLE_DIFFERENTIAL_H
#define GRBDA_GENERALIZED_JOINTS_TELLO_KNEE_ANKLE_DIFFERENTIAL_H

#include "TelloDifferential.h"
#include "Utils/CasadiGen/header/CasadiGen.h"

namespace grbda
{

    namespace ClusterJoints
    {

        class TelloKneeAnkleDifferential : public TelloDifferential
        {
        public:
            TelloKneeAnkleDifferential(TelloDifferentialModule &module)
            : TelloDifferential(module)
            {
            CasadiHelperFunctions jacobian_helpers(tkad_jacobian, tkad_jacobian_sparsity_out,
                                            tkad_jacobian_work);
            CasadiHelperFunctions bias_helpers(tkad_bias, tkad_bias_sparsity_out, tkad_bias_work);
            CasadiHelperFunctions IK_pos_helpers(tkad_IK_pos, tkad_IK_pos_sparsity_out,
                                                 tkad_IK_pos_work);
            CasadiHelperFunctions IK_vel_helpers(tkad_IK_vel, tkad_IK_vel_sparsity_out,
                                                 tkad_IK_vel_work);
    
            tello_constraint_ = std::make_shared<LoopConstraint::TelloDifferential>(
                jacobian_helpers, bias_helpers, IK_pos_helpers, IK_vel_helpers);
            loop_constraint_ = tello_constraint_;
            }
            virtual ~TelloKneeAnkleDifferential() {}
    
            ClusterJointTypes type() const override { return ClusterJointTypes::TelloKneeAnkleDifferential; }
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_TELLO_KNEE_ANKLE_DIFFERENTIAL_H
