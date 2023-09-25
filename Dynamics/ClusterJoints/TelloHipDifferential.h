#ifndef GRBDA_GENERALIZED_JOINTS_TELLO_HIP_DIFFERENTIAL_H
#define GRBDA_GENERALIZED_JOINTS_TELLO_HIP_DIFFERENTIAL_H

#include "TelloDifferential.h"
#include "Utils/CasadiGen/header/CasadiGen.h"

namespace grbda
{

    namespace ClusterJoints
    {

        class TelloHipDifferential : public TelloDifferential<>
        {
        public:
            TelloHipDifferential(TelloDifferentialModule &module)
            : TelloDifferential(module)
            {
                CasadiHelperFunctions jacobian_helpers(thd_jacobian, thd_jacobian_sparsity_out,
                                                       thd_jacobian_work);
                CasadiHelperFunctions bias_helpers(thd_bias, thd_bias_sparsity_out, thd_bias_work);
                CasadiHelperFunctions IK_pos_helpers(thd_IK_pos, thd_IK_pos_sparsity_out, thd_IK_pos_work);
                CasadiHelperFunctions IK_vel_helpers(thd_IK_vel, thd_IK_vel_sparsity_out, thd_IK_vel_work);

                tello_constraint_ = std::make_shared<LoopConstraint::TelloDifferential<double>>(
                    jacobian_helpers, bias_helpers, IK_pos_helpers, IK_vel_helpers);
                loop_constraint_ = tello_constraint_;
            }
            virtual ~TelloHipDifferential() {}

            ClusterJointTypes type() const override { return ClusterJointTypes::TelloHipDifferential; }
        };

    }

} // namespace grbda

#endif // GRBDA_GENERALIZED_JOINTS_TELLO_HIP_DIFFERENTIAL_H
