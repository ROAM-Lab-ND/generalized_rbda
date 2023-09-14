#pragma once

#include "TelloDifferential.h"
#include "3rd-parties/CasadiGen/header/CasadiGen.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class TelloHipDifferential : public TelloDifferential
        {
        public:
            TelloHipDifferential(Body &rotor_1, Body &rotor_2, Body &link_1, Body &link_2,
                     CoordinateAxis rotor_axis_1, CoordinateAxis rotor_axis_2,
                     CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2,
                     double gear_ratio)
                     : TelloDifferential(rotor_1, rotor_2, link_1, link_2,
                     rotor_axis_1, rotor_axis_2, joint_axis_1, joint_axis_2,
                     gear_ratio)
            {
            CasadiHelperFunctions jacobian_helpers(thd_jacobian, thd_jacobian_sparsity_out,
                                            thd_jacobian_work);
            CasadiHelperFunctions bias_helpers(thd_bias, thd_bias_sparsity_out, thd_bias_work);
            CasadiHelperFunctions IK_pos_helpers(thd_IK_pos, thd_IK_pos_sparsity_out, thd_IK_pos_work);
            CasadiHelperFunctions IK_vel_helpers(thd_IK_vel, thd_IK_vel_sparsity_out, thd_IK_vel_work);
    
            tello_constraint_ = std::make_shared<LoopConstraint::TelloDifferential>(
                jacobian_helpers, bias_helpers, IK_pos_helpers, IK_vel_helpers);
            loop_constraint_ = tello_constraint_;
            }
            virtual ~TelloHipDifferential() {}
    
            GeneralizedJointTypes type() const override { return GeneralizedJointTypes::TelloHipDifferential; }
    
        };

    }

} // namespace grbda
