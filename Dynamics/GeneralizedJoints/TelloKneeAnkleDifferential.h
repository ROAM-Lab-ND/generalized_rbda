#pragma once

#include "TelloDifferential.h"
#include "3rd-parties/CasadiGen/header/CasadiGen.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

        class TelloKneeAnkleDifferential : public TelloDifferential
        {
        public:
            TelloKneeAnkleDifferential(Body &rotor_1, Body &rotor_2, Body &link_1, Body &link_2,
                           CoordinateAxis rotor_axis_1, CoordinateAxis rotor_axis_2,
                           CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2,
                           double gear_ratio)
                           : TelloDifferential(rotor_1, rotor_2, link_1, link_2,
                           rotor_axis_1, rotor_axis_2, joint_axis_1, joint_axis_2,
                           gear_ratio)
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
    
            GeneralizedJointTypes type() const override { return GeneralizedJointTypes::TelloKneeAnkleDifferential; }
        };

    }

} // namespace grbda
