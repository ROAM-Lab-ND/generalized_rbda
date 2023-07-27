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
				 CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2)
				 : TelloDifferential(rotor_1, rotor_2, link_1, link_2,
				 rotor_axis_1, rotor_axis_2, joint_axis_1, joint_axis_2)
	    {
		CasadiHelperFunctions G_helpers(thd_J_dy_2_dqd, thd_J_dy_2_dqd_sparsity_out,
										thd_J_dy_2_dqd_work);
		CasadiHelperFunctions g_helpers(thd_g, thd_g_sparsity_out, thd_g_work);
		CasadiHelperFunctions k_helpers(thd_k, thd_k_sparsity_out, thd_k_work);
		CasadiHelperFunctions kikd_helpers(thd_kikd, thd_kikd_sparsity_out, thd_kikd_work);
		CasadiHelperFunctions IK_pos_helpers(thd_IK_pos, thd_IK_pos_sparsity_out, thd_IK_pos_work);
		CasadiHelperFunctions IK_vel_helpers(thd_IK_vel, thd_IK_vel_sparsity_out, thd_IK_vel_work);

		tello_constraint_ = std::make_shared<LoopConstraint::TelloDifferential>(
			G_helpers, g_helpers, k_helpers, kikd_helpers, IK_pos_helpers, IK_vel_helpers);
		loop_constraint_ = tello_constraint_;
	    }
	    virtual ~TelloHipDifferential() {}

	    GeneralizedJointTypes type() const { return GeneralizedJointTypes::TelloHipDifferential; }

	};

    }

} // namespace grbda
