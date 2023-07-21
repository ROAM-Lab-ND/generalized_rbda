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
		td_kikd = thd_kikd;
		td_kikd_sparsity_out = thd_kikd_sparsity_out;
		td_kikd_work = thd_kikd_work;
		td_J_dy_2_dqd = thd_J_dy_2_dqd;
		td_J_dy_2_dqd_sparsity_out = thd_J_dy_2_dqd_sparsity_out;
		td_J_dy_2_dqd_work = thd_J_dy_2_dqd_work;
		td_g = thd_g;
		td_g_sparsity_out = thd_g_sparsity_out;
		td_g_work = thd_g_work;
		td_k = thd_k;
		td_k_sparsity_out = thd_k_sparsity_out;
		td_k_work = thd_k_work;
		td_IK_pos = thd_IK_pos;
		td_IK_pos_sparsity_out = thd_IK_pos_sparsity_out;
		td_IK_pos_work = thd_IK_pos_work;
		td_IK_vel = thd_IK_vel;
		td_IK_vel_sparsity_out = thd_IK_vel_sparsity_out;
		td_IK_vel_work = thd_IK_vel_work;

		CasadiHelperFunctions G_helpers(thd_J_dy_2_dqd, thd_J_dy_2_dqd_sparsity_out, thd_J_dy_2_dqd_work);
		CasadiHelperFunctions g_helpers(thd_g, thd_g_sparsity_out, thd_g_work);
		CasadiHelperFunctions k_helpers(thd_k, thd_k_sparsity_out, thd_k_work);
		loop_constraint_ = std::make_shared<LoopConstraint::TelloDifferential>(G_helpers, g_helpers, k_helpers);
		
		}
	    virtual ~TelloHipDifferential() {}

	    GeneralizedJointTypes type() const override { return GeneralizedJointTypes::TelloHipDifferential; }

	};

    }

} // namespace grbda
