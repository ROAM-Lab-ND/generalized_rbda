#pragma once

#include "GeneralizedJoint.h"
#include "CasadiGen.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

	class TelloHipDifferential : public Base
	{
	public:
	    TelloHipDifferential(Body &rotor_1, Body &rotor_2, Body &link_1, Body &link_2,
				 CoordinateAxis rotor_axis_1, CoordinateAxis rotor_axis_2,
				 CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2);
	    virtual ~TelloHipDifferential() {}

	    GeneralizedJointTypes type() const override { return GeneralizedJointTypes::TelloHipDifferential; }

		void updateKinematics(const JointState &joint_state) override;

		void computeSpatialTransformFromParentToCurrentCluster(
		GeneralizedSpatialTransform &Xup) const override;

		JointState randomJointState() const override;

	private:
		void updateConstraintKinematics(const JointState &joint_state) override
		{
			// TODO(@MatthewChignoli): We are double dipping here in terms of having duplicate code, but Nicholas can clean this up

			const DVec<double> &q = joint_state.position; // TODO(@MatthewChignoli): Needs to be spanning

			Mat2<double> J_dy_2_dqd;
			vector<DVec<double>> arg = {q.head<2>(), q.tail<2>()};
			casadi_interface(arg, J_dy_2_dqd, thd_J_dy_2_dqd, thd_J_dy_2_dqd_sparsity_out, thd_J_dy_2_dqd_work);

			G_.bottomRows<2>() = J_dy_2_dqd;
		}

		JointPtr rotor_1_joint_;
	    JointPtr rotor_2_joint_;
	    JointPtr link_1_joint_;
	    JointPtr link_2_joint_;

	    SpatialTransform X21_;
	    
	    const Body rotor_1_;
	    const Body rotor_2_;
	    const Body link_1_;
	    const Body link_2_;

	};

    }

} // namespace grbda
