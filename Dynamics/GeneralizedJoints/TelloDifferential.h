#pragma once

#include "GeneralizedJoint.h"
#include "3rd-parties/CasadiGen/header/CasadiGen.h"

namespace grbda
{

	namespace LoopConstraint
	{
		struct TelloDifferential : Base
		{
			TelloDifferential(const CasadiHelperFunctions &G_helpers,
							  const CasadiHelperFunctions &g_helpers,
							  const CasadiHelperFunctions &k_helpers,
							  const CasadiHelperFunctions &kikd_helpers,
							  const CasadiHelperFunctions &IK_pos_helpers,
							  const CasadiHelperFunctions &IK_vel_helpers);

			std::shared_ptr<Base> clone() const override;

			DVec<double> gamma(const JointCoordinate &joint_pos) const override;

			void updateJacobians(const JointCoordinate &joint_pos) override;
			void updateBiases(const JointState &joint_state) override;

			const CasadiHelperFunctions G_helpers_;
			const CasadiHelperFunctions g_helpers_;
			const CasadiHelperFunctions k_helpers_;
			const CasadiHelperFunctions kikd_helpers_;
			const CasadiHelperFunctions IK_pos_helpers_;
			const CasadiHelperFunctions IK_vel_helpers_;
		};
	}

	namespace GeneralizedJoints
    {

	class TelloDifferential : public Base
	{
	public:
	    TelloDifferential(Body &rotor_1, Body &rotor_2, Body &link_1, Body &link_2,
				 CoordinateAxis rotor_axis_1, CoordinateAxis rotor_axis_2,
				 CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2);
	    virtual ~TelloDifferential() {}

		void updateKinematics(const JointState &joint_state) override;

	    void computeSpatialTransformFromParentToCurrentCluster(
		GeneralizedSpatialTransform &Xup) const override;

		std::vector<std::tuple<Body, JointPtr, DMat<double>>>
		bodiesJointsAndReflectedInertias() const override
		{
			std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_ref_inertias_;
			const Mat2<double> Z = Mat2<double>::Zero();
			bodies_joints_and_ref_inertias_.push_back(std::make_tuple(link_1_, link_1_joint_, Z));
			bodies_joints_and_ref_inertias_.push_back(std::make_tuple(link_2_, link_2_joint_, Z));
			return bodies_joints_and_ref_inertias_;
		}

		JointState randomJointState() const override;

	protected:
		std::shared_ptr<LoopConstraint::TelloDifferential> tello_constraint_;

	private:
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
