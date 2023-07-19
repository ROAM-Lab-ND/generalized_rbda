#pragma once

#include "GeneralizedJoint.h"
#include "3rd-parties/CasadiGen/header/CasadiGen.h"

namespace grbda
{

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
	    typedef int (*casadi_fn)(const double**, double**, long long int*, double*, int);
	    typedef const long long int* (*casadi_sparsity_out_fn)(long long int);
	    typedef int (*casadi_work_fn)(long long int*, long long int*, long long int*, long long int*);

	    casadi_fn td_kikd;
	    casadi_sparsity_out_fn td_kikd_sparsity_out;
	    casadi_work_fn td_kikd_work;
	    casadi_fn td_J_dy_2_dqd;
	    casadi_sparsity_out_fn td_J_dy_2_dqd_sparsity_out;
	    casadi_work_fn td_J_dy_2_dqd_work;
	    casadi_fn td_g;
	    casadi_sparsity_out_fn td_g_sparsity_out;
	    casadi_work_fn td_g_work;
	    casadi_fn td_k;
	    casadi_sparsity_out_fn td_k_sparsity_out;
	    casadi_work_fn td_k_work;
	    casadi_fn td_IK_pos = thd_IK_pos;
	    casadi_sparsity_out_fn td_IK_pos_sparsity_out;
	    casadi_work_fn td_IK_pos_work;
	    casadi_fn td_IK_vel;
	    casadi_sparsity_out_fn td_IK_vel_sparsity_out;
	    casadi_work_fn td_IK_vel_work;

	private:
	    void updateConstraintJacobians(const JointCoordinate &joint_pos) override;
	    void updateConstraintBias(const JointState &joint_state) override;

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
