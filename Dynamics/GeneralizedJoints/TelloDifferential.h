#pragma once

#include "GeneralizedJoint.h"
#include "3rd-parties/CasadiGen/header/CasadiGen.h"

namespace grbda
{

	namespace LoopConstraint
	{
		struct TelloDifferential : Base
		{
			typedef int (*casadi_fn)(const double **, double **, long long int *, double *, int);
			typedef const long long int *(*casadi_sparsity_out_fn)(long long int);
			typedef int (*casadi_work_fn)(long long int *, long long int *, long long int *, long long int *);

			TelloDifferential(CasadiHelperFunctions G_helpers, CasadiHelperFunctions g_helpers,
							  CasadiHelperFunctions k_helpers)
				: G_helpers_(G_helpers), g_helpers_(g_helpers), k_helpers_(k_helpers)
			{
				G_.setZero(4, 2);
				K_.setZero(2, 4);
				g_.setZero(4);
				k_.setZero(2);

				G_.topRows<2>() = DMat<double>::Identity(2, 2);
				K_.rightCols<2>() = DMat<double>::Identity(2, 2);
			}

			virtual std::shared_ptr<Base> clone() const
			{
				return std::make_shared<TelloDifferential>(*this);
			}

			DVec<double> gamma(const JointCoordinate &joint_pos) const override
			{
				throw std::runtime_error("Tello loop constraint does not have a gamma function");
			}

			void updateJacobians(const JointCoordinate &joint_pos) override
			{
#ifdef DEBUG_MODE
				if (!joint_pos.isSpanning())
					throw std::runtime_error("[TelloDifferential] Position for updating constraint Jacobians must be spanning");
#endif
				vector<DVec<double>> arg = {joint_pos.head<2>(), joint_pos.tail<2>()};
				Mat2<double> J_dy_2_dqd;
				casadi_interface(arg, J_dy_2_dqd, G_helpers_);

				G_.bottomRows<2>() = J_dy_2_dqd;
				K_.leftCols<2>() = -G_.bottomRows<2>();
			}

            void updateBiases(const JointState &joint_state) override
			{
#ifdef DEBUG_MODE
				if (!joint_state.position.isSpanning() || !joint_state.velocity.isSpanning())
					throw std::runtime_error("[TelloDifferential] Position and velocity for updating constraint bias must be spanning");
#endif

				const DVec<double> &q = joint_state.position;
				const DVec<double> &q_dot = joint_state.velocity;

				vector<DVec<double>> arg = {q.head<2>(), q.tail<2>(), q_dot.head<2>(), q_dot.tail<2>()};
				casadi_interface(arg, g_, g_helpers_);
				casadi_interface(arg, k_, k_helpers_);
			}

		protected:
			CasadiHelperFunctions G_helpers_;
			CasadiHelperFunctions g_helpers_;
			CasadiHelperFunctions k_helpers_;
			CasadiHelperFunctions kikd_helpers_;
			CasadiHelperFunctions IK_pos_helpers_;
			CasadiHelperFunctions IK_vel_helpers_;
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
	    typedef int (*casadi_fn)(const double**, double**, long long int*, double*, int);
	    typedef const long long int* (*casadi_sparsity_out_fn)(long long int);
	    typedef int (*casadi_work_fn)(long long int*, long long int*, long long int*, long long int*);

		// TODO(@MatthewChignoli): Eventually get rid of all of this stuff, no need for different TelloDifferential inherited classes
	    casadi_fn td_kikd;
	    casadi_sparsity_out_fn td_kikd_sparsity_out;
	    casadi_work_fn td_kikd_work;
	    casadi_fn td_IK_pos = thd_IK_pos;
	    casadi_sparsity_out_fn td_IK_pos_sparsity_out;
	    casadi_work_fn td_IK_pos_work;
	    casadi_fn td_IK_vel;
	    casadi_sparsity_out_fn td_IK_vel_sparsity_out;
	    casadi_work_fn td_IK_vel_work;

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
