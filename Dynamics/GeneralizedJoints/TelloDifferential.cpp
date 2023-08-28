#include "TelloDifferential.h"

namespace grbda
{

	namespace LoopConstraint
	{
		TelloDifferential::TelloDifferential(const CasadiHelperFunctions &jacobian_helpers,
											 const CasadiHelperFunctions &bias_helpers,
											 const CasadiHelperFunctions &IK_pos_helpers,
											 const CasadiHelperFunctions &IK_vel_helpers)
			: jacobian_helpers_(jacobian_helpers), bias_helpers_(bias_helpers),
              IK_pos_helpers_(IK_pos_helpers), IK_vel_helpers_(IK_vel_helpers)
		{
			G_.setZero(4, 2);
			K_.setZero(2, 4);
			g_.setZero(4);
			k_.setZero(2);
		}

		std::shared_ptr<Base> TelloDifferential::clone() const
		{
			return std::make_shared<TelloDifferential>(*this);
		}

		DVec<double> TelloDifferential::gamma(const JointCoordinate &joint_pos) const
		{
			throw std::runtime_error("Tello loop constraint does not have a gamma function");
		}

		void TelloDifferential::updateJacobians(const JointCoordinate &joint_pos)
		{
#ifdef DEBUG_MODE
			if (!joint_pos.isSpanning())
				throw std::runtime_error("[TelloDifferential] Position for updating constraint Jacobians must be spanning");
#endif
			vector<DVec<double>> arg = {joint_pos.head<2>(), joint_pos.tail<2>()};
            vector<Eigen::MatrixBase<DMat<double>>*> J = {&G_, &K_};
			casadi_interface(arg, J, jacobian_helpers_);
		}

		void TelloDifferential::updateBiases(const JointState &joint_state)
		{
#ifdef DEBUG_MODE
			if (!joint_state.position.isSpanning() || !joint_state.velocity.isSpanning())
				throw std::runtime_error("[TelloDifferential] Position and velocity for updating constraint bias must be spanning");
#endif

			const DVec<double> &q = joint_state.position;
			const DVec<double> &q_dot = joint_state.velocity;

			vector<DVec<double>> arg = {q.head<2>(), q.tail<2>(), q_dot.head<2>(), q_dot.tail<2>()};
            vector<Eigen::MatrixBase<DVec<double>>*> b = {&g_, &k_};
			casadi_interface(arg, b, bias_helpers_);
		}

	}

	namespace GeneralizedJoints
    {

	TelloDifferential::TelloDifferential(
	    Body &rotor_1, Body &rotor_2, Body &link_1, Body &link_2,
	    CoordinateAxis rotor_axis_1, CoordinateAxis rotor_axis_2,
	    CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2,
        double gear_ratio)
	    : Base(4, 4, 2, true, false), rotor_1_(rotor_1), rotor_2_(rotor_2),
	    link_1_(link_1), link_2_(link_2), gear_ratio_(gear_ratio)
	{
	    rotor_1_joint_ = single_joints_.emplace_back(new Joints::Revolute(rotor_axis_1));
	    rotor_2_joint_ = single_joints_.emplace_back(new Joints::Revolute(rotor_axis_2));
	    link_1_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_1));
	    link_2_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_2));

        S_.block<6, 1>(0, 0) = gear_ratio * rotor_1_joint_->S();
        S_.block<6, 1>(6, 1) = gear_ratio * rotor_2_joint_->S();
    }

	void TelloDifferential::updateKinematics(const JointState &joint_state)
	{
#ifdef DEBUG_MODE
	    jointStateCheck(joint_state);
#endif

	    const JointState spanning_joint_state = toSpanningTreeState(joint_state);
	    const DVec<double> &q = spanning_joint_state.position;
	    const DVec<double> &q_dot = spanning_joint_state.velocity;

	    rotor_1_joint_->updateKinematics(q.segment<1>(0), q_dot.segment<1>(0));
	    rotor_2_joint_->updateKinematics(q.segment<1>(1), q_dot.segment<1>(1));
	    link_1_joint_->updateKinematics(q.segment<1>(2), q_dot.segment<1>(2));
	    link_2_joint_->updateKinematics(q.segment<1>(3), q_dot.segment<1>(3));

	    X21_ = link_2_joint_->XJ() * link_2_.Xtree_;

		const DMat<double> &S1 = link_1_joint_->S();
		const DMat<double> X21_S1 = X21_.transformMotionSubspace(S1);
		const DMat<double> &S2 = link_2_joint_->S();
		const DVec<double> v2_relative = S2 * q_dot[3];
	    const DMat<double> v2_rel_crm = generalMotionCrossMatrix(v2_relative);

        S_.block<6, 1>(12, 0) = S1 * G()(2, 0);
        S_.block<6, 1>(12, 1) = S1 * G()(2, 1);
        S_.block<6, 1>(18, 0) = X21_S1 * G()(2, 0) + S2 * G()(3, 0);
        S_.block<6, 1>(18, 1) = X21_S1 * G()(2, 1) + S2 * G()(3, 1);
	    
		vJ_ = S_ * joint_state.velocity;

        cJ_.segment<6>(0) = rotor_1_joint_->S() * g()[0];
        cJ_.segment<6>(6) = rotor_2_joint_->S() * g()[1];
        cJ_.segment<6>(12) = S1 * g()[2];
        cJ_.segment<6>(18) = -v2_rel_crm * X21_S1 * q_dot[2] + X21_S1 * g()[2] + S2 * g()[3];
	}

	void TelloDifferential::computeSpatialTransformFromParentToCurrentCluster(
		GeneralizedSpatialTransform &Xup) const
	{
#ifdef DEBUG_MODE
	    if (Xup.getNumOutputBodies() != 4)
		throw std::runtime_error("[TelloDifferential] Xup must have 24 rows");
#endif

	    Xup[0] = rotor_1_joint_->XJ() * rotor_1_.Xtree_;
	    Xup[1] = rotor_2_joint_->XJ() * rotor_2_.Xtree_;
	    Xup[2] = link_1_joint_->XJ() * link_1_.Xtree_;
	    Xup[3] = link_2_joint_->XJ() * link_2_.Xtree_ * Xup[2];
	}

	JointState TelloDifferential::randomJointState() const
	{
	    JointCoordinate joint_pos(DVec<double>::Zero(num_positions_), position_is_spanning_);
	    JointCoordinate joint_vel(DVec<double>::Zero(num_velocities_), velocity_is_spanning_);
	    JointState joint_state(joint_pos, joint_vel);

	    // Position
	    std::vector<DVec<double>> dependent_state = {DVec<double>::Random(2)};
	    Vec2<double> minimal_pos = Vec2<double>::Zero(2);
	    casadi_interface(dependent_state, minimal_pos, tello_constraint_->IK_pos_helpers_);
        Vec2<double> independent_pos = gear_ratio_ * minimal_pos;
	    joint_state.position << independent_pos, dependent_state[0];

	    // Velocity
	    dependent_state.push_back(DVec<double>::Random(2));
	    Vec2<double> minimal_vel = Vec2<double>::Zero(2);
	    casadi_interface(dependent_state, minimal_vel, tello_constraint_->IK_vel_helpers_);
	    joint_state.velocity << minimal_vel;

	    return joint_state;
	}

	std::vector<std::tuple<Body, JointPtr, DMat<double>>>
	TelloDifferential::bodiesJointsAndReflectedInertias() const
	{
		std::vector<std::tuple<Body, JointPtr, DMat<double>>> bodies_joints_and_ref_inertias_;
		const Mat2<double> Z = Mat2<double>::Zero();
		bodies_joints_and_ref_inertias_.push_back(std::make_tuple(link_1_, link_1_joint_, Z));
		bodies_joints_and_ref_inertias_.push_back(std::make_tuple(link_2_, link_2_joint_, Z));
		return bodies_joints_and_ref_inertias_;
	}

	}

} // namespace grbda
