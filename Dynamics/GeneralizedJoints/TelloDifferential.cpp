#include "TelloDifferential.h"

namespace grbda
{

	namespace LoopConstraint
	{
		TelloDifferential::TelloDifferential(const CasadiHelperFunctions &G_helpers,
											 const CasadiHelperFunctions &g_helpers,
											 const CasadiHelperFunctions &k_helpers,
											 const CasadiHelperFunctions &kikd_helpers,
											 const CasadiHelperFunctions &IK_pos_helpers,
											 const CasadiHelperFunctions &IK_vel_helpers)
			: G_helpers_(G_helpers), g_helpers_(g_helpers),
			  k_helpers_(k_helpers), kikd_helpers_(kikd_helpers),
			  IK_pos_helpers_(IK_pos_helpers), IK_vel_helpers_(IK_vel_helpers)
		{
			G_.setZero(4, 2);
			K_.setZero(2, 4);
			g_.setZero(4);
			k_.setZero(2);

			G_.topRows<2>() = DMat<double>::Identity(2, 2);
			K_.rightCols<2>() = DMat<double>::Identity(2, 2);
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
			Mat2<double> J_dy_2_dqd;
			casadi_interface(arg, J_dy_2_dqd, G_helpers_);

			G_.bottomRows<2>() = J_dy_2_dqd;
			K_.leftCols<2>() = -G_.bottomRows<2>();
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
			casadi_interface(arg, g_, g_helpers_);
			casadi_interface(arg, k_, k_helpers_);
		}

	}

	namespace GeneralizedJoints
    {

	TelloDifferential::TelloDifferential(
	    Body &rotor_1, Body &rotor_2, Body &link_1, Body &link_2,
	    CoordinateAxis rotor_axis_1, CoordinateAxis rotor_axis_2,
	    CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2)
	    : Base(4, 4, 2, true, false), rotor_1_(rotor_1), rotor_2_(rotor_2),
	    link_1_(link_1), link_2_(link_2)
	{
	    rotor_1_joint_ = single_joints_.emplace_back(new Joints::Revolute(rotor_axis_1));
	    rotor_2_joint_ = single_joints_.emplace_back(new Joints::Revolute(rotor_axis_2));
	    link_1_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_1));
	    link_2_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_2));

	    S_.block<6, 1>(0, 0) = rotor_1_joint_->S();
	    S_.block<6, 1>(6, 1) = rotor_2_joint_->S();
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

	    const DMat<double>& S1 = link_1_joint_->S();
	    const DMat<double> X21_S1 = X21_.transformMotionSubspace(S1);
	    const DMat<double>& S2 = link_2_joint_->S();
	    const DVec<double> v2_relative = S2 * q_dot[3];
	    const DMat<double> v2_rel_crm = generalMotionCrossMatrix(v2_relative);

		const DMat<double> G = loop_constraint_->G();
	    S_.block<6, 1>(12, 0) = S1 * G.block<1, 1>(2,0);
	    S_.block<6, 1>(12, 1) = S1 * G.block<1, 1>(2, 1);
	    S_.block<6, 1>(18, 0) = X21_S1 * G.block<1, 1>(2, 0) + S2 * G.block<1, 1>(3, 0);
	    S_.block<6, 1>(18, 1) = X21_S1 * G.block<1 ,1>(2, 1) + S2 * G.block<1, 1>(3, 1);

	    // Given matrix abcd = [a b;c d] = G_.bottomRows(2) = -Kd.inv()*Ki,
	    // calculate a_dot, b_dot, c_dot, d_dot for S_ring_
	    vector<DVec<double>> arg = {q.head<2>(), q.tail<2>(), q_dot.head<2>(), q_dot.tail<2>()};
	    Mat2<double> Ki, Kd, Ki_dot, Kd_dot;
	    vector<Eigen::MatrixBase<Mat2<double>>*> K = {&Ki, &Kd, &Ki_dot, &Kd_dot};
	    casadi_interface(arg, K, tello_constraint_->kikd_helpers_);

	    const Mat2<double> Kd_inv = Kd.inverse();
	    Mat2<double> abcd_dot = -Kd_inv * Ki_dot + Kd_inv * Kd_dot * Kd_inv * Ki;

	    S_ring_.block<6, 1>(12, 0) = S1 * abcd_dot.block<1, 1>(0, 0);
	    S_ring_.block<6, 1>(12, 1) = S1 * abcd_dot.block<1, 1>(0, 1);
	    S_ring_.block<6, 1>(18, 0) = X21_S1 * abcd_dot.block<1, 1>(0, 0) +\
					 S2 * abcd_dot.block<1, 1>(1, 0) +\
					 (-v2_rel_crm * X21_S1) * G.block<1, 1>(2, 0);
	    S_ring_.block<6, 1>(18, 1) = X21_S1 * abcd_dot.block<1, 1>(0, 1) +\
					 S2 * abcd_dot.block<1, 1>(1, 1) +\
					 (-v2_rel_crm * X21_S1) * G.block<1, 1>(2, 1);

	    vJ_ = S_ * joint_state.velocity;
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
	    Vec2<double> y = Vec2<double>::Zero(2);
	    casadi_interface(dependent_state, y, tello_constraint_->IK_pos_helpers_);
	    joint_state.position << y, dependent_state[0];

	    // Velocity
	    dependent_state.push_back(DVec<double>::Random(2));
	    Vec2<double> y_dot = Vec2<double>::Zero(2);
	    casadi_interface(dependent_state, y_dot, tello_constraint_->IK_vel_helpers_);
	    joint_state.velocity << y_dot;

	    return joint_state;
	}
    }

} // namespace grbda
