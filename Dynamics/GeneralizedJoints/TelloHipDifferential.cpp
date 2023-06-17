#include "TelloHipDifferential.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

	TelloHipDifferential::TelloHipDifferential(
	    Body &rotor_1, Body &rotor_2, Body &link_1, Body &link_2,
	    CoordinateAxis rotor_axis_1, CoordinateAxis rotor_axis_2,
	    CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2)
	    : Base(2, 2, 4), rotor_1_(rotor_1), rotor_2_(rotor_2),
	    link_1_(link_1), link_2_(link_2)
	{
	    rotor_1_joint_ = single_joints_.emplace_back(new Joints::Revolute(rotor_axis_1));
	    rotor_2_joint_ = single_joints_.emplace_back(new Joints::Revolute(rotor_axis_2));
	    link_1_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_1));
	    link_2_joint_ = single_joints_.emplace_back(new Joints::Revolute(joint_axis_2));

	    G_.setZero(4, 2);
	    K_.setZero(2, 4);
	    g_.setZero(4);
	    k_.setZero(2);

	    G_.topRows<2>() = DMat<double>::Identity(2, 2);
	    K_.rightCols<2>() = DMat<double>::Identity(2, 2);
	    
	    S_.block<6, 1>(0, 0) = rotor_1_joint_->S();
	    S_.block<6, 1>(6, 1) = rotor_2_joint_->S();
	}

	void TelloHipDifferential::updateKinematics(const State<double> &joint_pos,
												const State<double> &joint_vel)
	{
		// TODO(@MatthewChignoli): Commented out because this depends on the State type
		// if (q.size() != 4)
		//     throw std::runtime_error("[TelloHipDifferential] Dimension of joint position must be 4");

		// DVec<double> q_dot = G_ * yd;
		const DVec<double> q = toSpanningTreePositions(joint_pos);
		const DVec<double> q_dot = toSpanningTreeVelocities(joint_vel);

		Mat2<double> J_dy_2_dqd;
	    vector<DVec<double>> arg = {q.head<2>(), q.tail<2>()};
	    casadi_interface(arg, J_dy_2_dqd, thd_J_dy_2_dqd, thd_J_dy_2_dqd_sparsity_out, thd_J_dy_2_dqd_work);

	    G_.bottomRows<2>() = J_dy_2_dqd;

		rotor_1_joint_->updateKinematics(q.segment<1>(0), q_dot.segment<1>(0));
	    rotor_2_joint_->updateKinematics(q.segment<1>(1), q_dot.segment<1>(1));
	    link_1_joint_->updateKinematics(q.segment<1>(2), q_dot.segment<1>(2));
	    link_2_joint_->updateKinematics(q.segment<1>(3), q_dot.segment<1>(3));

	    K_.leftCols<2>() = -G_.bottomRows<2>();

	    // Calculate g and k
	    arg = {q.head<2>(), q.tail<2>(), q_dot.head<2>(), q_dot.tail<2>()};
	    casadi_interface(arg, g_, thd_g, thd_g_sparsity_out, thd_g_work);
	    casadi_interface(arg, k_, thd_k, thd_k_sparsity_out, thd_k_work);

	    X21_ = link_2_joint_->XJ() * link_2_.Xtree_;

	    const DMat<double>& S1 = link_1_joint_->S();
	    const DMat<double> X21_S1 = X21_.transformMotionSubspace(S1);
	    const DMat<double>& S2 = link_2_joint_->S();
	    const DVec<double> v2_relative = S2 * q_dot[3];
	    const DMat<double> v2_rel_crm = generalMotionCrossMatrix(v2_relative);

	    S_.block<6, 1>(12, 0) = S1 * G_.block<1, 1>(2,0);
	    S_.block<6, 1>(12, 1) = S1 * G_.block<1, 1>(2, 1);
	    S_.block<6, 1>(18, 0) = X21_S1 * G_.block<1, 1>(2, 0) + S2 * G_.block<1, 1>(3, 0);
	    S_.block<6, 1>(18, 1) = X21_S1 * G_.block<1 ,1>(2, 1) + S2 * G_.block<1, 1>(3, 1);

	    // Given matrix abcd = [a b;c d] = G_.bottomRows(2) = -Kd.inv()*Ki,
	    // calculate a_dot, b_dot, c_dot, d_dot for S_ring_
	    Mat2<double> Ki, Kd, Ki_dot, Kd_dot;
	    vector<Eigen::MatrixBase<Mat2<double>>*> K = {&Ki, &Kd, &Ki_dot, &Kd_dot};
	    casadi_interface(arg, K, thd_kikd, thd_kikd_sparsity_out, thd_kikd_work);

	    const Mat2<double> Kd_inv = Kd.inverse();
	    Mat2<double> abcd_dot = -Kd_inv * Ki_dot + Kd_inv * Kd_dot * Kd_inv * Ki;

	    S_ring_.block<6, 1>(12, 0) = S1 * abcd_dot.block<1, 1>(0, 0);
	    S_ring_.block<6, 1>(12, 1) = S1 * abcd_dot.block<1, 1>(0, 1);
	    S_ring_.block<6, 1>(18, 0) = X21_S1 * abcd_dot.block<1, 1>(0, 0) +\
					 S2 * abcd_dot.block<1, 1>(1, 0) +\
					 (-v2_rel_crm * X21_S1) * G_.block<1, 1>(2, 0);
	    S_ring_.block<6, 1>(18, 1) = X21_S1 * abcd_dot.block<1, 1>(0, 1) +\
					 S2 * abcd_dot.block<1, 1>(1, 1) +\
					 (-v2_rel_crm * X21_S1) * G_.block<1, 1>(2, 1);

		// TODO(@MatthewChignoli): Issue if joint_vel is spanning
		vJ_ = S_ * joint_vel;
	}

	void TelloHipDifferential::computeSpatialTransformFromParentToCurrentCluster(
	    GeneralizedSpatialTransform &Xup) const
	{
	    if (Xup.getNumOutputBodies() != 4)
		    throw std::runtime_error("[TelloHipDifferential] Xup must have 24 rows");

	    Xup[0] = rotor_1_joint_->XJ() * rotor_1_.Xtree_;
	    Xup[1] = rotor_2_joint_->XJ() * rotor_2_.Xtree_;
	    Xup[2] = link_1_joint_->XJ() * link_1_.Xtree_;
	    Xup[3] = link_2_joint_->XJ() * link_2_.Xtree_ * Xup[2];
	}

    }

} // namespace grbda
