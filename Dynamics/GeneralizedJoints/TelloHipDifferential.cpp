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

	void TelloHipDifferential::updateKinematics(const DVec<double> &q, const DVec<double> &yd)
	{
	    if (q.size() != 4)
		    throw std::runtime_error("[TelloHipDifferential] Dimension of joint position must be 4");

	    Mat2<double> J_dy_2_dqd;
	    J_dy_2_dqd.setZero();
	    Vec2<double> arg_y = q.head<2>();
	    Vec2<double> arg_qd = q.tail<2>();
	    vector<double *> arg = {arg_y.data(), arg_qd.data()};
	    vector<double *> res = {J_dy_2_dqd.data()};
	    casadi_interface(arg, res, J_dy_2_dqd.size(), thd_J_dy_2_dqd, thd_J_dy_2_dqd_sparsity_out, thd_J_dy_2_dqd_work);

	    G_.bottomRows<2>() = J_dy_2_dqd;

	    DVec<double> q_dot = G_ * yd;

	    rotor_1_joint_->updateKinematics(q.segment<1>(0), q_dot.segment<1>(0));
	    rotor_2_joint_->updateKinematics(q.segment<1>(1), q_dot.segment<1>(1));
	    link_1_joint_->updateKinematics(q.segment<1>(2), q_dot.segment<1>(2));
	    link_2_joint_->updateKinematics(q.segment<1>(3), q_dot.segment<1>(3));

	    K_.leftCols<2>() = -G_.bottomRows<2>();

	    // Calculate g and k
	    Vec2<double> arg_y_dot = q_dot.head<2>();
	    Vec2<double> arg_qd_dot = q_dot.tail<2>();
	    arg = {arg_y.data(), arg_qd.data(), arg_y_dot.data(), arg_qd_dot.data()};
	    res = {g_.data()};
	    casadi_interface(arg, res, g_.size(), thd_g, thd_g_sparsity_out, thd_g_work);
	    res = {k_.data()};
	    casadi_interface(arg, res, k_.size(), thd_k, thd_k_sparsity_out, thd_k_work);

	    X21_ = link_2_joint_->XJ() * link_2_.Xtree_;

	    S_.block<6, 1>(12, 0) = link_1_joint_->S() * G_.block<1, 1>(2,0);
	    S_.block<6, 1>(12, 1) = link_1_joint_->S() * G_.block<1, 1>(2, 1);
	    S_.block<6, 1>(18, 0) = X21_.transformMotionSubspace(link_1_joint_->S()) * G_.block<1, 1>(2, 0) +
		    link_2_joint_->S() * G_.block<1, 1>(3, 0);
	    S_.block<6, 1>(18, 1) = X21_.transformMotionSubspace(link_1_joint_->S()) * G_.block<1 ,1>(2, 1) +
		    link_2_joint_->S() * G_.block<1, 1>(3, 1);

	    // Given matrix abcd = [a b;c d] = G_.bottomRows(2) = -Kd.inv()*Ki,
	    // calculate a_dot, b_dot, c_dot, d_dot for S_ring_
	    Mat2<double> Ki, Kd, Ki_dot, Kd_dot;
	    Ki.setZero();
	    Kd.setZero();
	    Ki_dot.setZero();
	    Kd_dot.setZero();
	    res = {Ki.data(), Kd.data(), Ki_dot.data(), Kd_dot.data()};
	    casadi_interface(arg, res, Ki.size(), thd_kikd, thd_kikd_sparsity_out, thd_kikd_work);
	    Mat2<double> abcd_dot = -Kd.inverse() * Ki_dot\
				    + Kd.inverse() * Kd_dot * Kd.inverse() * Ki;

	    DVec<double> v2_relative = link_2_joint_->S() * q_dot[3];
	    S_ring_.block<6, 1>(12, 0) = link_1_joint_->S() * abcd_dot.block<1, 1>(0, 0);
	    S_ring_.block<6, 1>(12, 1) = link_1_joint_->S() * abcd_dot.block<1, 1>(0, 1);
	    S_ring_.block<6, 1>(18, 0) = X21_.transformMotionSubspace(link_1_joint_->S())\
					 * abcd_dot.block<1, 1>(0, 0)\
					 + link_2_joint_->S() * abcd_dot.block<1, 1>(1, 0)\
					 + (-generalMotionCrossMatrix(v2_relative)\
					 * X21_.transformMotionSubspace(link_1_joint_->S()))\
					 * G_.block<1, 1>(2, 0);
	    S_ring_.block<6, 1>(18, 1) = X21_.transformMotionSubspace(link_1_joint_->S())\
					 * abcd_dot.block<1, 1>(0, 1)\
					 + link_2_joint_->S() * abcd_dot.block<1, 1>(1, 1)\
					 + (-generalMotionCrossMatrix(v2_relative)\
					 * X21_.transformMotionSubspace(link_1_joint_->S()))\
					 * G_.block<1, 1>(2, 1);

	    vJ_ = S_ * yd;
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
