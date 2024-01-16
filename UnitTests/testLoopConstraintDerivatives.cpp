#include "gtest/gtest.h"

#include "testHelpers.hpp"
#include "grbda/Dynamics/ClusterJoints/ClusterJointTypes.h"

using namespace grbda;

// TODO(@MatthewChignoli): Maybe we shouldn't call these file derivatives? We should just have separate files for testing loop constraints vs. cluster joints vs. full algos

// TODO(@MatthewChignoli): Unit test that makes sure
// phi(gamma(y)) = 0 for all y in the joint space
// q = Gy + g
// Where is the right place to put this unit test?

casadi::Function
createImplicitConstraintFunction(std::shared_ptr<LoopConstraint::Base<casadi::SX>> loop)
{
    using SX = casadi::SX;

    // Define symbolic variables
    SX cs_q_sym = SX::sym("q", loop->numSpanningPos(), 1);
    DVec<SX> q_sym(loop->numSpanningPos());
    casadi::copy(cs_q_sym, q_sym);

    SX cs_v_sym = SX::sym("qd", loop->numSpanningVel(), 1);
    DVec<SX> v_sym(loop->numSpanningVel());
    casadi::copy(cs_v_sym, v_sym);

    // Set state and update kinematics
    JointCoordinate<SX> joint_pos(q_sym, false);

    // Compute constraint violation
    DVec<SX> phi = loop->phi(joint_pos);
    SX cs_phi = casadi::SX(casadi::Sparsity::dense(phi.size(), 1));
    casadi::copy(phi, cs_phi);

    // Compute constraint jacobian
    SX cs_K = jacobian(cs_phi, cs_q_sym);

    // Compute constraint bias
    SX cs_Kdot = SX(cs_K.size1(), cs_K.size2());
    for (size_t i = 0; i < cs_K.size2(); i++)
    {
        cs_Kdot(casadi::Slice(), i) = jtimes(cs_K(casadi::Slice(), i), cs_q_sym, cs_v_sym);
    }
    SX cs_k = casadi::SX::mtimes(cs_Kdot, cs_v_sym);

    // Create casadi function
    std::vector<SX> args{cs_q_sym, cs_v_sym};
    std::vector<SX> res{cs_phi, cs_K, cs_k};
    casadi::Function implicitConstraintFunction("implicit", args, res);

    return implicitConstraintFunction;
}

// TODO(@MatthewChignoli): Template this test so that it is valid for all loop constraints?
// TODO(@MatthewChignoli): Also compare bias
// TODO(@MatthewChignoli): Where we will compare the explicit constraint jacobian? Same test? If yes, then it needs a new name. 
GTEST_TEST(LoopConstraintDerivatives, implicit)
{
    // TODO(@MatthewChignoli): add description of test

    double l1 = 1.4;
    double l2 = 1.1;
    double l3 = 1.7;
    double l4 = 1.2;

    // Create symbolic four bar loop constraint
    using SX = casadi::SX;
    using FourBarSX = LoopConstraint::FourBar<SX>;
    std::vector<SX> path1_lengths_sym{l1, l2};
    std::vector<SX> path2_lengths_sym{l3};
    Vec2<SX> offset_sym{l4, 0.};
    std::shared_ptr<FourBarSX> four_bar_sym = std::make_shared<FourBarSX>(path1_lengths_sym,
                                                                          path2_lengths_sym,
                                                                          offset_sym);
    casadi::Function implicitConstraintFunction = createImplicitConstraintFunction(four_bar_sym);

    // Create numerical four bar loop constraint
    using FourBar = LoopConstraint::FourBar<double>;
    std::vector<double> path1_lengths{l1, l2};
    std::vector<double> path2_lengths{l3};
    Vec2<double> offset{l4, 0.};
    std::shared_ptr<FourBar> four_bar_num = std::make_shared<FourBar>(path1_lengths,
                                                                      path2_lengths,
                                                                      offset);

    // Compare analytical vs. autodiff
    casadi::DM q_dm = random<casadi::DM>(3);
    DVec<double> q(3);
    casadi::copy(q_dm, q);

    casadi::DM v_dm = random<casadi::DM>(3);
    DVec<double> v(3);
    casadi::copy(v_dm, v);

    std::vector<casadi::DM> arg_ad = std::vector<casadi::DM>{q_dm, v_dm};
    std::vector<casadi::DM> res_ad = implicitConstraintFunction(arg_ad);
    DVec<double> phi_ad(2);
    casadi::copy(res_ad[0], phi_ad);
    DMat<double> K_ad(2, 3);
    casadi::copy(res_ad[1], K_ad);
    DVec<double> k_ad(2);
    casadi::copy(res_ad[2], k_ad);

    JointCoordinate<double> joint_pos(q, false);
    JointCoordinate<double> joint_vel(v, false);
    JointState<double> joint_state(joint_pos, joint_vel);
    DVec<double> phi = four_bar_num->phi(joint_pos);
    four_bar_num->updateJacobians(joint_pos);
    four_bar_num->updateBiases(joint_state);
    DMat<double> K = four_bar_num->K();
    DVec<double> k = four_bar_num->k();
    DMat<double> G = four_bar_num->G();
    DVec<double> g = four_bar_num->g();

    GTEST_ASSERT_LE((phi - phi_ad).norm(), 1e-12) << "phi:\n" << phi << "\nphi_ad:\n" << phi_ad;
    GTEST_ASSERT_LE((K - K_ad).norm(), 1e-12) << "K:\n" << K << "\nK_ad:\n" << K_ad;
    GTEST_ASSERT_LE((k - k_ad).norm(), 1e-12) << "k:\n" << k << "\nk_ad:\n" << k_ad;

    // Verify agreement between implicit and explicit jacobians and biases
    GTEST_ASSERT_LE((K * G).norm(), 1e-12) << "K*G:\n" << K * G;
    GTEST_ASSERT_LE((K * g - k).norm(), 1e-12) << "K*g - k:\n" << K * g - k;

}

// TODO(@MatthewChignoli): Let's think more clearly about the next steps
// We want the loop constraint to take a lambda function
// 
// Step 1: Add the fixed offset between the joints of link 1 and link 3
// What we really want to do is compare the four bar mechanism constructed with the URDF versus the four bar mechanism constructed manually. So the manual one will use the specialized loop constraint, while the URDF one will use the generic loop constraint
// So the next step is to manually create a robot that is or has a four bar mechanism
// Which means now the next step is to 
