#include "gtest/gtest.h"

#include "testHelpers.hpp"
#include "grbda/Dynamics/ClusterJoints/ClusterJointTypes.h"

using namespace grbda;

casadi::Function
createImplicitConstraintFunction(std::shared_ptr<LoopConstraint::Base<casadi::SX>> loop)
{
    using SX = casadi::SX;

    // Define symbolic variables
    SX cs_q_sym = SX::sym("q", loop->numSpanningPos(), 1);
    DVec<SX> q_sym(loop->numSpanningPos());
    casadi::copy(cs_q_sym, q_sym);

    // Set state and update kinematics
    JointCoordinate<SX> joint_pos(q_sym, false);

    // Compute constraint violation
    DVec<SX> phi = loop->phi(joint_pos);
    SX cs_phi = casadi::SX(casadi::Sparsity::dense(phi.size(), 1));
    casadi::copy(phi, cs_phi);

    // Compute constraint jacobian
    SX cs_K = jacobian(cs_phi, cs_q_sym);
    // DMat<SX> K(cs_K.size1(), cs_K.size2());
    // casadi::copy(cs_K, K);

    // Create casadi function
    std::vector<SX> args{cs_q_sym};
    std::vector<SX> res{cs_phi, cs_K};
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

    std::vector<casadi::DM> res_ad = implicitConstraintFunction(q_dm);
    DVec<double> phi_ad(2);
    casadi::copy(res_ad[0], phi_ad);
    DMat<double> K_ad(2, 3);
    casadi::copy(res_ad[1], K_ad);

    JointCoordinate<double> joint_pos(q, false);
    DVec<double> phi = four_bar_num->phi(joint_pos);
    four_bar_num->updateJacobians(joint_pos);
    DMat<double> K = four_bar_num->K();

    GTEST_ASSERT_LE((phi - phi_ad).norm(), 1e-12) << "phi: " << phi << "\nphi_ad: " << phi_ad;
    GTEST_ASSERT_LE((K - K_ad).norm(), 1e-12) << "K: " << K << "\nK_ad: " << K_ad;
}

// TODO(@MatthewChignoli): Let's think more clearly about the next steps
// We want the loop constraint to take a lambda function
// 
// Step 1: Add the fixed offset between the joints of link 1 and link 3
// What we really want to do is compare the four bar mechanism constructed with the URDF versus the four bar mechanism constructed manually. So the manual one will use the specialized loop constraint, while the URDF one will use the generic loop constraint
// So the next step is to manually create a robot that is or has a four bar mechanism
// Which means now the next step is to 
