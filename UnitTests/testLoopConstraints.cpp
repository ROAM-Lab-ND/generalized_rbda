#include "gtest/gtest.h"

#include "testHelpers.hpp"
#include "grbda/Dynamics/ClusterJoints/ClusterJointTypes.h"

using namespace grbda;

// TODO(@MatthewChignoli): For now, it is specialized for the four bar
// TODO(@MatthewChignoli): Template this test so that it is valid for all loop constraints?
class LoopConstraintDerivatives : public ::testing::Test
{
protected:
    using SX = casadi::SX;
    using FourBarSX = LoopConstraint::FourBar<SX>;
    using FourBar = LoopConstraint::FourBar<double>;
    using CasFcn = casadi::Function;

    std::shared_ptr<FourBarSX> four_bar_sym_;
    std::shared_ptr<FourBar> four_bar_num_;

    struct
    {
        CasFcn phiRootFinder;
        CasFcn K;
        CasFcn k;
    } casadi_fcns_;

private:
    // TODO(@MatthewChignoli): Randomize these? How do we know these are feasible?
    const double l1 = 1.4;
    const double l2 = 1.1;
    const double l3 = 1.7;
    const double l4 = 1.2;

protected:
    void SetUp() override
    {
        std::vector<SX> path1_lengths_sym{l1, l2};
        std::vector<SX> path2_lengths_sym{l3};
        Vec2<SX> offset_sym{l4, 0.};
        four_bar_sym_ = std::make_shared<FourBarSX>(path1_lengths_sym, path2_lengths_sym,
                                                    offset_sym);

        std::vector<double> path1_lengths{l1, l2};
        std::vector<double> path2_lengths{l3};
        Vec2<double> offset{l4, 0.};
        four_bar_num_ = std::make_shared<FourBar>(path1_lengths, path2_lengths, offset);

        createPhiRootFinder();
        createImplictConstraintFcns();
    }

    void TearDown() override
    {
    }

    std::vector<casadi::DM> randomJointState() const
    {
        // Find a feasible joint position
        casadi::DMDict arg;
        arg["x0"] = random<casadi::DM>(four_bar_sym_->numSpanningPos());
        casadi::DM q_dm = casadi_fcns_.phiRootFinder(arg).at("x");
        DVec<double> q(four_bar_sym_->numSpanningPos());
        casadi::copy(q_dm, q);
        JointCoordinate<double> joint_pos(q, false);

        // Compute the explicit constraint jacobians
        four_bar_num_->updateJacobians(joint_pos);
        DMat<double> G = four_bar_num_->G();

        // Compute a valid joint velocity
        DVec<double> v = G * DVec<double>::Random(four_bar_num_->numIndependentVel());
        casadi::DM v_dm = casadi::DM::zeros(four_bar_sym_->numSpanningVel(), 1);
        casadi::copy(v, v_dm);
        JointCoordinate<double> joint_vel(v, false);

        // Compute the constraint biases
        four_bar_num_->updateBiases(JointState<double>(joint_pos, joint_vel));
        DVec<double> g = four_bar_num_->g();

        // Compute a valid joint acceleration
        DVec<double> vd = G * DVec<double>::Random(four_bar_num_->numIndependentVel()) + g;
        casadi::DM vd_dm = casadi::DM::zeros(four_bar_sym_->numSpanningVel(), 1);
        casadi::copy(vd, vd_dm);

        return std::vector<casadi::DM>{q_dm, v_dm, vd_dm};
    }

private:
    void createPhiRootFinder()
    {
        // Define symbolic variables
        SX cs_q_sym = SX::sym("q", four_bar_sym_->numSpanningPos(), 1);
        DVec<SX> q_sym(four_bar_sym_->numSpanningPos());
        casadi::copy(cs_q_sym, q_sym);

        // Compute constraint violation
        JointCoordinate<SX> joint_pos(q_sym, false);
        DVec<SX> phi = four_bar_sym_->phi(joint_pos);
        SX f = phi.transpose() * phi;

        // Create root finder nlp
        casadi::SXDict nlp = {{"x", cs_q_sym}, {"f", f}};
        casadi::Dict opts = {};
        opts.insert(std::make_pair("print_time", false));
        opts.insert(std::make_pair("ipopt.linear_solver", "ma27"));
        opts.insert(std::make_pair("ipopt.print_level", 0));
        casadi_fcns_.phiRootFinder = casadi::nlpsol("solver", "ipopt", nlp, opts);
    }

    void createImplictConstraintFcns()
    {
        // Define symbolic variables
        SX cs_q_sym = SX::sym("q", four_bar_sym_->numSpanningPos(), 1);
        DVec<SX> q_sym(four_bar_sym_->numSpanningPos());
        casadi::copy(cs_q_sym, q_sym);

        SX cs_v_sym = SX::sym("v", four_bar_sym_->numSpanningVel(), 1);
        DVec<SX> v_sym(four_bar_sym_->numSpanningVel());
        casadi::copy(cs_v_sym, v_sym);

        // Set state and update kinematics
        JointCoordinate<SX> joint_pos(q_sym, false);

        // Compute constraint violation
        DVec<SX> phi = four_bar_sym_->phi(joint_pos);
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
        SX cs_k = -casadi::SX::mtimes(cs_Kdot, cs_v_sym);

        // Create casadi functions
        {
            std::vector<SX> args{cs_q_sym};
            std::vector<SX> res{cs_K};
            casadi_fcns_.K = casadi::Function("K", args, res);
        }

        {
            std::vector<SX> args{cs_q_sym, cs_v_sym};
            std::vector<SX> res{cs_k};
            casadi_fcns_.k = casadi::Function("k", args, res);
        }
    }
};

TEST_F(LoopConstraintDerivatives, implicitConstraintAutoDiff)
{
    // This test:
    // 1) Verifies that the method of generating random joint states results in feasible states 
    // that satisfy the implicit loop constraints
    // 2) Verifies that the analytical method of computing the implicit loop constraint jacobian 
    // and bias returns the same result as the autodiff method
    // 3) Verifies that the implicit and explicit constraint jacobians and biases are consistent

    for (int i = 0; i < 25; i++)
    {
        std::vector<casadi::DM> q_v_vd_dm = randomJointState();
        casadi::DM q_dm = q_v_vd_dm[0];
        casadi::DM v_dm = q_v_vd_dm[1];
        casadi::DM vd_dm = q_v_vd_dm[2];
        std::vector<casadi::DM> state_dm{q_dm, v_dm};

        DVec<double> q(3);
        casadi::copy(q_dm, q);

        DVec<double> v(3);
        casadi::copy(v_dm, v);

        DVec<double> vd(3);
        casadi::copy(vd_dm, vd);

        JointCoordinate<double> joint_pos(q, false);
        JointCoordinate<double> joint_vel(v, false);
        JointState<double> joint_state(joint_pos, joint_vel);

        DVec<double> phi = four_bar_num_->phi(joint_pos);
        four_bar_num_->updateJacobians(joint_pos);
        DMat<double> K = four_bar_num_->K();
        four_bar_num_->updateBiases(joint_state);
        DVec<double> k = four_bar_num_->k();

        // Verify that the constraints are satisfied
        GTEST_ASSERT_LE(phi.norm(), 1e-8) << "phi:\n"
                                          << phi;

        GTEST_ASSERT_LE((K * v).norm(), 1e-8) << "K*v:\n"
                                              << K * v;

        GTEST_ASSERT_LE((K * vd - k).norm(), 1e-8) << "K*vd + k:\n"
                                                   << K * vd + k;

        // Compare analytical vs. autodiff implicit constraints
        DMat<double> K_ad(2, 3);
        casadi::copy(casadi_fcns_.K(q_dm)[0], K_ad);
        GTEST_ASSERT_LE((K - K_ad).norm(), 1e-12) << "K:\n"
                                                  << K << "\nK_ad:\n"
                                                  << K_ad;

        DVec<double> k_ad(2);
        casadi::copy(casadi_fcns_.k(state_dm)[0], k_ad);
        GTEST_ASSERT_LE((k - k_ad).norm(), 1e-12) << "k:\n"
                                                  << k << "\nk_ad:\n"
                                                  << k_ad;

        // Verify agreement between implicit and explicit jacobians and biases
        DMat<double> G = four_bar_num_->G();
        GTEST_ASSERT_LE((K * G).norm(), 1e-12) << "K*G:\n"
                                               << K * G;

        DVec<double> g = four_bar_num_->g();
        GTEST_ASSERT_LE((K * g - k).norm(), 1e-12) << "K*g:\n"
                                                   << K * g << "\nk:\n"
                                                   << k;
    }
}
