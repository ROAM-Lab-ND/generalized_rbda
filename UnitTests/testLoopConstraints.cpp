#include "gtest/gtest.h"

#include <random>
#include "testHelpers.hpp"
#include "grbda/Dynamics/ClusterJoints/ClusterJointTypes.h"

using namespace grbda;

struct FourBarParameters
{
    Eigen::Vector<double, 4> link_lengths;
    int independent_coordinate;
    double offset_angle;
};

class fourBarLoopConstraintTestHelper
{
private:
    using SX = casadi::SX;
    using FourBar = LoopConstraint::FourBar<double>;
    using CasFcn = casadi::Function;
    using FourBarSX = LoopConstraint::FourBar<SX>;

    std::shared_ptr<FourBarSX> four_bar_sym_;

public:
    std::shared_ptr<FourBar> four_bar_num;

    struct
    {
        CasFcn K;
        CasFcn k;
    } casadi_fcns;

public:
    fourBarLoopConstraintTestHelper(const FourBarParameters& params) 
    {
        Eigen::Vector<double, 4> link_lengths = params.link_lengths;
        int independent_coordinate = params.independent_coordinate;
        double offset_angle = params.offset_angle;

        // Assign the differenent paths from the parent joint of the cluster to the constraint joint
        std::vector<SX> path1_lengths_sym, path2_lengths_sym;
        std::vector<double> path1_lengths, path2_lengths;
        for (int i = 0; i < 3; i++)
        {
            // TODO(@MatthewChignoli): For now, the root finder breaks down if you add links in an order other than link 1 and 2 in path1 and link 3 in path2
            // if (randomBool())
            if (i < 2)
            {
                path1_lengths_sym.push_back(link_lengths[i]);
                path1_lengths.push_back(link_lengths[i]);
            }
            else
            {
                path2_lengths_sym.push_back(link_lengths[i]);
                path2_lengths.push_back(link_lengths[i]);
            }
        }

        // Offset between fixed locations of joint 1 and joint 3
        Vec2<SX> offset_sym{link_lengths[3] * std::cos(offset_angle),
                            link_lengths[3] * std::sin(offset_angle)};
        Vec2<double> offset{link_lengths[3] * std::cos(offset_angle),
                            link_lengths[3] * std::sin(offset_angle)};

        // Create Loop Constraints
        four_bar_sym_ = std::make_shared<FourBarSX>(path1_lengths_sym, path2_lengths_sym,
                                                    offset_sym, independent_coordinate);
        four_bar_num = std::make_shared<FourBar>(path1_lengths, path2_lengths,
                                                 offset, independent_coordinate);

        // Create casadi functions
        four_bar_sym_->createRandomStateHelpers();
        createImplictConstraintFcns();
    }

    std::vector<casadi::DM> randomJointState() const
    {
        // Find a feasible joint position
        double range = 6.28;
        casadi::DM q_ind = range * (2. * casadi::DM::rand(four_bar_sym_->numIndependentPos()) - 1.);
        casadi::DM q_dep_guess = range * (2. * casadi::DM::rand(2) - 1.);
        casadi::DMDict arg;
        arg["p"] = q_ind;
        arg["x0"] = q_dep_guess;
        casadi::DM q_dep = four_bar_sym_->random_state_helpers_.phi_root_finder(arg).at("x");

        casadi::DM q_dm;
        switch (four_bar_sym_->independent_coordinate())
        {
        case 0:
            q_dm = casadi::DM::vertcat({q_ind, q_dep});
            break;
        case 1:
            q_dm = casadi::DM::vertcat({q_dep(0), q_ind, q_dep(1)});
            break;
        case 2:
            q_dm = casadi::DM::vertcat({q_dep(0), q_dep(1), q_ind});
            break;
        }
        DVec<double> q(four_bar_sym_->numSpanningPos());
        casadi::copy(q_dm, q);
        JointCoordinate<double> joint_pos(q, true);

        // Compute the explicit constraint jacobians
        four_bar_num->updateJacobians(joint_pos);
        DMat<double> G = four_bar_num->G();

        // Compute a valid joint velocity
        DVec<double> v = G * DVec<double>::Random(four_bar_num->numIndependentVel());
        casadi::DM v_dm = casadi::DM::zeros(four_bar_sym_->numSpanningVel(), 1);
        casadi::copy(v, v_dm);
        JointCoordinate<double> joint_vel(v, true);

        // Compute the constraint biases
        four_bar_num->updateBiases(JointState<double>(joint_pos, joint_vel));
        DVec<double> g = four_bar_num->g();

        // Compute a valid joint acceleration
        DVec<double> vd = G * DVec<double>::Random(four_bar_num->numIndependentVel()) + g;
        casadi::DM vd_dm = casadi::DM::zeros(four_bar_sym_->numSpanningVel(), 1);
        casadi::copy(vd, vd_dm);

        return std::vector<casadi::DM>{q_dm, v_dm, vd_dm};
    }

private:
    void createImplictConstraintFcns()
    {
        // Define symbolic variables
        SX cs_q_sym = SX::sym("q", four_bar_sym_->numSpanningPos(), 1);
        DVec<SX> q_sym(four_bar_sym_->numSpanningPos());
        casadi::copy(cs_q_sym, q_sym);

        SX cs_v_sym = SX::sym("v", four_bar_sym_->numSpanningVel(), 1);

        // Set state and update kinematics
        JointCoordinate<SX> joint_pos(q_sym, true);

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
            casadi_fcns.K = casadi::Function("K", args, res);
        }

        {
            std::vector<SX> args{cs_q_sym, cs_v_sym};
            std::vector<SX> res{cs_k};
            casadi_fcns.k = casadi::Function("k", args, res);
        }
    }
};

GTEST_TEST(LoopConstraint, FourBar)
{
    // This test:
    // 1) Verifies that the method of generating random joint states results in feasible states
    // that satisfy the implicit loop constraints
    // 2) Verifies that the analytical method of computing the implicit loop constraint jacobian
    // and bias returns the same result as the autodiff method
    // 3) Verifies that the implicit and explicit constraint jacobians and biases are consistent

    // TODO(@MatthewChignoli)
    // - change the order of link paths (or enforce they always have the same order?)

    std::vector<FourBarParameters> four_bars;
    four_bars.push_back({Eigen::Vector<double, 4>{0.096, .042 - .011, 0.096, .042 - .011}, 0, 0.});
    four_bars.push_back({Eigen::Vector<double, 4>{1., 1., 1., 1.}, 0, 1.0});
    four_bars.push_back({Eigen::Vector<double, 4>{5., 12., 12., 10.}, 0, 0.});

    four_bars.push_back({Eigen::Vector<double, 4>{1., 1., 1., 1.}, 1, 0.});
    four_bars.push_back({Eigen::Vector<double, 4>{5., 12., 12., 10.}, 1, 2.0});

    four_bars.push_back({Eigen::Vector<double, 4>{0.096, .042 - .011, 0.096, .042 - .011}, 2, 0.});
    four_bars.push_back({Eigen::Vector<double, 4>{1., 1., 1., 1.}, 2, 0.5});
    four_bars.push_back({Eigen::Vector<double, 4>{4., 8.25, 8.25, 2.}, 2, 0.});
    four_bars.push_back({Eigen::Vector<double, 4>{4., 8.25, 8.25, 2.}, 2, -1.3});

    for (const auto &params : four_bars)
    {
        fourBarLoopConstraintTestHelper helper(params);
        std::shared_ptr<LoopConstraint::FourBar<double>> four_bar_num = helper.four_bar_num;

        for (int i = 0; i < 5; i++)
        {
            std::vector<casadi::DM> q_v_vd_dm = helper.randomJointState();
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

            JointCoordinate<double> joint_pos(q, true);
            JointCoordinate<double> joint_vel(v, true);
            JointState<double> joint_state(joint_pos, joint_vel);

            DVec<double> phi = four_bar_num->phi(joint_pos);
            four_bar_num->updateJacobians(joint_pos);
            DMat<double> K = four_bar_num->K();
            four_bar_num->updateBiases(joint_state);
            DVec<double> k = four_bar_num->k();

            // Verify that the constraints are satisfied
            GTEST_ASSERT_LE(phi.norm(), 1e-8) << "phi:\n"
                                              << phi;

            GTEST_ASSERT_LE((K * v).norm(), 1e-8) << "K*v:\n"
                                                  << K * v;

            GTEST_ASSERT_LE((K * vd - k).norm(), 1e-8) << "K*vd + k:\n"
                                                       << K * vd + k;

            // Compare analytical vs. autodiff implicit constraints
            DMat<double> K_ad(2, 3);
            casadi::copy(helper.casadi_fcns.K(q_dm)[0], K_ad);
            GTEST_ASSERT_LE((K - K_ad).norm(), 1e-10) << "K:\n"
                                                      << K << "\nK_ad:\n"
                                                      << K_ad;

            DVec<double> k_ad(2);
            casadi::copy(helper.casadi_fcns.k(state_dm)[0], k_ad);
            GTEST_ASSERT_LE((k - k_ad).norm(), 1e-10) << "k:\n"
                                                      << k << "\nk_ad:\n"
                                                      << k_ad;

            // Verify agreement between implicit and explicit jacobians and biases
            DMat<double> G = four_bar_num->G();
            GTEST_ASSERT_LE((K * G).norm(), 1e-10) << "K*G:\n"
                                                   << K * G;

            DVec<double> g = four_bar_num->g();
            GTEST_ASSERT_LE((K * g - k).norm(), 1e-10) << "K*g:\n"
                                                       << K * g << "\nk:\n"
                                                       << k;
        }
    }
}

// TODO(@MatthewChignoli): How to test this?
GTEST_TEST(LoopConstraint, GenericImplicit)
{
    // This test creates an abritrary generic implicit loop constraint and then verifies that
    // the implicit and explicit constraint jacobians and biases are consistent

    using SX = casadi::SX;

    // Define a generic implicit loop constraint
    std::function<DVec<SX>(const JointCoordinate<SX> &)> phi_fcn =
        [](const JointCoordinate<SX> &joint_pos)
    {
        DVec<SX> phi(2);
        phi(0) = joint_pos[0] + joint_pos[1] + joint_pos[2];
        phi(1) = joint_pos[0] * joint_pos[1] + joint_pos[0] * joint_pos[1] * joint_pos[2];
        return phi;
    };

    // Possible options for the independent coordinate
    std::vector<std::vector<bool>> ind_coord_samples{
        {true, false, false},
        {false, true, false},
        {false, false, true}};

    for (const std::vector<bool> &is_coordinate_independent : ind_coord_samples)
    {
        LoopConstraint::GenericImplicit<double> generic_implicit(is_coordinate_independent,
                                                                 phi_fcn);

        // Kinematics
        int state_dim = is_coordinate_independent.size();
        DVec<double> q = DVec<double>::Random(state_dim);
        DVec<double> v = DVec<double>::Random(state_dim);
        JointCoordinate<double> joint_pos = JointCoordinate<double>(q, true);
        JointCoordinate<double> joint_vel = JointCoordinate<double>(v, true);
        JointState<double> joint_state(joint_pos, joint_vel);
        generic_implicit.updateJacobians(joint_pos);
        generic_implicit.updateBiases(joint_state);

        // Verify agreement between implicit and explicit jacobians and biases
        const DMat<double> &K = generic_implicit.K();
        const DVec<double> &k = generic_implicit.k();
        const DMat<double> &G = generic_implicit.G();
        const DVec<double> &g = generic_implicit.g();
        GTEST_ASSERT_LE((K * G).norm(), 1e-10);
        GTEST_ASSERT_LE((K * g - k).norm(), 1e-10);

        // Make sure jacobians and biases are not trivially zero
        if (q.norm() > 1e-8)
        {
            GTEST_ASSERT_GT(K.norm(), 1e-8);
            GTEST_ASSERT_GT(G.norm(), 1e-8);
            GTEST_ASSERT_GT(k.norm(), 1e-8);
            GTEST_ASSERT_GT(g.norm(), 1e-8);
        }
    }

    // TODO(@MatthewChignoli): Unit test specialized loop constraints against generic loop constraints
}
